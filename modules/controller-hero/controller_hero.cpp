#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <compensator/compensator.h>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller-base/controller_factory.h"
#include "predictor-outpost/predictor_outpost.h"
#include "controller_hero.h"
#include "predictor-armor/predictor_armor.h"

/**
 * \warning Controller registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ControllerRegistry<HeroController>
        HeroController::hero_controller_registry_("hero");

bool HeroController::Initialize() {
    // Common initialization.
    if (!Controller::Initialize("hero"))
        return false;
    controller_hero_debug_.Initialize(CmdlineArgParser::Instance().DebugShowImage());
    // Initialize the outpost detector.
    if (outpost_predictor_.Initialize("hero"))
        LOG(INFO) << "Outpost predictor initialize successfully!";
    else
        LOG(ERROR) << "Outpost predictor initialize unsuccessfully!";

    LOG(INFO) << "Hero controller is ready.";
    return true;
}

void HeroController::Run() {
    cv::Rect ROI;
    sleep(2);
    ArmorPredictor armor_predictor(Entity::kRed, "hero");
    while (!exit_signal_) {
        static std::chrono::steady_clock::time_point time = std::chrono::steady_clock::now();

        if (!GetImage<false>())
            continue;

        ReceiveSerialData();

        boxes_ = armor_detector_(frame_.image);

        if (CmdlineArgParser::Instance().RunModeOutpost() ||
            receive_packet_.mode == AimModes::kOutPost) {

            BboxToArmor(Armor::ArmorSize::kSmall);
            for (auto &armor: armors_) {
                DLOG(INFO) << "armor";
                DLOG(INFO) << armor.TranslationVectorWorld();
            }

            battlefield_ = Battlefield(frame_.time_stamp,
                                       receive_packet_.bullet_speed,
                                       receive_packet_.yaw_pitch_roll,
                                       armors_, receive_packet_.self_speed);

            outpost_predictor_.SetColor(receive_packet_.color);
//            send_packet_ = outpost_predictor_.OldRun(battlefield_);
            send_packet_ = outpost_predictor_.Run(battlefield_, receive_packet_.bullet_speed,frame_.image.size,
                                                  receive_packet_.yaw_pitch_roll, time,
                                                                 receive_packet_.armor_kind);
            auto roi = outpost_predictor_.GetROI(frame_.image);
            armor_detector_.UpdateROI(roi);
            controller_hero_debug_.DrawOutpostData(frame_.image,
                                                   roi,
                                                   boxes_,
                                                   &outpost_predictor_,
                                                   image_provider_->IntrinsicMatrix(),
                                                   frame_.image.size,
                                                   "Hero Run",
                                                   1);

        } else {

            BboxToArmor();
            for (auto &armor: armors_) {
                DLOG(INFO) << "armor";
                DLOG(INFO) << armor.TranslationVectorWorld();
            }
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_, receive_packet_.self_speed);
            send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size, receive_packet_.color);
            /*
             *  hero don't auto fire except shooting spin outpost
             */
            if (send_packet_.fire == 1) {
                send_packet_.fire = 0;
                send_packet_.check_sum -= 1;
            }
            std::ostringstream ss_yaw, ss_pitch;
            ss_yaw << std::fixed << std::setprecision(6) << send_packet_.yaw;
            ss_pitch << std::fixed << std::setprecision(6) << send_packet_.pitch;
            cv::putText(frame_.image, "yaw " + ss_yaw.str(),
                        cv::Point(0, 72), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 192, 0));
            cv::putText(frame_.image, "pitch " + ss_pitch.str(),
                        cv::Point(0, 96), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 192, 0));
            controller_hero_debug_.DrawArmorDetection(frame_.image,
                                                      {},
                                                      boxes_,
                                                      &armor_predictor,
                                                      image_provider_->IntrinsicMatrix(),
                                                      frame_.image.size,
                                                      "Hero Run",
                                                      1);
        }
//


        if (controller_hero_debug_.GetKey() == 'q')
            break;

        SendSerialData();

        boxes_.clear();
        armors_.clear();
        time = std::chrono::steady_clock::now();
        CountPerformanceData();
    }

    if (CmdlineArgParser::Instance().RunWithSerial())
        serial_->StopCommunication();

    image_provider_.reset();
}
