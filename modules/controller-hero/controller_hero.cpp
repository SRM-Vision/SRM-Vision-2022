#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <compensator/compensator.h>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller-base/controller_factory.h"
#include "predictor-outpost/predictor_outpost.h"
#include "controller_hero.h"
#include "predictor-armor/predictor_armor.h"
//#include "predictor-outpost/outpost_measure.h"


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

    // Initialize the outpost detector.
    if (outpost_predictor_.Initialize())
        LOG(INFO) << "Outpost predictor initialize successfully!";
    else
        LOG(ERROR) << "Outpost predictor initialize unsuccessfully!";

    LOG(INFO) << "Hero controller is ready.";
    return true;
}

void HeroController::Run() {
    sleep(2);
    ArmorPredictor armor_predictor(Entity::kBlue, "hero");
    while (!exit_signal_) {
        auto time = std::chrono::steady_clock::now();

        if (!GetImage<true>())
            continue;

        RunGimbal();

        debug::Painter::Instance()->UpdateImage(frame_.image);

        boxes_ = armor_detector_(frame_.image);
        for (auto &box: boxes_)
            DLOG(INFO) << "CONFIDENCE:  " << box.confidence;
        // if(receive_packet_.mode == kOutPost)
        if (CmdlineArgParser::Instance().RunModeOutpost()) {

            BboxToArmor(Armor::ArmorSize::kSmall);
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_);
            // outpost_measure_.Run(battlefield_,receive_packet_.color,receive_packet_.bullet_speed);

            outpost_predictor_.SetColor(receive_packet_.color);
            send_packet_ = outpost_predictor_.Run(battlefield_, receive_packet_.bullet_speed);

            if (send_packet_.fire)
                debug::Painter::Instance()->DrawText("Fire", {50, 50}, cv::Scalar(100, 255, 100), 2);
            debug::Painter::Instance()->ShowImage("ARMOR DETECT", 1);
        } else {
            /// TODO mode switch
            BboxToArmor();
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_);

            if (CmdlineArgParser::Instance().RunWithSerial()) {
                armor_predictor.SetColor(receive_packet_.color);
                send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size, receive_packet_.bullet_speed);
            } else
                send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size);

            if (receive_packet_.mode != AimModes::kOutPost)
                send_packet_.fire = 0;

            //armor_predictor.GetROI(ROI,frame_.image);
            for (const auto &box: boxes_) {
                debug::Painter::Instance()->DrawRotatedRectangle(box.points[0],
                                                                 box.points[1],
                                                                 box.points[2],
                                                                 box.points[3],
                                                                 cv::Scalar(0, 255, 0), 2);
                debug::Painter::Instance()->DrawText(std::to_string(box.id), box.points[0], 255, 2);
                debug::Painter::Instance()->DrawPoint(armors_.front().Center(), cv::Scalar(100, 255, 100), 2, 2);
            }
            debug::Painter::Instance()->DrawPoint(armor_predictor.ShootPointInPic(image_provider_->IntrinsicMatrix(),
                                                                                  frame_.image.size),
                                                  cv::Scalar(0, 0, 255), 1, 10);
            debug::Painter::Instance()->ShowImage("ARMOR DETECT", 1);
        }
//
//        Compensator::Instance().Offset(send_packet_.pitch, send_packet_.yaw,
//                                       receive_packet_.bullet_speed, send_packet_.check_sum,
//                                       armor_predictor.GetTargetDistance(),
//                                       receive_packet_.mode);

        auto key = cv::waitKey(5) & 0xff;
        if (key == 'q')
            break;
        else if (key == 's')
            ArmorPredictorDebug::Instance().Save();

        if (CmdlineArgParser::Instance().RunWithSerial()) {
            CheckSum();
            serial_->SendData(send_packet_, std::chrono::milliseconds(5));
        }
        boxes_.clear();
        armors_.clear();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
                                                                              - time);

        DLOG(INFO) << "FPS: " << 1000.0 / double(duration.count());
//        cv::waitKey(0);
    }

    // exit.
    if (CmdlineArgParser::Instance().RunWithGimbal())
        serial_->StopCommunication();

    image_provider_.reset();
}
