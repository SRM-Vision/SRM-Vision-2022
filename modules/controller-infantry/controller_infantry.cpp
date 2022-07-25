#include <opencv2/imgproc.hpp>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller-base/controller_factory.h"
#include "detector-rune-network/detector_rune_network.h"
#include "predictor-rune/predictor-rune.h"
#include "predictor-rune-kalman/predictor_rune_kalman.h"
#include "predictor-armor/predictor_armor.h"
#include "controller_infantry.h"
#include "controller_infantry_debug.h"
#include "compensator/compensator.h"

/**
 * \warning Controller registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ControllerRegistry<InfantryController>
        InfantryController::infantry_controller_registry_("infantry");

bool InfantryController::Initialize() {
    // Common initialization.
    if (!Controller::Initialize("infantry"))
        return false;

    // Initialize painter.
    controller_infantry_debug_.Initialize(CmdlineArgParser::Instance().DebugShowImage());
//    rune_predictor_kalman_.Initialize();

    // Initialize Rune module.
    Frame init_frame;
    image_provider_->GetFrame(init_frame);
    if (rune_detector_network_.Initialize("../assets/models/rune_detector_model.onnx"))
        LOG(INFO) << "Rune detector initialize successfully!";
    else {
        LOG(ERROR) << "Rune detector initialize failed.";
        return false;
    }
    rune_predictor_.Initialize("../config/infantry/rune-predictor-param.yaml");

    LOG(INFO) << "Infantry controller is ready.";
    return true;
}

void InfantryController::Run() {
    sleep(2);
    ArmorPredictor armor_predictor{Entity::kBlue, "infantry"};
    while (!exit_signal_) {
        if (!GetImage<true>())
            continue;

        ReceiveSerialData();

        if (CmdlineArgParser::Instance().RuneModeRune()
        || receive_packet_.mode == AimModes::kSmallRune
        || receive_packet_.mode == AimModes::kBigRune) {
            receive_packet_.mode = kBigRune;
            power_rune_ = rune_detector_network_.Run(receive_packet_.color, frame_);
            send_packet_ = rune_predictor_.Run(power_rune_,
                                               receive_packet_.mode,
                                               receive_packet_.bullet_speed,
                                               receive_packet_.yaw_pitch_roll[0],
                                               receive_packet_.yaw_pitch_roll[1]);
//            send_packet_ = rune_predictor_kalman_.Run(power_rune_, kBigRune, 30);
//            cv::Mat draw_image = frame_.image.clone();
//            cv::circle(draw_image, cv::Point2f(send_packet_.yaw, send_packet_.pitch), 2, {255, 255}, 4);
//            cv::imshow("image", draw_image);
            controller_infantry_debug_.DrawAutoAimRune(frame_.image, &rune_predictor_, "detector rune network", 1);
        }else{
            boxes_ = armor_detector_(frame_.image);

            BboxToArmor();
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_, receive_packet_.self_speed);

            send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size, receive_packet_.color);
            controller_infantry_debug_.DrawAutoAimArmor(frame_.image,
                                                          boxes_,
                                                          &armor_predictor,
                                                          image_provider_->IntrinsicMatrix(),
                                                          frame_.image.size,
                                                          "Infantry Run",
                                                          1);

//            Compensator::Instance().Offset(send_packet_.pitch, battlefield_.BulletSpeed(), send_packet_.check_sum,
//                                           armor_predictor.GetTargetDistance(), kNormal);
        }



        if (ControllerInfantryDebug::GetKey() == 'q')
            break;

        SendSerialData();

        boxes_.clear();
        armors_.clear();

        CountPerformanceData();
    }

    // Exit.
    if (CmdlineArgParser::Instance().RunWithSerial())
        serial_->StopCommunication();

    image_provider_.reset();
}
