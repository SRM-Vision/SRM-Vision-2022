#include <opencv2/imgproc.hpp>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller-base/controller_factory.h"
#include "detector-rune/detector_rune.h"
#include "detector-rune-network/detector_rune_network.h"
#include "predictor-rune/predictor-rune.h"
#include "predictor-armor/predictor_armor.h"
#include "controller_infantry5.h"
#include "controller_infantry5_debug.h"
#include "compensator/compensator.h"

/**
 * \warning Controller registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ControllerRegistry<Infantry5Controller>
        Infantry5Controller::infantry5_controller_registry_("infantry5");

bool Infantry5Controller::Initialize() {
    // Common initialization.
    if (!Controller::Initialize("infantry5"))
        return false;

    // Initialize painter.
    controller_infantry5_debug_.Initialize(CmdlineArgParser::Instance().DebugShowImage());

    // Initialize Rune module.
    Frame init_frame;
    image_provider_->GetFrame(init_frame);
    if (rune_detector_.Initialize("../config/infantry5/rune-detector-param.yaml"))
        LOG(INFO) << "Rune detector initialize successfully!";
    else {
        LOG(ERROR) << "Rune detector initialize failed.";
        return false;
    }
    rune_detector_network_.Initialize("../assets/models/rune_detector_model.onnx");
    rune_predictor_.Initialize("../config/infantry/rune-predictor-param.yaml");

    LOG(INFO) << "Infantry controller is ready.";
    return true;
}

void Infantry5Controller::Run() {
    sleep(2);
    ArmorPredictor armor_predictor{Entity::kBlue, "infantry5"};
    while (!exit_signal_) {
        if (!GetImage<true>())
            continue;

        ReceiveSerialData();

        if (CmdlineArgParser::Instance().RuneModeRune()
        || receive_packet_.mode == AimModes::kSmallRune
        || receive_packet_.mode == AimModes::kBigRune) {
            power_rune_ = rune_detector_network_.Run(receive_packet_.color, frame_);
//            power_rune_ = rune_detector_.Run(receive_packet_.color, frame_);
            send_packet_ = rune_predictor_.Run(power_rune_,
                                               receive_packet_.mode,
                                               receive_packet_.bullet_speed,
                                               receive_packet_.yaw_pitch_roll[0],
                                               receive_packet_.yaw_pitch_roll[1]);
            controller_infantry5_debug_.DrawAutoAimRune(frame_.image, &rune_predictor_, "detector rune network", 1);
        }else{
            boxes_ = armor_detector_(frame_.image);

            BboxToArmor();
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_, receive_packet_.self_speed);
            send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size, receive_packet_.color);
            controller_infantry5_debug_.DrawAutoAimArmor(frame_.image,
                                                          boxes_,
                                                          &armor_predictor,
                                                          image_provider_->IntrinsicMatrix(),
                                                          frame_.image.size,
                                                          "Infantry Run",
                                                          1);

//            Compensator::Instance().Offset(send_packet_.pitch, battlefield_.BulletSpeed(), send_packet_.check_sum,
//                                           armor_predictor.GetTargetDistance(), kNormal);
        }

        if (ControllerInfantry5Debug::GetKey() == 'q')
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
