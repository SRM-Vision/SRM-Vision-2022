#include <opencv2/imgproc.hpp>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller-base/controller_factory.h"
#include "predictor-rune/predictor-rune.h"
#include "predictor-armor/predictor_armor.h"
#include "controller_sentry_higher.h"
#include "controller_sentry_higher_debug.h"

/**
 * \warning Controller registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ControllerRegistry<SentryHigherController>
        SentryHigherController::sentry_higher_controller_registry_("sentry_higher");

bool SentryHigherController::Initialize() {
    // Common initialization.
    if (!Controller::Initialize("sentry_higher"))
        return false;

    // Initialize painter.TODO: use trackbar
    controller_sentry_higher_debug_.Initialize(CmdlineArgParser::Instance().DebugShowImage());

    // Initialize Rune module.
    Frame init_frame;
    image_provider_->GetFrame(init_frame);

    LOG(INFO) << "Infantry controller is ready.";
    return true;
}

void SentryHigherController::Run() {
    sleep(2);
    ArmorPredictor armor_predictor{Entity::kBlue, "sentry_higher"};
    while (!exit_signal_) {

        if (!GetImage<true>())
            continue;

        ReceiveSerialData();


            boxes_ = armor_detector_(frame_.image);

            BboxToArmor();
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_);
            DLOG(INFO) << "battlefield pitch" << battlefield_.YawPitchRoll()[0] << ' ' << battlefield_.YawPitchRoll()[1];
            if (CmdlineArgParser::Instance().RunWithSerial()) {
                armor_predictor.SetColor(receive_packet_.color);
                send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size, receive_packet_.bullet_speed);
            } else
                send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size);


            controller_sentry_higher_debug_.DrawAutoAimArmor(frame_.image,
                                                        boxes_,
                                                        &armor_predictor,
                                                        image_provider_->IntrinsicMatrix(),
                                                        frame_.image.size,
                                                        "Infantry Run",
                                                        1);


        if (ControllerSentryHigherDebug::GetKey() == 'q')
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
