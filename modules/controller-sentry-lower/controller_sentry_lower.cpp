#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <debug-tools/painter.h>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller_sentry_lower.h"
#include "compensator/compensator.h"
#include "predictor-armor/predictor_armor.h"

[[maybe_unused]] ControllerRegistry<SentryLowerController>
        SentryLowerController::sentry_lower_controller_registry_("sentry_lower");

bool SentryLowerController::Initialize() {
    // Common initialization.
    if (!Controller::Initialize("sentry_lower"))
        return false;

    // Initialize painter.TODO: use trackbar
    controller_sentry_lower_debug_.Initialize(CmdlineArgParser::Instance().DebugShowImage());

    // Initialize Rune module.
    Frame init_frame;
    image_provider_->GetFrame(init_frame);

    LOG(INFO) << "Sentry controller is ready.";
    return true;
}

void SentryLowerController::Run() {
    sleep(2);
    ArmorPredictor armor_predictor{Entity::kBlue, "sentry_lower"};
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


        controller_sentry_lower_debug_.DrawAutoAimArmor(frame_.image,
                                                        boxes_,
                                                        &armor_predictor,
                                                        image_provider_->IntrinsicMatrix(),
                                                        frame_.image.size,
                                                        "Sentry Run",
                                                        1);


        if (ControllerSentryLowerDebug::GetKey() == 'q')
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
