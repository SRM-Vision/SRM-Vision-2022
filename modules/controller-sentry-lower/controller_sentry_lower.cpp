#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <debug-tools/painter.h>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller_sentry_lower.h"
#include "compensator/compensator.h"
#include "trajectory-compensator/trajectory-compensator.h"
#include "predictor-armor/predictor_armor.h"

[[maybe_unused]] ControllerRegistry<SentryLowerController>
        SentryLowerController::sentry_lower_controller_registry_("sentry_lower");

bool SentryLowerController::Initialize() {
    // Common initialization.
    if (!Controller::Initialize("sentry_lower"))
        return false;

    // Initialize painter.
    controller_sentry_lower_debug_.Initialize(CmdlineArgParser::Instance().DebugShowImage());

    LOG(INFO) << "Sentry controller is ready.";
    return true;
}

void SentryLowerController::Run() {
    sleep(2);
    ArmorPredictor armor_predictor{Entity::kBlue, "sentry_lower"};

    auto pitch_solver = compensator::CompensatorTraj();
    pitch_solver.Initialize("sentry_lower");

    while (!exit_signal_) {

        // 2022.7.17, Tran Tuan: Reversed the real camera.
        if (!GetImage<false>())
            continue;

        ReceiveSerialData();

        boxes_ = armor_detector_(frame_.image);
        BboxToArmor();

        battlefield_ = Battlefield(frame_.time_stamp,
                                   receive_packet_.bullet_speed,
                                   receive_packet_.yaw_pitch_roll,
                                   armors_);

        DLOG(INFO) << "cY: " << battlefield_.YawPitchRoll()[0] << ", cP: " << battlefield_.YawPitchRoll()[1];

        send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size, receive_packet_.color);

        double delta_pitch = 0;

        if (!armors_.empty()) {
            double current_pitch = battlefield_.YawPitchRoll()[1];
            auto pitch_solution = pitch_solver.AnyTargetOffset(30, armors_[0]);
            delta_pitch = pitch_solution.x() - current_pitch;

            DLOG(INFO) << "cY: " << battlefield_.YawPitchRoll()[0]
                       << ", cP: " << battlefield_.YawPitchRoll()[1]
                       << ", dP: " << delta_pitch;
        }

        delta_pitch = pitch_solver.GroundTargetOffset(30, armors_[0]);

        send_packet_.pitch -= static_cast<float>(delta_pitch);

        controller_sentry_lower_debug_.DrawAutoAimArmor(
                frame_.image,
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
