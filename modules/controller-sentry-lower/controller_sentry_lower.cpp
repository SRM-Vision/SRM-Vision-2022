#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <debug-tools/painter.h>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller_sentry_lower.h"
#include "predictor-armor/predictor_armor_renew.h"
#include "compensator/compensator.h"

[[maybe_unused]] ControllerRegistry<SentryLowerController>
        SentryLowerController::sentry_lower_controller_registry_("sentry_lower");

bool SentryLowerController::Initialize() {
    // Use reset here to allocate memory for an abstract class.
    image_provider_.reset(CREATE_IMAGE_PROVIDER(CmdlineArgParser::Instance().RunWithCamera() ? "camera" : "video"));
    if (!image_provider_->Initialize(
            CmdlineArgParser::Instance().RunWithCamera() ?
            "../config/sentry_lower/camera-init.yaml" : "../config/sentry_lower/video-init.yaml")) {
        LOG(ERROR) << "Failed to initialize image provider.";
        // Till now the camera may be open, it's necessary to reset image_provider_ manually to release camera.
        image_provider_.reset();
        return false;
    }

    if (CmdlineArgParser::Instance().DebugUseTrackbar())
        painter_ = debug::Painter::Instance();
    else
        painter_ = debug::NoPainter::Instance();

    if (CmdlineArgParser::Instance().RunWithGimbal()) {
        serial_ = std::make_unique<Serial>();
        if (!serial_->StartCommunication()) {
            LOG(ERROR) << "Failed to start serial communication.";
            serial_->StopCommunication();
            // To use std::make_unique will automatically reset serial_ at the next time.
            // So, there's no need to reset it manually.
            return false;
        }
    }
    if (Compensator::Instance().Initialize("sentry_lower"))
        LOG(INFO) << "Setoff initialize successfully!";
    else
        LOG(ERROR) << "Setoff initialize unsuccessfully!";

    if (coordinate::InitializeMatrix("../config/sentry_lower/matrix-init.yaml"))
        LOG(INFO) << "Camera initialize successfully!";
    else
        LOG(ERROR) << "Camera initialize unsuccessfully!";

    LOG(INFO) << "Lower sentry_lower controller is ready.";
    return true;
}

void SentryLowerController::Run() {
    PredictorArmorRenew armor_predictor(Entity::Colors::kBlue, "sentry_lower");
    sleep(2);

    cv::Rect ROI; // roi of detect armor
    while (!exit_signal_) {
        auto time = std::chrono::steady_clock::now();
        if (!image_provider_->GetFrame(frame_)) {
            auto key = cv::waitKey(50) & 0xff;
            if (key == 'q')
                break;
            continue;
        }

        if (CmdlineArgParser::Instance().RunWithGimbal()) {
            SerialReceivePacket serial_receive_packet{};
            serial_->GetData(serial_receive_packet, std::chrono::milliseconds(5));
            receive_packet_ = ReceivePacket(serial_receive_packet);
        }
        boxes_ = armor_detector_(frame_.image, ROI);
        BboxToArmor();
        battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                   armors_);
        if (CmdlineArgParser::Instance().RunWithSerial()) {
            armor_predictor.SetColor(receive_packet_.color);
            send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size);
        } else
            send_packet_ = SendPacket(armor_predictor.Run(battlefield_, frame_.image.size, AimModes::kAntiTop));
        armor_predictor.GetROI(ROI, frame_.image);
        auto img = frame_.image.clone();
        painter_->UpdateImage(frame_.image);
        for (const auto &box: boxes_) {
            painter_->DrawRotatedRectangle(box.points[0],
                                           box.points[1],
                                           box.points[2],
                                           box.points[3],
                                           cv::Scalar(0, 255, 0), 2);
            painter_->DrawText(std::to_string(box.id), box.points[0], 255, 2);
            painter_->DrawPoint(armors_.front().Center(), cv::Scalar(100, 255, 100), 2, 2);
        }

        painter_->DrawPoint(armor_predictor.ShootPointInPic(image_provider_->IntrinsicMatrix(),
                                                            frame_.image.size),
                            cv::Scalar(0, 0, 255), 1, 10);
        painter_->ShowImage("ARMOR DETECT", 1);

        auto key = cv::waitKey(1) & 0xff;
        if (key == 'q')
            break;
        else if (key == 's')
            ArmorPredictorDebug::Instance().Save();

        Compensator::Instance().SetOff(send_packet_.pitch,send_packet_.yaw, receive_packet_.bullet_speed, send_packet_.check_sum,
                                       armor_predictor.GetTargetDistance());

        if (CmdlineArgParser::Instance().RunWithSerial())
            serial_->SendData(send_packet_, std::chrono::milliseconds(5));

        boxes_.clear();
        armors_.clear();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()
                                                                              - time);
        DLOG(INFO) << "FPS: " << 1000000.0 / duration.count();
    }

    if (CmdlineArgParser::Instance().RunWithGimbal())
        serial_->StopCommunication();

    image_provider_.reset();
}
