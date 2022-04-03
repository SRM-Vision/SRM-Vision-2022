#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <debug-tools/painter.h>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image_provider_factory.h"
#include "controller_sentry_lower.h"
#include "predictor-armor/predictor_armor.h"

[[maybe_unused]] ControllerRegistry<SentryLowerController>
        SentryLowerController::sentry_lower_controller_registry_("sentry_lower");

bool SentryLowerController::Initialize() {
    // Use reset here to allocate memory for an abstract class.
    image_provider_.reset(CREATE_IMAGE_PROVIDER(CmdlineArgParser::Instance().RunWithCamera() ? "camera" : "video"));
    if (!image_provider_->Initialize(
            CmdlineArgParser::Instance().RunWithCamera() ?
            "../config/sentry/camera-init.yaml" : "../config/sentry/video-init.yaml")) {
        LOG(ERROR) << "Failed to initialize image provider.";
        // Till now the camera may be open, it's necessary to reset image_provider_ manually to release camera.
        image_provider_.reset();
        return false;
    }

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

    if(coordinate::InitializeMatrix("../config/infantry/matrix-init.yaml"))
        LOG(INFO) << "Camera initialize successfully!";
    else
        LOG(ERROR) << "Camera initialize unsuccessfully!";

    LOG(INFO) << "Lower sentry controller is ready.";
    return true;
}

void SentryLowerController::Run() {
    ArmorPredictor armor_predictor(Entity::Colors::kBlue, true,"sentry");
    sleep(2);

    while (!exit_signal_) {
        if (!image_provider_->GetFrame(frame_))
            break;

        if (CmdlineArgParser::Instance().RunWithGimbal()) {
            SerialReceivePacket serial_receive_packet{};
            serial_->GetData(serial_receive_packet, std::chrono::milliseconds(5));
            receive_packet_ = ReceivePacket(serial_receive_packet);
        }
        boxes_ = armor_detector_(frame_.image);
        BboxToArmor();
        battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                   armors_);

        if(CmdlineArgParser::Instance().RunWithSerial()) {
            armor_predictor.color_ = receive_packet_.color;
            send_packet_ = SendPacket(armor_predictor.Run(battlefield_, AimModes::kAutoAntiTop,receive_packet_.bullet_speed));
        }else
            send_packet_ = SendPacket(armor_predictor.Run(battlefield_, AimModes::kNormal));
        auto img = frame_.image.clone();
        debug::Painter::Instance().UpdateImage(frame_.image);
        for (const auto &box: boxes_) {
            debug::Painter::Instance().DrawRotatedRectangle(box.points[0],
                                                            box.points[1],
                                                            box.points[2],
                                                            box.points[3],
                                                            cv::Scalar(0, 255, 0), 2);
            debug::Painter::Instance().DrawText(std::to_string(box.id), box.points[0], 255, 2);
            debug::Painter::Instance().DrawPoint(armors_.front().Center(), cv::Scalar(100, 255, 100));
        }

        Eigen::Matrix3d camera_matrix;
        cv::cv2eigen(image_provider_->IntrinsicMatrix(), camera_matrix);
        auto point = camera_matrix * armor_predictor.TranslationVectorCamPredict() / armor_predictor.TranslationVectorCamPredict()(2, 0);
        cv::Point2d point_cv = {point[0], point[1]};
        debug::Painter::Instance().DrawPoint(point_cv, cv::Scalar(0, 0, 255), 1, 10);
        debug::Painter::Instance().ShowImage("ARMOR DETECT");

        auto key = cv::waitKey(1) & 0xff;
        if (key == 'q')
            break;
        else if (key == 's')
            ArmorPredictorDebug::Instance().Save();
        if(CmdlineArgParser::Instance().RunWithSerial())
            serial_->SendData(send_packet_, std::chrono::milliseconds(5));

        boxes_.clear();
        armors_.clear();
    }

    if (CmdlineArgParser::Instance().RunWithGimbal())
        serial_->StopCommunication();

    image_provider_.reset();
}
