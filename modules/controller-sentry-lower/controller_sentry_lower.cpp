#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
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
    LOG(INFO) << "Lower sentry controller is ready.";
    return true;
}

void SentryLowerController::Run() {
    ArmorPredictor armor_predictor(Entity::Colors::kBlue, true);

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

        auto img = frame_.image.clone();
        for (const auto &box: boxes_) {
            line(img, box.points[0], box.points[1], cv::Scalar(0, 255, 0), 2);
            line(img, box.points[1], box.points[2], cv::Scalar(0, 255, 0), 2);
            line(img, box.points[2], box.points[3], cv::Scalar(0, 255, 0), 2);
            line(img, box.points[3], box.points[0], cv::Scalar(0, 255, 0), 2);
            cv::putText(img, std::to_string(box.id), box.points[0],
                        cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar((box.color == 0) * 255, 0, (box.color == 1) * 255), 2);
        }

        BboxToArmor();
        battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.quaternion,
                                   armors_);

        send_packet_ = SerialSendPacket(armor_predictor.Run(battlefield_, ArmorPredictor::Modes::kAntiTop));
        Eigen::Matrix3d camera_matrix;
        cv::cv2eigen(image_provider_->IntrinsicMatrix(), camera_matrix);
        DrawPredictedPoint(img, camera_matrix, armor_predictor.TranslationVectorCamPredict());
        cv::imshow("Sentry Lower", img);

        if ((cv::waitKey(1) & 0xff) == 'q')
            break;
        SerialSendPacket send_packet{1.f, 2.f, 3.f, false,4.f};
        //serial_->SendData(send_packet, std::chrono::milliseconds(5));

        boxes_.clear();
        armors_.clear();
    }

    if (CmdlineArgParser::Instance().RunWithGimbal())
        serial_->StopCommunication();
}
