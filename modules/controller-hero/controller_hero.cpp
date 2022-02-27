#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image_provider_factory.h"
#include "controller_hero.h"

/**
 * \warning Controller registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ControllerRegistry<HeroController>
        HeroController::hero_controller_registry_("hero");

bool HeroController::Initialize() {
    // Use reset here to allocate memory for an abstract class.
    image_provider_.reset(
            CREATE_IMAGE_PROVIDER(CmdlineArgParser::Instance().RunWithCamera() ? "camera" : "video"));
    if (!image_provider_->Initialize(
            CmdlineArgParser::Instance().RunWithCamera() ?
            "../config/hero/camera-init.yaml" : "../config/hero/video-init.yaml")) {
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

    LOG(INFO) << "Hero controller is ready.";
    return true;
}

void HeroController::Run() {
    while (!exit_signal_) {
        if (!image_provider_->GetFrame(frame_))
            break;
        SerialReceivePacket serial_receive_packet{};

        if (CmdlineArgParser::Instance().RunWithGimbal()) {
            serial_->GetData(serial_receive_packet, std::chrono::milliseconds(20));
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
        cv::imshow("Hero", img);
        if ((cv::waitKey(1) & 0xff) == 'q')
            break;

        BboxToArmor();
        battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.quaternion,
                                   armors_);

        boxes_.clear();
        armors_.clear();
    }

    if (CmdlineArgParser::Instance().RunWithGimbal())
        serial_->StopCommunication();
}
