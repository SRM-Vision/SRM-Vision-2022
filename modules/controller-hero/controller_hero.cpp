#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <debug-tools/painter.h>
#include <predictor-armor/predictor_armor.h>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image_provider_factory.h"
#include "controller_hero.h"
#include "predictor-outpost/predictor-outpost.h"

/**
 * \warning Controller registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ControllerRegistry<HeroController>
        HeroController::hero_controller_registry_("hero");

bool HeroController::Initialize() {
    // Use reset here to allocate memory for an abstract class.
    image_provider_.reset(CREATE_IMAGE_PROVIDER(CmdlineArgParser::Instance().RunWithCamera() ? "camera" : "video"));
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

    if(coordinate::InitializeMatrix("../config/infantry/matrix-init.yaml"))
        LOG(INFO) << "Camera initialize successfully!";
    else
        LOG(ERROR) << "Camera initialize unsuccessfully!";

    LOG(INFO) << "Hero controller is ready.";
    return true;
}

void HeroController::Run() {
    ArmorPredictor armor_predictor(Entity::Colors::kBlue, true,"hero");
    sleep(2);

    while (!exit_signal_) {
        if (!image_provider_->GetFrame(frame_))
            break;
        debug::Painter::Instance().UpdateImage(frame_.image);
        if (CmdlineArgParser::Instance().RunWithGimbal()) {
            SerialReceivePacket serial_receive_packet{};
            serial_->GetData(serial_receive_packet, std::chrono::milliseconds(5));
            receive_packet_ = ReceivePacket(serial_receive_packet);

            DLOG(WARNING) << "mode: " << receive_packet_.mode << " bullet speed: " << receive_packet_.bullet_speed
                       << " armor kind: " << receive_packet_.armor_kind << " color: " << receive_packet_.color
                       << " prior_enemy: " << receive_packet_.prior_enemy;
            DLOG(WARNING) << "yaw, pitch, roll: "
                       << receive_packet_.yaw_pitch_roll[0] << " | " << receive_packet_.yaw_pitch_roll[1] << " | " << receive_packet_.yaw_pitch_roll[2];
        }

        // Choose mode
        if (receive_packet_.mode == 50)          // 50 means Outpost in Receive_packet.AimMode
        {
            DLOG(INFO) << receive_packet_.mode;
            // TODO
        } else {  //TODO 20 means normal auto_aim
            boxes_ = armor_detector_(frame_.image);
            BboxToArmor();
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_);

            // If run with serial
            if(CmdlineArgParser::Instance().RunWithSerial()) {
                armor_predictor.color_ = receive_packet_.color; // color == 23 -> blue; color == 33 -> red
                send_packet_ = SendPacket(armor_predictor.Run(battlefield_, receive_packet_.mode,receive_packet_.bullet_speed));
            } else
                send_packet_ = SendPacket(armor_predictor.Run(battlefield_, AimModes::kNormal));

            // Draw points
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

            // Coordinate change
            Eigen::Matrix3d camera_matrix;
            cv::cv2eigen(image_provider_->IntrinsicMatrix(), camera_matrix);
            auto point = camera_matrix * armor_predictor.TranslationVectorCamPredict() / armor_predictor.TranslationVectorCamPredict()(2, 0);
            cv::Point2d point_cv = {point[0], point[1]};
            debug::Painter::Instance().DrawPoint(point_cv, cv::Scalar(0, 0, 255), 1, 10);
            debug::Painter::Instance().ShowImage("ARMOR DETECT");
        }
        auto key = cv::waitKey(1) & 0xff;

        if (key == 'q')
            break;
        else if(key == 's')
            ArmorPredictorDebug::Instance().Save();

        if(CmdlineArgParser::Instance().RunWithSerial())
        {
            float coefficient1, coefficient2; // TODO today
            send_packet_.pitch = coefficient1 * receive_packet_.bullet_speed + coefficient2 * static_cast<float>(send_packet_.distance_mode);
            serial_->SendData(send_packet_, std::chrono::milliseconds(5));
        }

        boxes_.clear();
        armors_.clear();
    }

    if (CmdlineArgParser::Instance().RunWithGimbal())
        serial_->StopCommunication();

    image_provider_.reset();
}
