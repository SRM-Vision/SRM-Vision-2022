#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image_provider_factory.h"
#include "controller-base/controller_factory.h"
#include "predictor-armor/predictor_armor.h"
#include "detector-rune/detector_rune.h"
#include "predictor-rune/predictor_rune.h"
#include "controller_infantry.h"
#include "chrono"

/**
 * \warning Controller registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ControllerRegistry<InfantryController>
        InfantryController::infantry_controller_registry_("infantry");

bool InfantryController::Initialize() {
    // Use reset here to allocate memory for an abstract class.
    image_provider_.reset(CREATE_IMAGE_PROVIDER(CmdlineArgParser::Instance().RunWithCamera() ? "camera" : "video"));
    if (!image_provider_->Initialize(
            CmdlineArgParser::Instance().RunWithCamera() ?
            "../config/infantry/camera-init.yaml" : "../config/infantry/video-init.yaml")) {
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

    // rune initialize program.
    Frame init_frame;
    image_provider_->GetFrame(init_frame);
    if (rune_detector_.Initialize("../config/infantry/rune-detector-param.yaml", init_frame,
                                  CmdlineArgParser::Instance().DebugUseTrackbar()))
        LOG(INFO) << "Rune detector initialize successfully!";
    else
        LOG(ERROR) << "Rune detector initialize unsuccessfully!";
    if (rune_predictor_.Initialize("../config/infantry/rune-predictor-param.yaml"),
                                  CmdlineArgParser::Instance().DebugUseTrackbar())
        LOG(INFO) << "Rune predictor initialize successfully!";
    else
        LOG(ERROR) << "Rune predictor initialize unsuccessfully!";

    if(coordinate::InitializeMatrix("../config/infantry/matrix-init.yaml"))
        LOG(INFO) << "Camera initialize successfully!";
    else
        LOG(ERROR) << "Camera initialize unsuccessfully!";

    LOG(INFO) << "Infantry controller is ready.";
    return true;
}

void InfantryController::Run() {
    ArmorPredictor armor_predictor(Entity::Colors::kBlue, true,"infantry");

    sleep(2);

    while (!exit_signal_) {
        auto time = std::chrono::steady_clock::now();
        if (!image_provider_->GetFrame(frame_))
            break;
        cv::flip(frame_.image, frame_.image, 0);
        cv::flip(frame_.image, frame_.image, 1);
        debug::Painter::Instance().UpdateImage(frame_.image);

        if (CmdlineArgParser::Instance().RunWithGimbal()) {
            SerialReceivePacket serial_receive_packet{};
            serial_->GetData(serial_receive_packet, std::chrono::milliseconds(5));
            receive_packet_ = ReceivePacket(serial_receive_packet);
        }

        if (CmdlineArgParser::Instance().RuneModeRune()) {
            power_rune_ = rune_detector_.Run(frame_);
            send_packet_ = SendPacket(rune_predictor_.Run(power_rune_, kBigRune));
            debug::Painter::Instance().DrawPoint(rune_predictor_.FinalTargetPoint(),
                                                 cv::Scalar(0, 255, 0), 3, 3);
            debug::Painter::Instance().ShowImage("Rune");
        } else {
            boxes_ = armor_detector_(frame_.image);
            BboxToArmor();
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_);
            /// TODO mode switch
            if(CmdlineArgParser::Instance().RunWithSerial()) {
                armor_predictor.color_ = receive_packet_.color;
                send_packet_ = SendPacket(armor_predictor.Run(battlefield_, receive_packet_.mode, receive_packet_.bullet_speed));
            }else
                send_packet_ = SendPacket(armor_predictor.Run(battlefield_, AimModes::kAntiTop));
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
            auto point = camera_matrix * armor_predictor.TranslationVectorCamPredict()
                    / armor_predictor.TranslationVectorCamPredict()(2, 0);
            cv::Point2d point_cv = {point[0], point[1]};
            debug::Painter::Instance().DrawPoint(point_cv, cv::Scalar(0, 0, 255), 1, 10);
            debug::Painter::Instance().ShowImage("ARMOR DETECT");
        }

        auto key = cv::waitKey(1) & 0xff;
        if (key == 'q')
            break;
        else if(key == 's')
            ArmorPredictorDebug::Instance().Save();

        if(CmdlineArgParser::Instance().RunWithSerial()) {
            serial_->SendData(send_packet_, std::chrono::milliseconds(5));
        }
        boxes_.clear();
        armors_.clear();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
                - time);
        DLOG(INFO) << "FPS: " << 1000.0/duration.count();
    }


    // exit.
    if (CmdlineArgParser::Instance().RunWithGimbal())
        serial_->StopCommunication();

    image_provider_.reset();
}

