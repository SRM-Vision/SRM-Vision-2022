#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller-base/controller_factory.h"
#include "predictor-armor/predictor_armor_renew.h"
#include "detector-rune/detector_rune.h"
#include "predictor-rune/predictor-rune.h"
#include "controller_infantry.h"
#include "compensator/compensator.h"
#include "digital-twin/entity.h"

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
    rune_predictor_.Initialize("../config/infantry/rune-predictor-param.yaml", true);
    if ( CmdlineArgParser::Instance().DebugUseTrackbar() )// TODO Debug
    {
        painter_ = debug::Painter::Instance();
        LOG(INFO) << "Rune predictor initialize successfully!";
    }
    else
    {
        painter_ = debug::NoPainter::Instance();
        LOG(ERROR) << "Rune predictor initialize unsuccessfully!";
    }


    if(Compensator::Instance().Initialize("infantry"))
        LOG(INFO) << "Set off initialize successfully!";
    else
        LOG(ERROR) << "Set off initialize unsuccessfully!";

    if (coordinate::InitializeMatrix("../config/infantry/matrix-init.yaml"))
        LOG(INFO) << "Camera initialize successfully!";
    else
        LOG(ERROR) << "Camera initialize unsuccessfully!";

    LOG(INFO) << "Infantry controller is ready.";
    return true;
}

void InfantryController::Run() {
    sleep(2);

    cv::Rect ROI; // roi of detect armor
    while (!exit_signal_) {
        auto time = std::chrono::steady_clock::now();
        if (!image_provider_->GetFrame(frame_)){
            sleep(1);
            continue;
        }

        // When used camera, need to flip image
        if(CmdlineArgParser::Instance().RunWithCamera()){
            cv::flip(frame_.image, frame_.image, 0);
            cv::flip(frame_.image, frame_.image, 1);
        }

        painter_->UpdateImage(frame_.image);

        if (CmdlineArgParser::Instance().RunWithGimbal()) {
            SerialReceivePacket serial_receive_packet{};
            serial_->GetData(serial_receive_packet, std::chrono::milliseconds(5));
            receive_packet_ = ReceivePacket(serial_receive_packet);
        }

        if (CmdlineArgParser::Instance().RuneModeRune()) {
            power_rune_ = rune_detector_.Run(receive_packet_.color, frame_);
            send_packet_ = SendPacket(rune_predictor_.Run(power_rune_, receive_packet_.mode, receive_packet_.bullet_speed));
            painter_->DrawPoint(rune_predictor_.PredictedPoint(),
                                                 cv::Scalar(0, 255, 255), 3, 3);
            painter_->ShowImage("Rune", 1);
        } else {
            boxes_ = armor_detector_(frame_.image,ROI);
            for(auto &box:boxes_){
                DLOG(INFO) << "CONFIDENCE: " << box.confidence << " color : " << box.color;
            }
            BboxToArmor();
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_);
            /// TODO mode switch
            if (CmdlineArgParser::Instance().RunWithSerial()) {
                armor_predictor_.SetColor(Entity::kBlue);
                send_packet_ = armor_predictor_.Run(battlefield_, frame_.image.size ,
                                                   receive_packet_.mode, receive_packet_.bullet_speed);
            } else
                send_packet_ = armor_predictor_.Run(battlefield_, frame_.image.size,AimModes::kAntiTop);
            armor_predictor_.GetROI(ROI,frame_.image);
            painter_->UpdateImage(frame_.image);
            painter_->DrawBoundingBox(ROI,cv::Scalar(0,0,255),2);
            for (const auto &box: boxes_) {
                painter_->DrawRotatedRectangle(box.points[0],
                                                                box.points[1],
                                                                box.points[2],
                                                                box.points[3],
                                                                cv::Scalar(0, 255, 0), 2);
                painter_->DrawText(std::to_string(box.id), box.points[0], 255, 2);
                painter_->DrawPoint(armors_.front().Center(), cv::Scalar(100, 255, 100), 2, 2);
            }
            painter_->DrawPoint(armor_predictor_.ShootPointInPic(image_provider_->IntrinsicMatrix(),
                                                                                 frame_.image.size),
                                                 cv::Scalar(0, 0, 255), 1, 10);
            armor_predictor_.AllShootPoint(image_provider_->IntrinsicMatrix());
            painter_->ShowImage("ARMOR DETECT", 1);
        }

        auto key = cv::waitKey(1) & 0xff;
        if (key == 'q')
            break;
        else if (key == 's')
            ArmorPredictorDebug::Instance().Save();
        Compensator::Instance().Setoff(send_packet_.pitch,
                                       receive_packet_.bullet_speed,
                                       armor_predictor_.GetTargetDistance(),
                                       receive_packet_.mode);
        if (CmdlineArgParser::Instance().RunWithSerial()) {
            serial_->SendData(send_packet_, std::chrono::milliseconds(5));
        }
        boxes_.clear();
        armors_.clear();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()
                                                                              - time);
        DLOG(INFO) << "all computation cost ms: " << double(duration.count())/1e6;
        DLOG(INFO) << " FPS: " << 1e9 / double(duration.count());
    }

    // exit.
    if (CmdlineArgParser::Instance().RunWithGimbal())
        serial_->StopCommunication();

    image_provider_.reset();
}

