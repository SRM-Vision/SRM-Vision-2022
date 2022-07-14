#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller-base/controller_factory.h"
#include "detector-rune/detector_rune.h"
#include "predictor-rune/predictor-rune.h"
#include "compensator/compensator.h"
#include "predictor-armor/predictor_armor.h"
#include "controller_infantry.h"
#include "controller_infantry_debug.h"

/**
 * \warning Controller registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ControllerRegistry<InfantryController>
        InfantryController::infantry_controller_registry_("infantry");

bool InfantryController::Initialize() {
    if (!InitializeImageProvider("infantry") || !InitializeGimbalSerial())
        return false;

    // Initialize Rune module.
    Frame init_frame;
    image_provider_->GetFrame(init_frame);
    if (rune_detector_.Initialize("../config/infantry/rune-detector-param.yaml"))
        LOG(INFO) << "Rune detector initialize successfully!";
    else {
        LOG(ERROR) << "Rune detector initialize failed.";
        return false;
    }
    rune_predictor_.Initialize("../config/infantry/rune-predictor-param.yaml", true);
    if (CmdlineArgParser::Instance().DebugUseTrackbar())  // TODO Debug
    {
        painter_ = debug::Painter::Instance();
        LOG(INFO) << "Running with debug painter.";
    } else {
        painter_ = debug::NoPainter::Instance();
        LOG(INFO) << "Running without debug painter.";
    }

    if (Compensator::Instance().Initialize("infantry"))
        LOG(INFO) << "Offset initialized.";
    else {
        LOG(ERROR) << "Offset initialize failed.";
        return false;
    }

    if (coordinate::InitializeMatrix("../config/infantry/matrix-init.yaml"))
        LOG(INFO) << "Camera initialized.";
    else {
        LOG(ERROR) << "Camera coordinate matrix initialize failed.";
        return false;
    }

    LOG(INFO) << "Infantry controller is ready.";
    return true;
}

void InfantryController::Run() {
    sleep(2);
    cv::Rect ROI;  // Armor ROI rect.
    ArmorPredictor armor_predictor{Entity::kBlue, "infantry"};
    while (!exit_signal_) {
        auto time = std::chrono::steady_clock::now();

        if (!GetImage<true>())
            continue;

        RunGimbal();

        painter_->UpdateImage(frame_.image);
        if (CmdlineArgParser::Instance().RuneModeRune()) {
            power_rune_ = rune_detector_.Run(receive_packet_.color, frame_);
            send_packet_ = SendPacket(rune_predictor_.Run(power_rune_, receive_packet_.mode, receive_packet_.bullet_speed));
            painter_->DrawPoint(rune_predictor_.PredictedPoint(),
                                cv::Scalar(0, 255, 255), 3, 3);
            painter_->ShowImage("Rune", 1);
        } else {
            boxes_ = armor_detector_(frame_.image, ROI);

            BboxToArmor();
            battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                       armors_);
            /// TODO mode switch
            if (CmdlineArgParser::Instance().RunWithSerial()) {
                armor_predictor.SetColor(receive_packet_.color);
                send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size, receive_packet_.bullet_speed);
            } else
                send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size);
            armor_predictor.GetROI(ROI, frame_.image);
            painter_->UpdateImage(frame_.image);
            painter_->DrawBoundingBox(ROI, cv::Scalar(0, 0, 255), 2);
            for (const auto &box: boxes_) {
                painter_->DrawRotatedRectangle(box.points[0],
                                               box.points[1],
                                               box.points[2],
                                               box.points[3],
                                               cv::Scalar(0, 255, 0), 2);
                painter_->DrawText(std::to_string(box.id), box.points[0], 255, 2);
            }
            painter_->DrawPoint(armor_predictor.ShootPointInPic(image_provider_->IntrinsicMatrix(),frame_.image.size),
                                cv::Scalar(0, 0, 255), 1, 10);
            painter_->DrawPoint(armor_predictor.TargetCenter(), cv::Scalar(100, 255, 100), 2, 2);
            painter_->ShowImage("ARMOR DETECT", 1);
        }

        auto key = cv::waitKey(1) & 0xff;
        if (key == 'q')
            break;
        else if (key == 's')
            ArmorPredictorDebug::Instance().Save();

        if (CmdlineArgParser::Instance().RunWithSerial()) {
            serial_->SendData(send_packet_, std::chrono::milliseconds(5));
        }
        boxes_.clear();
        armors_.clear();
        auto duration =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - time);
        LOG(INFO) << "Cost: " << double(duration.count()) / 1e6 << ", FPS: " << 1e9 / double(duration.count()) << ".";
    }

    // Exit.
    if (CmdlineArgParser::Instance().RunWithGimbal() || CmdlineArgParser::Instance().RunWithSerial())
        serial_->StopCommunication();

    image_provider_.reset();
}
