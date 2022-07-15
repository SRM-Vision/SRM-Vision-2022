#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <debug-tools/painter.h>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller_sentry_higher.h"
#include "compensator/compensator.h"
#include "predictor-armor/predictor_armor.h"

[[maybe_unused]] ControllerRegistry<SentryHigherController>
        SentryHigherController::sentry_higher_controller_registry_("sentry_higher");

bool SentryHigherController::Initialize() {
    // Common initialization.
    if (!Controller::Initialize("sentry_higher"))
        return false;

    // Initialize the painter.
    if (CmdlineArgParser::Instance().DebugShowImage())
        painter_ = debug::NoPainter::Instance();
    else
        painter_ = debug::NoPainter::Instance();

    LOG(INFO) << "Higher sentry_higher controller is ready.";
    return true;
}

void SentryHigherController::Run() {
    ArmorPredictor armor_predictor(Entity::kBlue, "sentry_higher");
    sleep(2);

    while (!exit_signal_) {
        auto time = std::chrono::steady_clock::now();

        if (!GetImage<false>())
            continue;

        RunGimbal();

        boxes_ = armor_detector_(frame_.image);
        BboxToArmor();
        battlefield_ = Battlefield(frame_.time_stamp, receive_packet_.bullet_speed, receive_packet_.yaw_pitch_roll,
                                   armors_);

        if (CmdlineArgParser::Instance().RunWithSerial()) {
            armor_predictor.SetColor(receive_packet_.color);
            send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size);
        } else
            send_packet_ = armor_predictor.Run(battlefield_, frame_.image.size);
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

        Compensator::Instance().Offset(send_packet_.pitch, send_packet_.yaw, receive_packet_.bullet_speed,
                                       send_packet_.check_sum,
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