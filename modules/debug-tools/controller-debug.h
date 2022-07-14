//
// Created by screw on 2022/7/14.
//

#ifndef CONTROLLER_DEBUG_H_
#define CONTROLLER_DEBUG_H_

#include <iostream>
#include "parameter-maintain/parameter-maintain.h"
#include "predictor-armor/predictor_armor.h"

class ControllerDebug
{
public:
    void Initialize(const bool use_painter)
    {
        if (use_painter)
        {
            painter_ = debug::Painter::Instance();
            LOG(INFO) << "Running with debug painter.";
        } else {
            painter_ = debug::NoPainter::Instance();
            LOG(INFO) << "Running without debug painter.";
        }
    }

public:
    debug::IPainter* painter_;

public:

    inline void UpdateImage(const cv::Mat& image)
    {
        painter_->UpdateImage(image);
    }


    inline void drawROI(const cv::Rect_<int>& ROI)
    {
        painter_->DrawBoundingBox(ROI, cv::Scalar(0, 0, 255), 2);
    }

    inline void drawArmors(const std::vector<bbox_t> &bboxes,
                           ArmorPredictor* armor_predictor,
                           const cv::Mat &intrinsic_matrix,
                           const cv::MatSize &image_size)
    {
        for (const auto &box: bboxes) {
            painter_->DrawRotatedRectangle(box.points[0],
                                           box.points[1],
                                           box.points[2],
                                           box.points[3],
                                           cv::Scalar(0, 255, 0), 2);
            painter_->DrawText(std::to_string(box.id), box.points[0], 255, 2);
        }
        painter_->DrawPoint(armor_predictor->ShootPointInPic(intrinsic_matrix, image_size),
                            cv::Scalar(0, 0, 255), 1, 10);
        painter_->DrawPoint(armor_predictor->TargetCenter(), cv::Scalar(100, 255, 100), 2, 2);
    }

    inline void ShowImage(const std::string &window_names, const int &wait_time)
    {
        painter_->ShowImage(window_names, wait_time);
    }

};

#endif //CONTROLLER_DEBUG_H_
