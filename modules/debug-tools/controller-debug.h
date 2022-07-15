//
// Created by screw on 2022/7/14.
//

#ifndef CONTROLLER_DEBUG_H_
#define CONTROLLER_DEBUG_H_

#include <iostream>
#include "parameter-maintain/parameter-maintain.h"
#include "predictor-armor/predictor_armor.h"

/**
 * \brief This Class is used as the tool class for individual controllerDebug class.
 * \attention You may add new functions for general debug, but functions needs to be inline void.
 */
class ControllerDebug
{
public:
    /**
     * \brief For manually initialize the Debug tools.
     * \param use_painter
     * \note param:debug is replaced by #if !NDEBUG
     */
    void Initialize(const bool use_painter)
    {
#if !NDEBUG
        if (use_painter)
        {
            painter_ = debug::Painter::Instance();
            LOG(INFO) << "Running with debug painter.";
        } else {
            painter_ = debug::NoPainter::Instance();
            LOG(INFO) << "Running without debug painter.";
        }
#else
        painter_ = debug::NoPainter::Instance();
#endif
    }


public:
    /// Painter for drawing
    debug::IPainter* painter_;

public:
    /**
     * \brief Update the image for showing, Remember to update every time you want to draw sth..
     * \param image
     */
    inline void UpdateImage(const cv::Mat& image) const
    {
        painter_->UpdateImage(image);
    }


    /**
     * \brief Draw roi in the frame
     * \param roi
     */
    inline void DrawRoi(const cv::Rect_<int>& roi) const
    {
        painter_->DrawBoundingBox(roi, cv::Scalar(0, 0, 255), 2);
    }

    /**
     * \brief Draw detected armors results from the armor_detector.
     * \param boxes
     * \param armor_predictor
     * \param intrinsic_matrix
     * \param image_size
     */
    inline void DrawArmors(const std::vector<bbox_t> &boxes,
                           ArmorPredictor* armor_predictor,
                           const cv::Mat &intrinsic_matrix,
                           const cv::MatSize &image_size) const
    {
        for (const auto &box: boxes) {
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

    /**
     * \brief Draw text in the frame
     * \param text
     */
    inline void ShowText(const std::string &text) const
    {
        painter_->DrawText(text,{200,200},{0,0,255},2);
    }

    /**
     * \brief Draw point.
     * \param point
     */
    inline void ShowPoint(const cv::Point2f& point) const
    {
        painter_->DrawPoint(point,{0,255,0},5,2);
    }

    /**
     * \brief Show drawn image, Remember to do this as the last step for drawing.
     * \param window_names
     * \param wait_time
     */
    inline void ShowImage(const std::string &window_names, const int &wait_time) const
    {
        painter_->ShowImage(window_names, wait_time);
    }

};

#endif //CONTROLLER_DEBUG_H_
