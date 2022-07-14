#ifndef CONTROLLER_INFANTRY_DEBUG_H_
#define CONTROLLER_INFANTRY_DEBUG_H_

#include <iostream>
#include "parameter-maintain/parameter-maintain.h"
#include "predictor-outpost/predictor_outpost.h"
#include "debug-tools/controller-debug.h"
#include "controller_hero_debug.h"

class ControllerHeroDebug{
public:
    void Initialize(const bool use_painter)
    {
        controller_debug_.Initialize(use_painter);
    }

private:
    ControllerDebug controller_debug_;

public:
    inline void DrawArmorDetection(const cv::Mat& image,
                                   const cv::Rect_<int>& ROI,
                                   const std::vector<bbox_t> &bboxes,
                                   ArmorPredictor* armor_predictor,
                                   const cv::Mat &intrinsic_matrix,
                                   const cv::MatSize &image_size,
                                   const std::string &window_names,
                                   const int &wait_time)
    {
        controller_debug_.UpdateImage(image);
        controller_debug_.drawROI(ROI);
        controller_debug_.drawArmors(bboxes, armor_predictor, intrinsic_matrix, image_size);
        controller_debug_.ShowImage(window_names, wait_time);
    }

    inline void DrawOutpostData(const cv::Mat& image,
                                   const cv::Rect_<int>& ROI,
                                   OutpostPredictor* outpost_predictor,
                                   const cv::Mat &intrinsic_matrix,
                                   const cv::MatSize &image_size,
                                   const std::string &window_names,
                                   const int &wait_time)
    {
        controller_debug_.UpdateImage(image);
        controller_debug_.drawROI(ROI);
        controller_debug_.ShowPoint(outpost_predictor->OutpostCenter());
        if(outpost_predictor->Fire())
            controller_debug_.ShowText("Fire");
        controller_debug_.ShowImage(window_names, wait_time);
    }


};

#endif //CONTROLLER_INFANTRY_DEBUG_H_
