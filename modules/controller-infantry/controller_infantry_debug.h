//
// Created by screw on 2022/7/14.
//

#ifndef CONTROLLER_INFANTRY_DEBUG_H_
#define CONTROLLER_INFANTRY_DEBUG_H_

#include <iostream>
#include "parameter-maintain/parameter-maintain.h"
#include "predictor-armor/predictor_armor.h"
#include "predictor-rune/predictor_rune_debug.h"
#include "detector-rune/detector_rune_debug.h"
#include "debug-tools/controller-debug.h"

class ControllerInfantryDebug{
public:
    void Initialize(const bool use_painter)
    {
        controller_debug_.Initialize(use_painter);
    }

public:
    ControllerDebug controller_debug_;

public:
    inline void DrawAutoAimArmor(const cv::Mat& image,
                                 const std::vector<bbox_t> &bboxes,
                                 ArmorPredictor* armor_predictor,
                                 const cv::Mat &intrinsic_matrix,
                                 const cv::MatSize &image_size,
                                 const std::string &window_names,
                                 const int &wait_time)
    {
        controller_debug_.UpdateImage(image);
        controller_debug_.drawArmors(bboxes, armor_predictor, intrinsic_matrix, image_size);
        controller_debug_.ShowImage(window_names, wait_time);
    }

    inline void DrawAutoAimRune(const cv::Mat& image,
                                RunePredictor* rune_predictor,
                                const std::string &window_names,
                                const int &wait_time)
    {
        controller_debug_.UpdateImage(image);
        controller_debug_.painter_->DrawPoint(rune_predictor->PredictedPoint(),
                                              cv::Scalar(0, 255, 255), 3, 3);
        controller_debug_.ShowImage(window_names, wait_time);
    }

    inline char GetKey()
    {
#if !NDEBUG
        char key = cv::waitKey(1) & 0xff;
        if (key == 's')
        {
            ArmorPredictorDebug::Instance().Save();
            RunePredictorDebug::Instance().Save();
            RuneDetectorDebug::Instance().Save();
        }

        if (key == 'q')
            return 'q';

#endif
        return 'h';
    }
};

#endif //CONTROLLER_INFANTRY_DEBUG_H_
