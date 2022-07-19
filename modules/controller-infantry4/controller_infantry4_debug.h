//
// Created by screw on 2022/7/14.
//

#ifndef CONTROLLER_INFANTRY_4_DEBUG_H_
#define CONTROLLER_INFANTRY_4_DEBUG_H_

#include <iostream>
#include "parameter-maintain/parameter-maintain.h"
#include "predictor-armor/predictor_armor.h"
#include "predictor-rune/predictor_rune_debug.h"
#include "detector-rune/detector_rune_debug.h"
#include "debug-tools/controller-debug.h"

/**
 * \brief Debug tool class for controller infantry.
 */
class ControllerInfantry4Debug{
public:
    /**
     * \brief Manually initialization.
     * \param use_painter
     */
    void Initialize(const bool use_painter)
    {
        controller_debug_.Initialize(use_painter);
    }

public:
    /// general controller debug class
    ControllerDebug controller_debug_;

public:
    /**
     * \brief Draw Auto aim armors for infantry controller's armor detection.
     * \param image
     * \param bboxes
     * \param armor_predictor
     * \param intrinsic_matrix
     * \param image_size
     * \param window_names
     * \param wait_time
     */
    inline void DrawAutoAimArmor(const cv::Mat& image,
                                 const std::vector<bbox_t> &bboxes,
                                 ArmorPredictor* armor_predictor,
                                 const cv::Mat &intrinsic_matrix,
                                 const cv::MatSize &image_size,
                                 const std::string &window_names,
                                 const int &wait_time) const
    {
        controller_debug_.UpdateImage(image);
        controller_debug_.DrawArmors(bboxes, armor_predictor, intrinsic_matrix, image_size);
        controller_debug_.ShowImage(window_names, wait_time);
    }

    /**
     * \brief Draw Auto aim runes for infantry controller's rune detections.
     * \param image Original input image
     * \param rune_predictor
     * \param window_names
     * \param wait_time
     */
    inline void DrawAutoAimRune(const cv::Mat& image,
                                RunePredictor* rune_predictor,
                                const std::string &window_names,
                                const int &wait_time) const
    {
        controller_debug_.UpdateImage(image);
        controller_debug_.painter_->DrawPoint(rune_predictor->PredictedPoint(),
                                              cv::Scalar(0, 255, 255), 3, 3);
        controller_debug_.painter_->DrawText("P", rune_predictor->ArmorCenterP(), cv::Scalar(255, 0, 255), 3);
        controller_debug_.painter_->DrawText("R", rune_predictor->EnergyCenterR(), cv::Scalar(255, 255, 0), 3);
        controller_debug_.ShowImage(window_names, wait_time);
    }

    /**
     * \brief Integrate opencv's waitkey
     * \return 's' for saving, 'q' for quit, 'h' for normal
     */
    static inline char GetKey()
    {
#if !NDEBUG
        char key = char(cv::waitKey(1) & 0xff);
        if (key == 's')
        {
            ArmorPredictorDebug::Instance().Save();
            RunePredictorDebug::Instance().Save();
            RuneDetectorDebug::Instance().Save();
            return 's';
        }

        if (key == 'q')
            return 'q';

#endif
        return 'h';
    }
};

#endif //CONTROLLER_INFANTRY_DEBUG_H_
