#ifndef CONTROLLER_SENTRY_LOWER_DEBUG_H_
#define CONTROLLER_SENTRY_LOWER_DEBUG_H_

#include <iostream>
#include "parameter-maintain/parameter-maintain.h"
#include "predictor-armor/predictor_armor.h"
#include "debug-tools/controller-debug.h"

/// \brief Debug tool class for controller infantry.
class ControllerSentryLowerDebug {
public:
    /**
     * \brief Manually initialization.
     * \param use_painter
     */
    void Initialize(const bool use_painter) {
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
    inline void DrawAutoAimArmor(const cv::Mat &image,
                                 const std::vector<bbox_t> &bboxes,
                                 ArmorPredictor *armor_predictor,
                                 const cv::Mat &intrinsic_matrix,
                                 const cv::MatSize &image_size,
                                 const std::string &window_names,
                                 const int &wait_time) const {
        controller_debug_.UpdateImage(image);
        controller_debug_.DrawArmors(bboxes, armor_predictor, intrinsic_matrix, image_size);
        controller_debug_.ShowImage(window_names, wait_time);
    }

    /**
     * \brief Integrate opencv's waitkey
     * \return 's' for saving, 'q' for quit, 'n' for normal.
     */
    static inline char GetKey() {
#if !NDEBUG
        char key = char(cv::waitKey(1) & 0xff);
        if (key == 's') {
            ArmorPredictorDebug::Instance().Save();
            return 's';
        }

        if (key == 'q')
            return 'q';
#endif
        return 'n';
    }
};

#endif  // CONTROLLER_SENTRY_LOWER_DEBUG_H_
