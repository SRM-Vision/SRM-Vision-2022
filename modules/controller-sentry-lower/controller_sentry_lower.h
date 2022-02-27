/**
 * Infantry controller header.
 * \author trantuan-20048607, screw-44
 * \date 2022.2.8
 * \warning NEVER include this file except in ./controller_sentry_lower.cpp.
 */

#ifndef CONTROLLER_SENTRY_LOWER_H_
#define CONTROLLER_SENTRY_LOWER_H_

#include "controller-base/controller_factory.h"

class [[maybe_unused]] SentryLowerController final : public Controller {
public:
    bool Initialize() final;

    void Run() final;

private:
    /**
     * \brief Draw the output point from predictor.
     * \param [in,out] image Source image.
     * \param [in] camera_matrix Camera internal matrix.
     * \param [in] camera_point Point in camera coordinate.
     */
    inline static void DrawPredictedPoint(
            cv::Mat &image,
            const Eigen::Matrix3d &camera_matrix,
            const Eigen::Vector3d &camera_point) {
        auto point = camera_matrix * camera_point / camera_point(2, 0);
        cv::Point2d point_cv = {point[0], point[1]};
        DLOG(INFO) << "Predictor output: " << point_cv;
        cv::circle(image, point_cv, 1, cv::Scalar(0, 0, 255), 10);
    }

    /// Own registry in controller factory.
    [[maybe_unused]] static ControllerRegistry<SentryLowerController> sentry_lower_controller_registry_;
};

#endif  // CONTROLLER_SENTRY_LOWER_H_
