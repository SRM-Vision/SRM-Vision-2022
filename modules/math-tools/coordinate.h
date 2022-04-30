/**
 * Coordinate transformer header.
 * \author trantuan-20048607
 * \date 2022.1.30
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef COORDINATE_H_
#define COORDINATE_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <opencv2/core/eigen.hpp>
#include <ceres/jet.h>
#include "algorithms.h"
#include <glog/logging.h>

namespace coordinate {
    typedef Eigen::Vector3d TranslationVector;
    typedef Eigen::Vector3d RotationVector;

    typedef Eigen::Matrix<double, 3, 1> TranslationMatrix;
    typedef Eigen::Matrix3d RotationMatrix;

    // IMU and Camera joint calibration.
    static double rm_cam_to_imu_data[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    static double tm_cam_to_imu_data[] = {0, 0, 0};
    static RotationMatrix camera_to_imu_rotation_matrix(rm_cam_to_imu_data);
    static TranslationMatrix camera_to_imu_translation_matrix(tm_cam_to_imu_data);

    bool InitializeMatrix(const std::string &path);
}

namespace coordinate::transform {
    [[maybe_unused]] inline RotationMatrix
    EulerAngleToRotationMatrix(const std::array<float, 3> e_yaw_pitch_roll) {
        // Prefix "e_" here means "Euler angle".

        // Prefix "r_" here means "rotation".
        RotationMatrix r_yaw, r_roll, r_pitch;

        // [Experimental] Use SSE2 or NEON to accelerate sine and cosine.
#if defined(__x86_64__) | defined(__aarch64__)
        float r[4] = {e_yaw_pitch_roll[0], e_yaw_pitch_roll[1], e_yaw_pitch_roll[2], float(0)}, sin_r[4], cos_r[4];
        algorithm::SinCosFloatX4(r, sin_r, cos_r);
        r_yaw << cos_r[0], 0, sin_r[0],
                0, 1, 0,
                -sin_r[0], 0, cos_r[0]; // Y
        r_roll << cos_r[1], -sin_r[1], 0,
                sin_r[1], cos_r[1], 0,
                0, 0, 1;    // Z
        r_pitch << 1, 0, 0,
                0, cos_r[2], -sin_r[2],
                0, sin_r[2], cos_r[2];  // X
#else
        r_yaw << cos(e_yaw_pitch_roll[0]), 0, sin(e_yaw_pitch_roll[0]),
                0, 1, 0,
                -sin(e_yaw_pitch_roll[0]), 0, cos(e_yaw_pitch_roll[0]);
        r_roll << cos(e_yaw_pitch_roll[2]), -sin(e_yaw_pitch_roll[2]), 0,
                sin(e_yaw_pitch_roll[2]), cos(e_yaw_pitch_roll[2]), 0,
                0, 0, 1;
        r_pitch << 1, 0, 0,
                0, cos(e_yaw_pitch_roll[1]), -sin(e_yaw_pitch_roll[1]),
                0, sin(e_yaw_pitch_roll[1]), cos(e_yaw_pitch_roll[1]);
#endif

        return r_yaw * r_pitch * r_roll;
    }

    [[maybe_unused]] inline TranslationVector
    CameraToWorld(const TranslationVector &tv_cam,
                  const RotationMatrix &rm_imu,
                  const TranslationMatrix &tm_cam_to_imu,
                  const RotationMatrix &rm_cam_to_imu) {
        return (rm_cam_to_imu * rm_imu).transpose() * (tv_cam + tm_cam_to_imu);
    }

    [[maybe_unused]] inline TranslationVector
    WorldToCamera(const TranslationVector &tv_world,
                  const RotationMatrix &rm_imu_to_world,
                  const TranslationMatrix &tm_cam_to_imu,
                  const RotationMatrix &rm_cam_to_imu) {
        return (rm_cam_to_imu * rm_imu_to_world) * tv_world - tm_cam_to_imu;
    }

    inline cv::Point2f CameraToPicture(const cv::Mat& intrinsic_matrix,const Eigen::Vector3d& predict_camera_vector){
        Eigen::Matrix3d camera_matrix;
        cv::cv2eigen(intrinsic_matrix, camera_matrix);
        auto point = camera_matrix * predict_camera_vector / predict_camera_vector(2, 0);
        return {float(point[0]), float(point[1])};
    }
}

namespace coordinate::convert {
    /**
     * \brief Transform rectangular coordinate to spherical coordinate.
     * \tparam T Template type.
     * \param [in] rectangular Point (x, y, z) in rectangular coordinate system.
     * \param [out] spherical Point (yaw, pitch, distance) in spherical coordinate system.
     */
    template<typename T>
    inline void Rectangular2Spherical(const T rectangular[3], T spherical[3]) {
        spherical[0] = ceres::atan2(rectangular[0], rectangular[2]);
        spherical[1] = ceres::atan2(rectangular[1],
                                    ceres::sqrt(rectangular[0] * rectangular[0] + rectangular[2] * rectangular[2]));
        spherical[2] = ceres::sqrt(rectangular[0] * rectangular[0] +
                                   rectangular[1] * rectangular[1] +
                                   rectangular[2] * rectangular[2]);
    }

    /**
     * \brief Transform rectangular coordinate to spherical coordinate.
     * \param rectangular Point (x, y, z) in rectangular coordinate system.
     * \return Point (yaw, pitch, distance) in spherical coordinate system.
     */
    inline Eigen::Vector3d Rectangular2Spherical(const Eigen::Vector3d &rectangular) {
        Eigen::Vector3d spherical;
        spherical(0, 0) = atan2(rectangular(0, 0), rectangular(2, 0));
        spherical(1, 0) = atan2(rectangular(1, 0),
                                sqrt(rectangular(0, 0) * rectangular(0, 0) +
                                     rectangular(2, 0) * rectangular(2, 0)));
        spherical(2, 0) = sqrt(rectangular(0, 0) * rectangular(0, 0) +
                               rectangular(1, 0) * rectangular(1, 0) +
                               rectangular(2, 0) * rectangular(2, 0));
        return spherical;
    }
}

#endif  // COORDINATE_H_
