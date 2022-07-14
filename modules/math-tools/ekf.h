/**
 * Extended Kalman Filter templated class header.
 * \author lzy20020320
 * \date 2022.1.30
 * \note This file is only for internal use of implementation.
 *   To use predictor in other modules, include the corresponding headers.
 */

#ifndef EKF_H_
#define EKF_H_

#include <ceres/jet.h>
#include <Eigen/Dense>
#include "glog/logging.h"
#include "predictor-armor/predictor_armor_debug.h"

/**
 * \brief Extended kalman filter template.
 * \tparam N_x Data length of x.
 * \tparam N_y Data length of y.
 */
template<unsigned int N_x, unsigned int N_y>
class ExtendedKalmanFilter {
public:
    typedef Eigen::Matrix<double, N_x, N_x> MatrixXX;
    typedef Eigen::Matrix<double, N_y, N_x> MatrixYX;
    typedef Eigen::Matrix<double, N_x, N_y> MatrixXY;
    typedef Eigen::Matrix<double, N_y, N_y> MatrixYY;
    typedef Eigen::Matrix<double, N_x, 1> VectorX;
    typedef Eigen::Matrix<double, N_y, 1> VectorY;

    ExtendedKalmanFilter()
            : x_estimate_(VectorX::Zero()),
              status_cov_(MatrixXX::Identity()) {
        predict_cov_ << 0.01, 0, 0, 0, 0,
                        0, 100, 0, 0, 0,
                        0, 0, 0.01, 0, 0,
                        0, 0, 0, 100, 0,
                        0, 0, 0, 0, 0.01;
        measure_cov_ << 1, 0, 0,
                        0, 1, 0,
                        0, 0, 800;
    }

    inline void Initialize(const VectorX &x = VectorX::Zero()) {
        x_estimate_ = x;
    }

    template<typename Func>
    VectorX Predict(Func &&func) {
        AlterPredictCovMeasureCov(ArmorPredictorDebug::Instance().PredictedXZNoise(),
                                  ArmorPredictorDebug::Instance().PredictedYNoise(),
                                  ArmorPredictorDebug::Instance().PredictedXSpeedNoise(),
                                  ArmorPredictorDebug::Instance().PredictedYSpeedNoise(),
                                  ArmorPredictorDebug::Instance().MeasureXNoise(),
                                  ArmorPredictorDebug::Instance().MeasureYNoise(),
                                  ArmorPredictorDebug::Instance().MeasureZNoise());
        ceres::Jet<double, N_x> x_estimated_auto_jet[N_x];
        for (auto i = 0; i < N_x; ++i) {
            x_estimated_auto_jet[i].a = x_estimate_[i];
            x_estimated_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_x> x_predict_auto_jet[N_x];
        func(x_estimated_auto_jet, x_predict_auto_jet);
        for (auto i = 0; i < N_x; ++i) {
            x_predict_[i] = x_predict_auto_jet[i].a;
            predict_jacobi_.block(i, 0, 1, N_x) = x_predict_auto_jet[i].v.transpose();
        }
        status_cov_ = predict_jacobi_ * status_cov_ * predict_jacobi_.transpose() + predict_cov_;
        return x_predict_;
    }

    template<typename Func>
    VectorX Update(Func &&func, const VectorY &y) {
        ceres::Jet<double, N_x> x_predict_auto_jet[N_x];
        for (auto i = 0; i < N_x; ++i) {
            x_predict_auto_jet[i].a = x_predict_[i];
            x_predict_auto_jet[i].v[i] = 1;
        }

        ceres::Jet<double, N_x> y_predict_auto_jet[N_y];
        func(x_predict_auto_jet, y_predict_auto_jet);
        for (auto i = 0; i < N_y; ++i) {
            y_predict_[i] = y_predict_auto_jet[i].a;
            measure_jacobi_.block(i, 0, 1, N_x) = y_predict_auto_jet[i].v.transpose();
        }

        kalman_gain_ = status_cov_ *
                       measure_jacobi_.transpose() *
                       (measure_jacobi_ * status_cov_ * measure_jacobi_.transpose() +
                        measure_cov_).inverse();
        x_estimate_ = x_predict_ + kalman_gain_ * (y - y_predict_);
        status_cov_ = (MatrixXX::Identity() - kalman_gain_ * measure_jacobi_) * status_cov_;

        return x_estimate_;
    }

    void AlterPredictCovMeasureCov(double p_xz_noise, double p_y_noise, double p_x_speed_noise, double p_y_speed_noise,
                              double m_x_noise, double m_y_noise, double m_z_noise) {
        predict_cov_ << p_xz_noise, 0, 0, 0, 0,
                        0, p_x_speed_noise, 0, 0, 0,
                        0, 0, p_y_noise, 0, 0,
                        0, 0, 0, p_y_speed_noise, 0,
                        0, 0, 0, 0, p_xz_noise;
        measure_cov_ << m_x_noise, 0, 0,
                        0, m_y_noise, 0,
                        0, 0, m_z_noise;
    }

    VectorX x_estimate_;       ///< Estimated status var. [Xe]
    VectorX x_predict_;        ///< Predicted status var. [Xp]
    MatrixXX predict_jacobi_;  ///< Prediction jacobi matrix. [F]
    MatrixYX measure_jacobi_;  ///< Measurement jacobi matrix. [H]
    MatrixXX status_cov_;      ///< Status covariance matrix. [P]
    MatrixXX predict_cov_;     ///< Prediction covariance matrix. [Q]
    MatrixYY measure_cov_;     ///< Measurement covariance matrix. [R]
    MatrixXY kalman_gain_;     ///< Kalman gain. [K]
    VectorY y_predict_;        ///< Predicted measuring var. [Yp]
};

#endif  // EKF_H_
