//
// Created by screw on 2022/7/21.
//
#include <math.h>
#include "predictor_rune_kalman.h"


struct PredictFunction {
    PredictFunction() : t_(0) {}

    template<typename T>
    void operator()(const T x_0[5], T x[5]) {
        x[0] = x_0[1]*sin(x_0[2]*t_ + x_0[3]) + 2.09 -x_0[1];  /// < V
        x[1] = x_0[1];  /// < A
        x[2] = x_0[2];  /// < W
        x[3] = x_0[3];  /// < theta
        x[4] = x_0[4];  /// < c
    }

    double t_;
};

struct MeasureFunction {
    template<typename T>
    void operator()(const T x[5], T y[1]) {
        y[0] = x[0];
    }
};


RunePredictorKalman::RunePredictorKalman() {

}

bool RunePredictorKalman::Initialize() {
    return false;
}

void RunePredictorKalman::InitializeEKF(const double rad_v) {
    Eigen::Matrix<double, 5, 1> x_real;
    Eigen::Matrix<double, 5, 5> predict_cov;
    Eigen::Matrix<double, 1, 1> measure_cov;
    x_real << rad_v, ///< 1.0875
            0.9125,   ///< 0.780 - 1.045
            1.942,   ///< 1.884 - 2.000
            0,    ///< -3.14 - 3.14
            0;
    predict_cov << 1,0,0,0,0,
                   0,10,0,0,0,
                   0,0,10,0,0,
                   0,0,0,10,0,
                   0,0,0,0,10;
    measure_cov << 1;
    ekf_.Initialize(x_real,predict_cov,measure_cov);
}

SendPacket RunePredictorKalman::Run(const PowerRune &power_rune, AimModes aim_modes, float bullet_speed) {

    auto current_rad = algorithm::Atan2Float(power_rune.RtpVec().y, power_rune.RtpVec().x);
    if (rotate_speed == -1141514.65742)
    {
        InitializeEKF(current_rad);
        rotate_speed = 0;
        return {};
    }
    rotate_speed = current_rad - last_rad;
    DLOG(INFO) << "rotated speed: " << rotate_speed;

    PredictFunction predict_function;
    MeasureFunction measure_function;
    predict_function.t_ = power_rune.CurrentTime() * 1e-3;
    Eigen::Matrix<double, 1, 1> y_real;
    y_real << rotate_speed;
    last_rad = current_rad;
    ekf_.Predict(predict_function);
    auto x_e = ekf_.Update(measure_function, y_real);
    DLOG(INFO) << "x_e: \n" << x_e << "\n";
    double bullet_fly_time = 7 / bullet_speed + 5;
    double vector_size = std::sqrt(
            std::pow(power_rune.RtpVec().x, 2)
            + std::pow(power_rune.RtpVec().y, 2));
    double _rad = std::atan2(power_rune.RtpVec().y, power_rune.RtpVec().x);
    cv::Point2f offset_point = cv::Point2f(
            cos(_rad+bullet_fly_time*x_e[0])*vector_size, sin(_rad+bullet_fly_time*x_e[0])*vector_size);
    cv::Point2f draw_point = power_rune.CenterR() + offset_point;


    return {draw_point.x,draw_point.y, 0, 0, 0, 0, 0,0, 0, 0, 0, 0,0};
}
