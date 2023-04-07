//
// Created by screw on 2022/7/24.
//

#ifndef COMPENTATOR_APPROXIMATE_H_
#define COMPENTATOR_APPROXIMATE_H_

#include "trajectory-solver/trajectory-solver.h"
#include <data-structure/communication.h>
#include <opencv2/core/persistence.hpp>
#include "glog/logging.h"

class CompensatorApproximate{
public:
    bool Initialize(const std::string &robot_name_);
    void Offset(float &pitch, double bullet_speed, double distance, AimModes mode);

private:
    double bullet_speed_{};
    std::string robot_name_{};
    // Offset后的数字表示弹速等级
    // 字母d表示用于计算plane_distance
    Eigen::Vector4f offset0d_{};
    Eigen::Vector4f offset1d_{};
    Eigen::Vector4f offset2d_{};
    // 字母p表示用于计算delta_pitch
    Eigen::Vector4f offset0p_{};
    Eigen::Vector4f offset1p_{};
    Eigen::Vector4f offset2p_{};
};

#endif //COMPENTATOR_APPROXIMATE_H_
