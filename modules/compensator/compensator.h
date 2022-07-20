//
// Created by lzy on 2022/5/6.
//

#ifndef COMPENSATOR_H_
#define COMPENSATOR_H_

#include "trajectory-solver/trajectory-solver.h"
#include <data-structure/communication.h>
#include <opencv2/core/persistence.hpp>
#include "glog/logging.h"
class Compensator{
public:
    inline static Compensator &Instance() {
        static Compensator _;
        return _;
    }
    bool Initialize(const std::string& robot_name_, double bullet_speed);
    void InitModel(double bullet_speed, const std::string& robot_name);
    void Offset(float &pitch, float & yaw, double bullet_speed, float &check_sum, double distance, AimModes mode = AimModes::kNormal);
    double PitchOffset(float &pitch, double bullet_speed, double distance, AimModes mode = AimModes::kNormal);
    double GetPlaneDistance(double distance, AimModes mode = AimModes::kNormal);
private:
    trajectory_solver::PitchAngleSolver angle_solver_{};
    double bullet_speed_{};

    std::string robot_name_{};
    // Offset后的数字表示弹速等级
    // 字母d表示用于计算plane_distance，字母p表示用于计算delta_pitch
    Eigen::Vector4f offset0d_{};
    Eigen::Vector4f offset0p_{};
    Eigen::Vector4f offset1d_{};
    Eigen::Vector4f offset1p_{};
    Eigen::Vector4f offset2d_{};
    Eigen::Vector4f offset2p_{};
};
#endif //COMPENSATOR_H_
