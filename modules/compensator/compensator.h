//
// Created by lzy on 2022/5/6.
//

#ifndef COMPENSATOR_H_
#define COMPENSATOR_H_

#include <data-structure/communication.h>
#include <opencv2/core/persistence.hpp>
#include "glog/logging.h"
class Compensator{
public:
    inline static Compensator &Instance() {
        static Compensator _;
        return _;
    }
    bool Initialize(std::string robot_name_);
    void Setoff(float &pitch,double bullet_speed,double distance,AimModes mode = AimModes::kNormal);

private:
    std::string robot_name_{};
    // setoff后的数字表示弹速等级
    // 字母d表示用于计算plane_distance，字母p表示用于计算delta_pitch
    Eigen::Vector4f setoff0d_{};
    Eigen::Vector4f setoff0p_{};
    Eigen::Vector4f setoff1d_{};
    Eigen::Vector4f setoff1p_{};
    Eigen::Vector4f setoff2d_{};
    Eigen::Vector4f setoff2p_{};
};
#endif //COMPENSATOR_H_
