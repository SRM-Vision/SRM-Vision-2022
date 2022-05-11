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
    Eigen::Vector4f setoff0_{};
    Eigen::Vector4f setoff1_{};
};
#endif //COMPENSATOR_H_
