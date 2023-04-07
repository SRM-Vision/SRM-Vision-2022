//
// Created by screw on 2022/7/24.
//

#include "compensator-approximate.h"


#define CALCULATE_POLYNOMIAL(_result, _coefficients, _x)  \
    do {                                                  \
        _result =                                         \
        _coefficients[0] * _x * _x * _x +                 \
        _coefficients[1] * _x * _x +                      \
        _coefficients[2] * _x +                           \
        _coefficients[3];                                 \
    } while(0);


bool CompensatorApproximate::Initialize(const std::string &robot_name) {

    cv::FileStorage config_;
    config_.open("../config/" + robot_name + "/offset-param.yaml", cv::FileStorage::READ);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open offset file ";
        return false;
    }
    if (config_["A0"].empty() || config_["B0"].empty() || config_["C0"].empty() || config_["D0"].empty() ||
        config_["A1"].empty() || config_["B1"].empty() || config_["C1"].empty() || config_["D1"].empty() ||
        config_["AA0"].empty() || config_["BB0"].empty() || config_["CC0"].empty() || config_["DD0"].empty() ||
        config_["AA1"].empty() || config_["BB1"].empty() || config_["CC1"].empty() || config_["DD1"].empty() ||
        config_["AAA0"].empty() || config_["BBB0"].empty() || config_["CCC0"].empty() || config_["DDD0"].empty() ||
        config_["AAA1"].empty() || config_["BBB1"].empty() || config_["CCC1"].empty() || config_["DDD1"].empty()) {
        LOG(ERROR) << "The data in offset file is wrong";
        return false;
    }

    robot_name_ = robot_name;
    config_["A0"] >> offset0d_[0];
    config_["B0"] >> offset0d_[1];
    config_["C0"] >> offset0d_[2];
    config_["D0"] >> offset0d_[3];
    config_["A1"] >> offset0p_[0];
    config_["B1"] >> offset0p_[1];
    config_["C1"] >> offset0p_[2];
    config_["D1"] >> offset0p_[3];
    config_["AA0"] >> offset1d_[0];
    config_["BB0"] >> offset1d_[1];
    config_["CC0"] >> offset1d_[2];
    config_["DD0"] >> offset1d_[3];
    config_["AA1"] >> offset1p_[0];
    config_["BB1"] >> offset1p_[1];
    config_["CC1"] >> offset1p_[2];
    config_["DD1"] >> offset1p_[3];
    config_["AAA0"] >> offset2d_[0];
    config_["BBB0"] >> offset2d_[1];
    config_["CCC0"] >> offset2d_[2];
    config_["DDD0"] >> offset2d_[3];
    config_["AAA1"] >> offset2p_[0];
    config_["BBB1"] >> offset2p_[1];
    config_["CCC1"] >> offset2p_[2];
    config_["DDD1"] >> offset2p_[3];
    return true;
}

void CompensatorApproximate::Offset(float &pitch, double bullet_speed, double distance, AimModes mode) {
    if (pitch == 0 || bullet_speed == 0 || distance == 0)
        return;

    float plane_distance, delta_pitch;
    DLOG(INFO) << "before offset pitch:" << pitch;

    if (robot_name_[0] == 's') {  // 首字母是s是哨兵
        CALCULATE_POLYNOMIAL(plane_distance, offset0d_, distance)
        CALCULATE_POLYNOMIAL(delta_pitch, offset0p_, plane_distance)
    }

    if (robot_name_ == "hero") {   // hero has only one shoot speed
        CALCULATE_POLYNOMIAL(plane_distance, offset0d_, distance)
        CALCULATE_POLYNOMIAL(delta_pitch, offset0p_, plane_distance)
    }

    if (robot_name_[0] == 'i') {   // 首字母是i是步兵
        if (bullet_speed <= 15) {
            CALCULATE_POLYNOMIAL(plane_distance, offset0d_, distance)
            CALCULATE_POLYNOMIAL(delta_pitch, offset0p_, plane_distance)
        }
        if (bullet_speed <= 18) {
            CALCULATE_POLYNOMIAL(plane_distance, offset1d_, distance)
            CALCULATE_POLYNOMIAL(delta_pitch, offset1p_, plane_distance)
        }
        if (bullet_speed <= 30) {
            CALCULATE_POLYNOMIAL(plane_distance, offset2d_, distance)
            CALCULATE_POLYNOMIAL(delta_pitch, offset2p_, plane_distance)
        }
    }

    pitch -= delta_pitch;
    LOG(INFO)<<"aaa:"<<delta_pitch;
    DLOG(INFO) << "after offset pitch: " << pitch;
}
