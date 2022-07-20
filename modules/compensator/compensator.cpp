//
// Created by lzy on 2022/5/6.
//
#include"compensator.h"

bool Compensator::Initialize(const std::string &robot_name, double bullet_speed) {
    bullet_speed_ = bullet_speed;
    InitModel(bullet_speed_, robot_name);

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

void
Compensator::Offset(float &pitch, double bullet_speed, float &check_sum, double distance, AimModes mode) {
    // TODO more mode
    if (pitch == 0 || bullet_speed == 0 || distance == 0)
        return;
    if (robot_name_ == "sentry_lower") {
        DLOG(INFO) << "before offset pitch:" << pitch;
        float plane_distance = offset0d_[0] * distance - offset0d_[1];
        float delta_pitch = offset0p_[0] * plane_distance + offset0p_[1];
        pitch -= delta_pitch;
        DLOG(INFO) << "after offset pitch: " << pitch;
    } else if (robot_name_ == "sentry_higher") {
        float plane_distance = 0, delta_pitch = 0;
        DLOG(INFO) << "before offset pitch:" << pitch;
        plane_distance = offset2d_[0] * distance * distance * distance +
                         offset2d_[1] * distance * distance +
                         offset2d_[2] * distance +
                         offset2d_[3];
        delta_pitch = offset2p_[0] * plane_distance * plane_distance * plane_distance +
                      offset2p_[1] * plane_distance * plane_distance +
                      offset2p_[2] * plane_distance +
                      offset2p_[3];
        pitch -= delta_pitch;
        check_sum -= delta_pitch;
        DLOG(INFO) << "after offset pitch: " << pitch;
    } else if (robot_name_ == "hero") {
        float plane_distance = 0, delta_pitch = 0, delta_yaw = 0;
        DLOG(INFO) << "before offset pitch:" << pitch;
        if (mode == AimModes::kOutPost) {
            if (distance > 3) {
                plane_distance = offset2d_[0] * distance * distance * distance +
                                 offset2d_[1] * distance * distance +
                                 offset2d_[2] * distance +
                                 offset2d_[3];
                delta_pitch = offset2p_[0] * plane_distance * plane_distance * plane_distance +
                              offset2p_[1] * plane_distance * plane_distance +
                              offset2p_[2] * plane_distance +
                              offset2p_[3];
            } else delta_pitch = 0;
        } else {
            if (bullet_speed == 10) {
                plane_distance = offset0d_[0] * distance * distance * distance +
                                 offset0d_[1] * distance * distance +
                                 offset0d_[2] * distance +
                                 offset0d_[3];
                delta_pitch = offset0p_[0] * plane_distance * plane_distance * plane_distance +
                              offset0p_[1] * plane_distance * plane_distance +
                              offset0p_[2] * plane_distance +
                              offset0p_[3];
            } else if (bullet_speed == 16) {
                plane_distance = offset1d_[0] * distance * distance * distance +
                                 offset1d_[1] * distance * distance +
                                 offset1d_[2] * distance +
                                 offset1d_[3];
                delta_pitch = offset1p_[0] * plane_distance * plane_distance * plane_distance +
                              offset1p_[1] * plane_distance * plane_distance +
                              offset1p_[2] * plane_distance +
                              offset1p_[3];
            }
            if (distance < 2) delta_pitch = 0.065;
        }
        pitch -= delta_pitch;
        check_sum -= (delta_pitch + delta_yaw);
        DLOG(INFO) << "after offset pitch: " << pitch;
    } else if (robot_name_[0] == 'i') {   // 首字母是i是步兵
        float plane_distance = 0, delta_pitch = 0;
        DLOG(INFO) << "before offset pitch:" << pitch;
        if (bullet_speed == 15) {
            plane_distance = offset0d_[0] * distance * distance * distance +
                             offset0d_[1] * distance * distance +
                             offset0d_[2] * distance +
                             offset0d_[3];
            delta_pitch = offset0p_[0] * plane_distance * plane_distance * plane_distance +
                          offset0p_[1] * plane_distance * plane_distance +
                          offset0p_[2] * plane_distance +
                          offset0p_[3];
        } else if (bullet_speed == 18) {
            plane_distance = offset1d_[0] * distance * distance * distance +
                             offset1d_[1] * distance * distance +
                             offset1d_[2] * distance +
                             offset1d_[3];
            delta_pitch = offset1p_[0] * plane_distance * plane_distance * plane_distance +
                          offset1p_[1] * plane_distance * plane_distance +
                          offset1p_[2] * plane_distance +
                          offset1p_[3];
        } else if (bullet_speed == 30) {
            plane_distance = offset2d_[0] * distance * distance * distance +
                             offset2d_[1] * distance * distance +
                             offset2d_[2] * distance +
                             offset2d_[3];
            delta_pitch = offset2p_[0] * plane_distance * plane_distance * plane_distance +
                          offset2p_[1] * plane_distance * plane_distance +
                          offset2p_[2] * plane_distance +
                          offset2p_[3];
        }
        pitch -= delta_pitch;
        check_sum -= delta_pitch;
        DLOG(INFO) << "after offset pitch: " << pitch;
    }
}

double Compensator::PitchOffset(float &pitch, double bullet_speed, double distance, AimModes mode) {
    if (bullet_speed_ != bullet_speed)  // When bullet-speed is changed, it needs to refit model.
        InitModel(bullet_speed, robot_name_);

    float plane_distance, delta_pitch;
    DLOG(INFO) << "Pitch to compensate: " << pitch;
    if (robot_name_[0] == 'i') {   // Infantry's first char is 'i'.
        if (bullet_speed == 15) {
            plane_distance = float(offset0d_[0] * distance * distance * distance +
                                   offset0d_[1] * distance * distance +
                                   offset0d_[2] * distance +
                                   offset0d_[3]);
        } else if (bullet_speed == 18) {
            plane_distance = float(offset1d_[0] * distance * distance * distance +
                                   offset1d_[1] * distance * distance +
                                   offset1d_[2] * distance +
                                   offset1d_[3]);
        } else {
            plane_distance = float(offset2d_[0] * distance * distance * distance +
                                   offset2d_[1] * distance * distance +
                                   offset2d_[2] * distance +
                                   offset2d_[3]);
        }
        DLOG(INFO) << "plane_distance: " << plane_distance;
        angle_solver_.UpdateParam(0.13, plane_distance);
        auto res = angle_solver_.Solve(-CV_PI / 6, CV_PI / 3, 0.01, 16);
        delta_pitch = float(res.x()) - pitch;

        DLOG(INFO) << "Compensated pitch: " << res.x();
        return delta_pitch;
    } else if (robot_name_ == "hero") {
        if (mode == AimModes::kOutPost) {
            plane_distance = float(offset0d_[0] * distance * distance * distance +
                                   offset0d_[1] * distance * distance +
                                   offset0d_[2] * distance +
                                   offset0d_[3]);
        }
        angle_solver_.UpdateParam(1.2, plane_distance);
        auto res = angle_solver_.Solve(-CV_PI / 6, CV_PI / 3, 0.01, 16);
        delta_pitch = float(res.x()) - pitch;

        DLOG(INFO) << "Compensated pitch: " << pitch;
        return delta_pitch;
    }
}

void Compensator::InitModel(double bullet_speed, const std::string &robot_name) {
    if (robot_name[0] == 'i') {
        auto f = trajectory_solver::AirResistanceModel();
        f.SetParam(0.48, 994, 30, 0.017, 0.0032);
        auto a = trajectory_solver::BallisticModel();
        a.SetParam(f, 31);
        angle_solver_ = trajectory_solver::PitchAngleSolver();
        angle_solver_.SetParam(a, bullet_speed, 0.32, 0.18, 1);
        return;
    } else if (robot_name == "hero") {
        auto f = trajectory_solver::AirResistanceModel();
        f.SetParam(0.26, 994, 36, 0.0425, 0.041);
        auto a = trajectory_solver::BallisticModel();
        a.SetParam(f, 31);
        angle_solver_ = trajectory_solver::PitchAngleSolver();
        angle_solver_.SetParam(a, bullet_speed, 0.43, 1.2, 1);
        return;
    }
}

double Compensator::GetPlaneDistance(double distance, AimModes mode) {
    if(mode == AimModes::kOutPost) {
        auto plane_distance = float(offset0d_[0] * distance * distance * distance +
                                    offset0d_[1] * distance * distance +
                                    offset0d_[2] * distance +
                                    offset0d_[3]);
        return plane_distance;
    }
    return 0;
}



