//
// Created by lzy on 2022/5/6.
//
#include"compensator.h"
bool Compensator::Initialize(std::string robot_name) {
    cv::FileStorage config_;
    config_.open("../config/"+robot_name+"/setoff-param.yaml", cv::FileStorage::READ);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open setoff file ";
        return false;
    }
    if( config_["A0"].empty()||config_["B0"].empty()||
        config_["C0"].empty()||config_["D0"].empty()||
        config_["A1"].empty()||config_["B1"].empty()||
        config_["C1"].empty()||config_["D1"].empty()){
        LOG(ERROR) << "The data in setoff file is wrong";
        return false;
    }
    robot_name_ = robot_name;
    config_["A0"] >> setoff0_[0];
    config_["B0"] >> setoff0_[1];
    config_["C0"] >> setoff0_[2];
    config_["D0"] >> setoff0_[3];
    config_["A1"] >> setoff1_[0];
    config_["B1"] >> setoff1_[1];
    config_["C1"] >> setoff1_[2];
    config_["D1"] >> setoff1_[3];
    return true;
}

void Compensator::Setoff(float &pitch,double bullet_speed, double distance) {
    if (robot_name_ == "sentry_lower") {
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        float plane_distance = setoff0_[0] * distance - setoff0_[1];
        float delta_pitch = setoff1_[0] * plane_distance + setoff1_[1];
        pitch += delta_pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }else if (robot_name_ == "sentry_higher"){
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }else if (robot_name_ == "hero") {
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    } else if (robot_name_ == "infantry") {
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }
}
