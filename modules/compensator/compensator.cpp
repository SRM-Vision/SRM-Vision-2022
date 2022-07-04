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
    if( config_["A0"].empty()||config_["B0"].empty()||config_["C0"].empty()||config_["D0"].empty()||
        config_["A1"].empty()||config_["B1"].empty()||config_["C1"].empty()||config_["D1"].empty()||
        config_["AA0"].empty()||config_["BB0"].empty()||config_["CC0"].empty()||config_["DD0"].empty()||
        config_["AA1"].empty()||config_["BB1"].empty()||config_["CC1"].empty()||config_["DD1"].empty()||
        config_["AAA0"].empty()||config_["BBB0"].empty()||config_["CCC0"].empty()||config_["DDD0"].empty()||
        config_["AAA1"].empty()||config_["BBB1"].empty()||config_["CCC1"].empty()||config_["DDD1"].empty()){
        LOG(ERROR) << "The data in setoff file is wrong";
        return false;
    }
    robot_name_ = robot_name;
    config_["A0"] >> setoff0d_[0];config_["B0"] >> setoff0d_[1];config_["C0"] >> setoff0d_[2];config_["D0"] >> setoff0d_[3];
    config_["A1"] >> setoff0p_[0];config_["B1"] >> setoff0p_[1];config_["C1"] >> setoff0p_[2];config_["D1"] >> setoff0p_[3];
    config_["AA0"] >> setoff1d_[0];config_["BB0"] >> setoff1d_[1];config_["CC0"] >> setoff1d_[2];config_["DD0"] >> setoff1d_[3];
    config_["AA1"] >> setoff1p_[0];config_["BB1"] >> setoff1p_[1];config_["CC1"] >> setoff1p_[2];config_["DD1"] >> setoff1p_[3];
    config_["AAA0"] >> setoff2d_[0];config_["BBB0"] >> setoff2d_[1];config_["CCC0"] >> setoff2d_[2];config_["DDD0"] >> setoff2d_[3];
    config_["AAA1"] >> setoff2p_[0];config_["BBB1"] >> setoff2p_[1];config_["CCC1"] >> setoff2p_[2];config_["DDD1"] >> setoff2p_[3];
    return true;
}

void Compensator::Setoff(float &pitch,double bullet_speed, double distance,AimModes mode) {
    //TODO more mode
    if (pitch == 0 || bullet_speed == 0 || distance == 0)
        return;
    if (robot_name_ == "sentry_lower") {
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        float plane_distance = setoff0d_[0] * distance - setoff0d_[1];
        float delta_pitch = setoff0p_[0] * plane_distance + setoff0p_[1];
        pitch += delta_pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }
    else if (robot_name_ == "sentry_higher"){
        float plane_distance = 0,delta_pitch =0;
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }
    else if (robot_name_ == "hero") {
        float plane_distance = 0,delta_pitch = 0;
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        if ( bullet_speed ==10){
            plane_distance = setoff0d_[0] * distance * distance * distance +
                             setoff0d_[1] * distance * distance +
                             setoff0d_[2] * distance +
                             setoff0d_[3];
            delta_pitch = setoff0p_[0] * plane_distance * plane_distance * plane_distance +
                          setoff0p_[1] * plane_distance * plane_distance +
                          setoff0p_[2] * plane_distance +
                          setoff0p_[3];
        }
        else if( bullet_speed ==16){
            plane_distance = setoff1d_[0] * distance * distance * distance +
                             setoff1d_[1] * distance * distance +
                             setoff1d_[2] * distance +
                             setoff1d_[3];
            delta_pitch = setoff1p_[0] * plane_distance * plane_distance * plane_distance +
                          setoff1p_[1] * plane_distance * plane_distance +
                          setoff1p_[2] * plane_distance +
                          setoff1p_[3];
        }
        pitch += delta_pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }
    else if (robot_name_[0] == 'i') {   // 首字母是i是步兵
        float plane_distance = 0,delta_pitch = 0;
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        if ( bullet_speed ==15){
            plane_distance = setoff0d_[0] * distance * distance * distance +
                             setoff0d_[1] * distance * distance +
                             setoff0d_[2] * distance +
                             setoff0d_[3];
            delta_pitch = setoff0p_[0] * plane_distance * plane_distance * plane_distance +
                          setoff0p_[1] * plane_distance * plane_distance +
                          setoff0p_[2] * plane_distance +
                          setoff0p_[3];
        }
        else if( bullet_speed ==18){
            plane_distance = setoff1d_[0] * distance * distance * distance +
                             setoff1d_[1] * distance * distance +
                             setoff1d_[2] * distance +
                             setoff1d_[3];
            delta_pitch = setoff1p_[0] * plane_distance * plane_distance * plane_distance +
                          setoff1p_[1] * plane_distance * plane_distance +
                          setoff1p_[2] * plane_distance +
                          setoff1p_[3];
        }
        else if ( bullet_speed ==30){
            plane_distance = setoff2d_[0] * distance * distance * distance +
                             setoff2d_[1] * distance * distance +
                             setoff2d_[2] * distance +
                             setoff2d_[3];
            delta_pitch = setoff2p_[0] * plane_distance * plane_distance * plane_distance +
                          setoff2p_[1] * plane_distance * plane_distance +
                          setoff2p_[2] * plane_distance +
                          setoff2p_[3];
        }
        pitch += delta_pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }
}
