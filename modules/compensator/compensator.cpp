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
    config_["A0"] >> setoff00_[0];config_["B0"] >> setoff00_[1];config_["C0"] >> setoff00_[2];config_["D0"] >> setoff00_[3];
    config_["A1"] >> setoff01_[0];config_["B1"] >> setoff01_[1];config_["C1"] >> setoff01_[2];config_["D1"] >> setoff01_[3];
    config_["AA0"] >> setoff10_[0];config_["BB0"] >> setoff10_[1];config_["CC0"] >> setoff10_[2];config_["DD0"] >> setoff10_[3];
    config_["AA1"] >> setoff11_[0];config_["BB1"] >> setoff11_[1];config_["CC1"] >> setoff11_[2];config_["DD1"] >> setoff11_[3];
    config_["AAA0"] >> setoff20_[0];config_["BBB0"] >> setoff20_[1];config_["CCC0"] >> setoff20_[2];config_["DDD0"] >> setoff20_[3];
    config_["AAA1"] >> setoff21_[0];config_["BBB1"] >> setoff21_[1];config_["CCC1"] >> setoff21_[2];config_["DDD1"] >> setoff21_[3];
    return true;
}

void Compensator::Setoff(float &pitch,double bullet_speed, double distance,AimModes mode) {
    //TODO more mode
    if (pitch == 0 || bullet_speed == 0 || distance == 0)
        return;
    if (robot_name_ == "sentry_lower") {
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        float plane_distance = setoff00_[0] * distance - setoff00_[1];
        float delta_pitch = setoff01_[0] * plane_distance + setoff01_[1];
        pitch += delta_pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }else if (robot_name_ == "sentry_higher"){
        float plane_distance = 0,delta_pitch =0;
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }else if (robot_name_ == "hero") {
        float plane_distance = 0,delta_pitch = 0;
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        if ( bullet_speed ==10){
            plane_distance = setoff00_[0] * distance * distance * distance +
                             setoff00_[1] * distance * distance +
                             setoff00_[2] * distance +
                             setoff00_[3];
            delta_pitch = setoff01_[0] * plane_distance * plane_distance * plane_distance +
                          setoff01_[1] * plane_distance * plane_distance +
                          setoff01_[2] * plane_distance +
                          setoff01_[3];
        }
        else if( bullet_speed ==16){
            plane_distance = setoff10_[0] * distance * distance * distance +
                             setoff10_[1] * distance * distance +
                             setoff10_[2] * distance +
                             setoff10_[3];
            delta_pitch = setoff11_[0] * plane_distance * plane_distance * plane_distance +
                          setoff11_[1] * plane_distance * plane_distance +
                          setoff11_[2] * plane_distance +
                          setoff11_[3];
        }
        pitch += delta_pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    } else if (robot_name_ == "infantry") {
        float plane_distance = 0,delta_pitch = 0;
        DLOG(INFO) << "before setoff pitch:"<<pitch;
        if ( bullet_speed ==15){
            plane_distance = setoff00_[0] * distance * distance * distance +
                             setoff00_[1] * distance * distance +
                             setoff00_[2] * distance +
                             setoff00_[3];
            delta_pitch = setoff01_[0] * plane_distance * plane_distance * plane_distance +
                          setoff01_[1] * plane_distance * plane_distance +
                          setoff01_[2] * plane_distance +
                          setoff01_[3];
        }
        else if( bullet_speed ==18){
            plane_distance = setoff10_[0] * distance * distance * distance +
                             setoff10_[1] * distance * distance +
                             setoff10_[2] * distance +
                             setoff10_[3];
            delta_pitch = setoff11_[0] * plane_distance * plane_distance * plane_distance +
                          setoff11_[1] * plane_distance * plane_distance +
                          setoff11_[2] * plane_distance +
                          setoff11_[3];
        }
        else if ( bullet_speed ==30){
            plane_distance = setoff20_[0] * distance * distance * distance +
                             setoff20_[1] * distance * distance +
                             setoff20_[2] * distance +
                             setoff20_[3];
            delta_pitch = setoff21_[0] * plane_distance * plane_distance * plane_distance +
                          setoff21_[1] * plane_distance * plane_distance +
                          setoff21_[2] * plane_distance +
                          setoff21_[3];
        }
        pitch += delta_pitch;
        DLOG(INFO) << "after setoff pitch: "<<pitch;
    }
}
