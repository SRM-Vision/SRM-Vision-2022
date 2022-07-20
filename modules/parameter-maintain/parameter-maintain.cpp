//
// Created by screw on 2022/7/14.
//

#include "parameter-maintain.h"
#include "glog/logging.h"
#include "detector-rune/detector_rune_debug.h"

bool ParameterMaintain::initDetectorRuneParameters() {
    config_.open(rune_detector_config_path, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open rune detector config file " << rune_detector_config_path << ".";
        return false;
    }

    config_["MIN_FAN_AREA"] >> min_fan_area_;
    config_["MAX_FAN_AREA"] >> max_fan_area_;
    config_["MIN_ARMOR_AREA"] >> min_armor_area_;
    config_["MAX_ARMOR_AREA"] >> max_armor_area_;
    config_["MAX_R_WH_DEVIATION"] >> max_R_wh_deviation_;
    config_["MIN_BOUNDING_BOX_AREA"] >> min_bounding_box_area_;
    config_["MIN_R_AREA"] >> min_R_area_;
    config_["MAX_R_AREA"] >> max_R_area_;
    config_["SPLIT_GRAY_THRESH"] >> split_gray_thresh_;
    config_["MIN_ARMOR_WH_RATIO"] >> min_armor_wh_ratio_;
    config_["MAX_ARMOR_WH_RATIO"] >> max_armor_wh_ratio_;
    config_["MIN_FAN_WH_RATIO"] >> min_fan_wh_ratio_;
    config_["MAX_FAN_WH_RATIO"] >> max_fan_wh_ratio_;
    config_["MAX_R_WH_DEVIATION"] >> max_R_wh_deviation_;
    config_["KERNEL_SIZE"] >> kernel_size_;
    config_["DELTA_U"] >> delta_u_;
    config_["DELTA_V"] >> delta_v_;
    config_.release();
    return true;
}

bool ParameterMaintain::saveDetectorRuneParameters() {
    // Open config file.
    config_.open(rune_detector_config_path, cv::FileStorage::WRITE);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open rune detector config file " << rune_detector_config_path << ".";
        return false;
    }

    // Write config data.
    config_ << "MIN_FAN_AREA" << min_fan_area_;
    config_ << "MAX_FAN_AREA" << max_fan_area_;
    config_ << "MIN_ARMOR_AREA" << min_armor_area_;
    config_ << "MAX_ARMOR_AREA" << max_armor_area_;
    config_ << "MIN_R_AREA" << min_R_area_;
    config_ << "MAX_R_AREA" << max_R_area_;
    config_ << "SPLIT_GRAY_THRESH" << split_gray_thresh_;
    config_ << "MIN_ARMOR_WH_RATIO" << min_armor_wh_ratio_;
    config_ << "MAX_ARMOR_WH_RATIO" << max_armor_wh_ratio_;
    config_ << "MIN_BOUNDING_BOX_AREA" << min_bounding_box_area_;
    config_ << "MIN_FAN_WH_RATIO" << min_fan_wh_ratio_;
    config_ << "MAX_FAN_WH_RATIO" << max_fan_wh_ratio_;
    config_ << "MAX_R_WH_DEVIATION" << max_R_wh_deviation_;
    config_ << "KERNEL_SIZE" << kernel_size_;
    config_ << "DELTA_U" << delta_u_;
    config_ << "DELTA_V" << delta_v_;
    config_.release();
    DLOG(INFO) << "Config of rune detector is updated.";
    return true;
}

bool ParameterMaintain::initPredictorRuneParameters() {
    // Open config file.
    config_.open(rune_predictor_config_path, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open rune predictor config file " << rune_predictor_config_path << ".";
        return false;
    }
    config_["AMPLITUDE"] >> rotational_speed_.a;
    config_["PALSTANCE"] >> rotational_speed_.w;
    config_["PHASE"] >> rotational_speed_.p;
    rotational_speed_.b = 2.090 - rotational_speed_.a;
    config_.release();
    return true;
}

// doesn't need to save
bool ParameterMaintain::savePredictorRuneParameters() { return true; }

bool ParameterMaintain::initPredictorArmorParameters() {
    config_.open(armor_predictor_config_path, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open armor predictor config file " << armor_predictor_config_path << ".";
        return false;
    }
    config_["P_XZ_NOISE"] >> p_xz_noise_;
    config_["P_Y_NOISE"] >> p_y_noise_;
    config_["P_X_SPEED_NOISE"] >> p_x_speed_noise_;
    config_["P_Y_SPEED_NOISE"] >> p_y_speed_noise_;
    config_["P_X_ACCELERATION_NOISE"] >> p_x_acceleration_noise_;
    config_["P_Y_ACCELERATION_NOISE"] >> p_y_acceleration_noise_;
    config_["M_X_NOISE"] >> m_x_noise_;
    config_["M_Y_NOISE"] >> m_y_noise_;
    config_["M_Z_NOISE"] >> m_z_noise_;
    config_["SHOOT_DELAY"] >> shoot_delay_;
    config_.release();
    return true;
}

bool ParameterMaintain::savePredictorArmorParameters() {
    config_.open(armor_predictor_config_path, cv::FileStorage::WRITE);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open armor predictor config file " << armor_predictor_config_path << ".";
        return false;
    }
    // Write config data.
    config_ << "P_XZ_NOISE" << p_xz_noise_;
    config_ << "P_Y_NOISE" << p_y_noise_;
    config_ << "P_X_SPEED_NOISE" << p_x_speed_noise_;
    config_ << "P_Y_SPEED_NOISE" << p_y_speed_noise_;
    config_ << "P_X_ACCELERATION_NOISE" << p_x_acceleration_noise_;
    config_ << "P_Y_ACCELERATION_NOISE" << p_y_acceleration_noise_;
    config_ << "M_X_NOISE" << m_x_noise_;
    config_ << "M_Y_NOISE" << m_y_noise_;
    config_ << "M_Z_NOISE" << m_z_noise_;
    config_ << "SHOOT_DELAY" << shoot_delay_;
    config_.release();
    LOG(INFO) << "Config of ekf is updated.";
    return true;
}

bool ParameterMaintain::initPredictorOutpostParameters() {
    config_.open(outpost_predictor_config_path, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open outpost predictor config file " << outpost_predictor_config_path << ".";
        return false;
    }
    config_["SHOOT_DELAY_3M"] >> outpost_shoot_delay_3m_;
    config_["SHOOT_DELAY_5M"] >> outpost_shoot_delay_5m_;
    config_["SHOOT_DELAY_6M"] >> outpost_shoot_delay_6m_;
    config_["DELTA_PITCH_UP"] >> delta_pitch_up_;
    config_["DELTA_PITCH_DOWN"] >> delta_pitch_down_;
    config_["DELTA_YAW_LEFT"] >> delta_yaw_left_;
    config_["DELTA_YAW_RIGHT"] >> delta_yaw_right_;
    config_.release();
    return true;
}

bool ParameterMaintain::savePredictorOutpostParameters() {
    config_.open(outpost_predictor_config_path, cv::FileStorage::WRITE);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open outpost predictor config file " << outpost_predictor_config_path << ".";
        return false;
    }

    // Write config data.
    config_ << "SHOOT_DELAY_3M" << outpost_shoot_delay_3m_;
    config_ << "SHOOT_DELAY_5M" << outpost_shoot_delay_5m_;
    config_ << "SHOOT_DELAY_6M" << outpost_shoot_delay_6m_;
    config_ << "DELTA_PITCH_UP" << delta_pitch_up_;
    config_ << "DELTA_PITCH_DOWN" << delta_pitch_down_;
    config_ << "DELTA_YAW_LEFT" << delta_yaw_left_;
    config_ << "DELTA_YAW_RIGHT" << delta_yaw_right_;
    config_.release();

    DLOG(INFO) << "Config of outpost is updated.";
    return true;
}





