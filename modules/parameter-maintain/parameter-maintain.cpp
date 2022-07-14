//
// Created by screw on 2022/7/14.
//

#include "parameter-maintain.h"
#include "glog/logging.h"
#include "detector-rune/detector_rune_debug.h"

void ParameterMaintain::initDetectorRuneParameters() {
    try {
        config_.open(rune_detector_config_path, cv::FileStorage::READ);
        DLOG(INFO) << rune_detector_config_path ;
    } catch (const std::exception &) {
        DLOG(ERROR) << "Failed to open rune detector config file" << rune_detector_config_path << ".";
    }
    // Read config data.
    try {
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
    } catch (std::exception &) {
        DLOG(ERROR) << "Failed to load config of rune detector.";
    }
    config_.release();
}

void ParameterMaintain::saveDetectorRuneParameters() {
    // Open config file.
    try {
        config_.open(rune_detector_config_path, cv::FileStorage::WRITE);
    }
    catch (const std::exception &) {
        DLOG(ERROR) << "Failed to open rune detector config file " << rune_detector_config_path << ".";
        return;
    }

    // Write config data.
    try {
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
    } catch (std::exception &) {
        DLOG(ERROR) << "Failed to update config of rune detector.";
        return;
    }
    config_.release();
    DLOG(INFO) << "Config of rune detector is updated.";
}

void ParameterMaintain::initPredictorRuneParameters() {
    // Open config file.
    config_.open(rune_predictor_config_path, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        DLOG(ERROR) << "Failed to open rune detector config file " << rune_predictor_config_path << ".";
    }
    config_["AMPLITUDE"] >> rotational_speed_.a;
    config_["PALSTANCE"] >> rotational_speed_.w;
    config_["PHASE"] >> rotational_speed_.p;
    rotational_speed_.b = 2.090 - rotational_speed_.a;
    config_.release();
}

// doesn't need to save
void ParameterMaintain::savePredictorRuneParameters() {}

void ParameterMaintain::initPredictorArmorParameters() {
    config_.open(armor_predictor_config_path, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        DLOG(ERROR) << "Failed to open predict config file " << armor_predictor_config_path << ".";
    }
    config_["P_XZ_NOISE"] >> p_xz_noise_;
    config_["P_Y_NOISE"] >> p_y_noise_;
    config_["P_X_SPEED_NOISE"] >> p_x_speed_noise_;
    config_["P_Y_SPEED_NOISE"] >> p_y_speed_noise_;
    config_["M_X_NOISE"] >> m_x_noise_;
    config_["M_Y_NOISE"] >> m_y_noise_;
    config_["M_Z_NOISE"] >> m_z_noise_;
    config_["SHOOT_DELAY"] >> shoot_delay_;
    config_.release();
}

void ParameterMaintain::savePredictorArmorParameters() {
    config_.open(armor_predictor_config_path, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        DLOG(ERROR) << "Failed to open predict config file " << armor_predictor_config_path << ".";
    }
    // Write config data.
    config_ << "P_XZ_NOISE" << p_xz_noise_;
    config_ << "P_Y_NOISE" << p_y_noise_;
    config_ << "P_X_SPEED_NOISE" << p_x_speed_noise_;
    config_ << "P_Y_SPEED_NOISE" << p_y_speed_noise_;
    config_ << "M_X_NOISE" << m_x_noise_;
    config_ << "M_Y_NOISE" << m_y_noise_;
    config_ << "M_Z_NOISE" << m_z_noise_;
    config_ << "SHOOT_DELAY" << shoot_delay_;
    config_.release();
    LOG(INFO) << "Config of ekf is updated.";
}





