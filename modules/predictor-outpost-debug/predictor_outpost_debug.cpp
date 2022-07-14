
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "predictor_outpost_debug.h"

bool OutpostPredictorDebug::Initialize(const std::string &config_path, bool debug_use_trackbar) {
    config_path_ = config_path;
    // Open config file.
    config_.open(config_path_, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open predict config file " << config_path_ << ".";
        return false;
    }
    // Read config data.
    if(     config_["SHOOT_DELAY"].empty() ||
            config_["DELTA_PITCH_UP"].empty() ||
            config_["DELTA_PITCH_DOWN"].empty() ||
            config_["DELTA_YAW_LEFT"].empty() ||
            config_["DELTA_YAW_RIGHT"].empty()){
        LOG(ERROR) << "Failed to read data form predict config file" << config_path_ << ".";
        return false;
    }
    config_["SHOOT_DELAY"] >> shoot_delay_;
    config_["DELTA_PITCH_UP"] >> delta_pitch_up_;
    config_["DELTA_PITCH_DOWN"] >> delta_pitch_down_;
    config_["DELTA_YAW_LEFT"] >> delta_yaw_left_;
    config_["DELTA_YAW_RIGHT"] >>delta_yaw_right_;
    config_.release();

    if (debug_use_trackbar && CmdlineArgParser::Instance().RunModeOutpost()) {
        debug::Trackbar<double>::Instance().AddTrackbar("delta_pitch_down:",
                                                        trackbar_windows_name_,
                                                        delta_pitch_down_,
                                                        kMax_delta_pitch_down);

        debug::Trackbar<double>::Instance().AddTrackbar("delta_pitch_up:",
                                                        trackbar_windows_name_,
                                                        delta_pitch_up_,
                                                        kMax_delta_pitch_up);

        debug::Trackbar<double>::Instance().AddTrackbar("delta_yaw_left:",
                                                        trackbar_windows_name_,
                                                        delta_yaw_left_,
                                                        kMax_delta_yaw_left);

        debug::Trackbar<double>::Instance().AddTrackbar("delta_yaw_right:",
                                                        trackbar_windows_name_,
                                                        delta_yaw_right_,
                                                        kMax_delta_yaw_right);

        debug::Trackbar<double>::Instance().AddTrackbar("shoot_delay:",
                                                        trackbar_windows_name_,
                                                        shoot_delay_,
                                                        kMax_shoot_delay_);
    }
    return true;
}




void OutpostPredictorDebug::Save() {
    config_.open(config_path_, cv::FileStorage::WRITE);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open rune ekf config file " << config_path_ << ".";
        return;
    }

    // Write config data.
    config_ << "P_XZ_NOISE" << delta_yaw_left_;
    config_ << "P_Y_NOISE" << delta_yaw_right_;
    config_ << "P_X_SPEED_NOISE" << delta_pitch_up_;
    config_ << "P_Y_SPEED_NOISE" << delta_pitch_down_;
    config_ << "SHOOT_DELAY" << shoot_delay_;
    config_.release();

    LOG(INFO) << "Config of ekf is updated.";
}
