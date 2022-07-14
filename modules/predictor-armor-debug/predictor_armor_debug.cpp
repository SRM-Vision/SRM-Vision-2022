#include "predictor_armor_debug.h"
#include "cmdline-arg-parser/cmdline_arg_parser.h"

bool ArmorPredictorDebug::Initialize(const std::string &config_path, bool debug_use_trackbar) {

    config_path_ = config_path;
    // Open config file.
    config_.open(config_path_, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open predict config file " << config_path_ << ".";
        return false;
    }
    // Read config data.
    if(     config_["P_XZ_NOISE"].empty() ||
            config_["P_Y_NOISE"].empty() ||
            config_["P_X_SPEED_NOISE"].empty() ||
            config_["P_Y_SPEED_NOISE"].empty() ||
            config_["M_X_NOISE"].empty() ||
            config_["M_Y_NOISE"].empty() ||
            config_["M_Z_NOISE"].empty() ||
            config_["SHOOT_DELAY"].empty()){
        LOG(ERROR) << "Failed to read data form predict config file" << config_path_ << ".";
        return false;
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

    if (debug_use_trackbar && !CmdlineArgParser::Instance().RuneModeRune()) {
        debug::Trackbar<double>::Instance().AddTrackbar("p_xz_noise:",
                                                        trackbar_windows_name_,
                                                        p_xz_noise_,
                                                        kMax_p_xz_noise);

        debug::Trackbar<double>::Instance().AddTrackbar("p_y_noise:",
                                                        trackbar_windows_name_,
                                                        p_y_noise_,
                                                        kMax_p_y_noise);

        debug::Trackbar<double>::Instance().AddTrackbar("p_x_speed_noise:",
                                                        trackbar_windows_name_,
                                                        p_x_speed_noise_,
                                                        kMax_p_x_speed_noise);

        debug::Trackbar<double>::Instance().AddTrackbar("p_y_speed_noise:",
                                                        trackbar_windows_name_,
                                                        p_y_speed_noise_,
                                                        kMax_p_y_speed_noise);

        debug::Trackbar<double>::Instance().AddTrackbar("m_x_noise:",
                                                        trackbar_windows_name_,
                                                        m_x_noise_,
                                                        kMax_m_x_noise);

        debug::Trackbar<double>::Instance().AddTrackbar("m_y_noise:",
                                                        trackbar_windows_name_,
                                                        m_y_noise_,
                                                        kMax_m_y_noise);

        debug::Trackbar<double>::Instance().AddTrackbar("m_z_noise:",
                                                        trackbar_windows_name_,
                                                        m_z_noise_,
                                                        kMax_m_z_noise);
        debug::Trackbar<double>::Instance().AddTrackbar("shoot_delay",
                                                        trackbar_windows_name_,
                                                        shoot_delay_,
                                                        kMax_shoot_delay);
        debug::Trackbar<double>::Instance().AddTrackbar("delta_pitch",
                                                        trackbar_windows_name_,
                                                        delta_pitch_,
                                                        kDelta_pitch);

        debug::Trackbar<double>::Instance().AddTrackbar("delta_yaw",
                                                        trackbar_windows_name_,
                                                        delta_yaw_,
                                                        kDelta_yaw);
    }
    return true;
}

void ArmorPredictorDebug::Save() {
    // Open config file.
    config_.open(config_path_, cv::FileStorage::WRITE);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open rune ekf config file " << config_path_ << ".";
        return;
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


