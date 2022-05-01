#include "predictor_armor_debug.h"

void ArmorPredictorDebug::Initialize(const std::string &config_path, bool debug_use_trackbar) {

    config_path_ = config_path;
    // Open config file.
    config_.open(config_path_, cv::FileStorage::READ);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open ekf config file " << config_path_ << ".";
    }
    // Read config data.
    config_["P_XYZ_NOISE"] >> p_xyz_noise_;
    config_["P_XY_SPEED_NOISE"] >> p_xy_speed_noise_;
    config_["M_XY_NOISE"] >> m_xy_noise_;
    config_["M_Z_NOISE"] >> m_z_noise_;
    config_.release();
    if (debug_use_trackbar) {
        debug::Trackbar<double>::Instance().AddTrackbar("p_xyz_noise:",
                                                        trackbar_windows_name_,
                                                        p_xyz_noise_,
                                                        kMax_p_xyz_noise);

        debug::Trackbar<double>::Instance().AddTrackbar("p_xy_speed_noise:",
                                                        trackbar_windows_name_,
                                                        p_xy_speed_noise_,
                                                        kMax_p_xy_speed_noise);

        debug::Trackbar<double>::Instance().AddTrackbar("m_xy_noise:",
                                                        trackbar_windows_name_,
                                                        m_xy_noise_,
                                                        kMax_m_xy_noise);

        debug::Trackbar<double>::Instance().AddTrackbar("m_z_noise:",
                                                        trackbar_windows_name_,
                                                        m_z_noise_,
                                                        kMax_m_z_noise);
        debug::Trackbar<double>::Instance().AddTrackbar("delta_pitch",
                                                        trackbar_windows_name_,
                                                        delta_pitch_,
                                                        kDelta_pitch);
    }
}

void ArmorPredictorDebug::Save() {
    // Open config file.
    config_.open(config_path_, cv::FileStorage::WRITE);
    if (!config_.isOpened()) {
        LOG(ERROR) << "Failed to open rune ekf config file " << config_path_ << ".";
        return;
    }

    // Write config data.
    config_ << "P_XYZ_NOISE" << p_xyz_noise_;
    config_ << "P_XY_SPEED_NOISE" << p_xy_speed_noise_;
    config_ << "M_XY_NOISE" << m_xy_noise_;
    config_ << "M_Z_NOISE" << m_z_noise_;
    config_.release();
    LOG(INFO) << "Config of ekf is updated.";

}


