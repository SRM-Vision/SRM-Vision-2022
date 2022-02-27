#include "predictor_armor_debug.h"

void ArmorPredictorDebug::Initialize(bool debug_use_trackbar) {
    if(debug_use_trackbar)
    {
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
    }
}



