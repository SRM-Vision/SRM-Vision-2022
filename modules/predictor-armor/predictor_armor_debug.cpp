#include "predictor_armor_debug.h"
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "math-tools/ekf.h"

void ArmorPredictorDebug::Save() {
    parameter_maintain_.savePredictorArmorParameters();
}

void ArmorPredictorDebug::addTrackbar() {
    debug::Trackbar<double>::Instance().AddTrackbar(
            "p_xz_noise:",
            trackbar_windows_name_,
            parameter_maintain_.p_xz_noise_,
            kMax_p_xz_noise);

    debug::Trackbar<double>::Instance().AddTrackbar(
            "p_y_noise:",
            trackbar_windows_name_,
            parameter_maintain_.p_y_noise_,
            kMax_p_y_noise);

    debug::Trackbar<double>::Instance().AddTrackbar(
            "p_x_speed_noise:",
            trackbar_windows_name_,
            parameter_maintain_.p_x_speed_noise_,
            kMax_p_x_speed_noise);

    debug::Trackbar<double>::Instance().AddTrackbar(
            "p_y_speed_noise:",
            trackbar_windows_name_,
            parameter_maintain_.p_y_speed_noise_,
            kMax_p_y_speed_noise);

    debug::Trackbar<double>::Instance().AddTrackbar(
            "m_x_noise:",
            trackbar_windows_name_,
            parameter_maintain_.m_x_noise_,
            kMax_m_x_noise);

    debug::Trackbar<double>::Instance().AddTrackbar(
            "m_y_noise:",
            trackbar_windows_name_,
            parameter_maintain_.m_y_noise_,
            kMax_m_y_noise);

    debug::Trackbar<double>::Instance().AddTrackbar(
            "m_z_noise:",
            trackbar_windows_name_,
            parameter_maintain_.m_z_noise_,
            kMax_m_z_noise);

    debug::Trackbar<double>::Instance().AddTrackbar(
            "shoot_delay",
            trackbar_windows_name_,
            parameter_maintain_.shoot_delay_,
            kMax_shoot_delay);

    debug::Trackbar<double>::Instance().AddTrackbar(
            "delta_pitch",
            trackbar_windows_name_,
            delta_pitch_,
            kDelta_pitch);

    debug::Trackbar<double>::Instance().AddTrackbar(
            "delta_yaw",
            trackbar_windows_name_,
            delta_yaw_,
            kDelta_yaw);
}

void ArmorPredictorDebug::AlterPredictCovMeasureCov(ExtendedKalmanFilter<5,3>& ekf) const {
    ekf.predict_cov_ << PredictedXZNoise(), 0, 0, 0, 0,
                        0, PredictedXSpeedNoise(), 0, 0, 0,
                        0, 0, PredictedYNoise(), 0, 0,
                        0, 0, 0, PredictedYSpeedNoise(), 0,
                        0, 0, 0, 0, PredictedXZNoise();
    ekf.measure_cov_ << MeasureXNoise(), 0, 0,
                        0, MeasureYNoise(), 0,
                        0, 0, MeasureZNoise();
}