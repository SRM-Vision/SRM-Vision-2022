#ifndef PREDICTOR_ARMOR_DEBUG_H_
#define PREDICTOR_ARMOR_DEBUG_H_

#include <glog/logging.h>
#include "lang-feature-extension/attr_reader.h"
#include "predictor_armor_debug.h"
#include "debug-tools/trackbar.h"

class ArmorPredictorDebug {
public:
    inline static ArmorPredictorDebug &Instance() {
        static ArmorPredictorDebug _;
        return _;
    }

    bool Initialize(const std::string &config_path, bool debug_use_trackbar = true);

    void Save();

private:
    const std::string trackbar_windows_name_ = "Armor Predictor Debug";
    cv::FileStorage config_;
    std::string config_path_;

public:
    ATTR_READER(p_xz_noise_, PredictedXZNoise)

    ATTR_READER(p_y_noise_, PredictedYNoise)

    ATTR_READER(p_x_speed_noise_, PredictedXSpeedNoise)

    ATTR_READER(p_y_speed_noise_,PredictedYSpeedNoise)

    ATTR_READER(m_x_noise_, MeasureXNoise)

    ATTR_READER(m_y_noise_, MeasureYNoise)

    ATTR_READER(m_z_noise_, MeasureZNoise)

    ATTR_READER(delta_pitch_, DeltaPitch)


private:
    const double kMax_p_xz_noise = 1;
    const double kMax_p_y_noise = 1;
    const double kMax_p_x_speed_noise = 1000;
    const double kMax_p_y_speed_noise = 1000;
    const double kMax_m_x_noise = 10;
    const double kMax_m_y_noise = 10;
    const int kMax_m_z_noise = 1000;
    const double kDelta_pitch = 1;

    double p_xz_noise_ = 0.01;
    double p_y_noise_ = 0.01;
    double p_x_speed_noise_ = 10;
    double p_y_speed_noise_ = 10;
    double m_x_noise_ = 1;
    double m_y_noise_ = 1;
    double m_z_noise_ = 800;
    double delta_pitch_ = 0.0;
};


#endif  // PREDICTOR_ARMOR_DEBUG_H_
