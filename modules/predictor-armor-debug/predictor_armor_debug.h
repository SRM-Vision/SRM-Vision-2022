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

    void Initialize(bool debug_use_trackbar = false);

private:
    const std::string trackbar_windows_name_ = "Armor Predictor Debug";

public:
    ATTR_READER(p_xyz_noise_, PredictedXZYNoise)

    ATTR_READER(p_xy_speed_noise_, PredictedXZYSpeedNoise)

    ATTR_READER(m_xy_noise_, MeasureXYNoise)

    ATTR_READER(m_z_noise_, MeasureZNoise)


private:
    const int kMax_p_xyz_noise = 1;
    const int  kMax_p_xy_speed_noise = 500;
    const int kMax_m_xy_noise = 10;
    const int  kMax_m_z_noise = 1000;

    double p_xyz_noise_ = 0.01;
    double p_xy_speed_noise_ = 100;
    double m_xy_noise_ = 1;
    double m_z_noise_ = 800;
};


#endif  // PREDICTOR_ARMOR_DEBUG_H_
