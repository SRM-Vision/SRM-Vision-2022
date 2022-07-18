#ifndef PREDICTOR_ARMOR_DEBUG_H_
#define PREDICTOR_ARMOR_DEBUG_H_

#include <glog/logging.h>
#include "lang-feature-extension/attr-reader.h"
#include "parameter-maintain/parameter-maintain.h"
#include "debug-tools/trackbar.h"

template<unsigned int N_x, unsigned int N_y>
class ExtendedKalmanFilter;

class ArmorPredictorDebug {
public:
    inline static ArmorPredictorDebug &Instance() {
        static ArmorPredictorDebug _;
        return _;
    }

    bool Initialize(const std::string &controller_type_name)
    {
        parameter_maintain_.ManualInit(controller_type_name);
        return true;
    }

    void Save();

private:
    ParameterMaintain parameter_maintain_{"infantry"};
    double delta_pitch_ = 0.0;
    double delta_yaw_ = 0.0;

public:
    ATTR_READER(parameter_maintain_.p_xz_noise_, PredictedXZNoise)
    ATTR_READER(parameter_maintain_.p_y_noise_, PredictedYNoise)
    ATTR_READER(parameter_maintain_.p_x_speed_noise_, PredictedXSpeedNoise)
    ATTR_READER(parameter_maintain_.p_y_speed_noise_,PredictedYSpeedNoise)
    ATTR_READER(parameter_maintain_.p_y_acceleration_noise_,PredictedYAccelerationNoise)
    ATTR_READER(parameter_maintain_.p_x_acceleration_noise_,PredictedXAccelerationNoise)
    ATTR_READER(parameter_maintain_.m_x_noise_, MeasureXNoise)
    ATTR_READER(parameter_maintain_.m_y_noise_, MeasureYNoise)
    ATTR_READER(parameter_maintain_.m_z_noise_, MeasureZNoise)
    ATTR_READER(parameter_maintain_.shoot_delay_, ShootDelay)
    ATTR_READER(delta_pitch_, DeltaPitch)
    ATTR_READER(delta_yaw_, DeltaYaw)

private:
    const std::string trackbar_windows_name_ = "Armor Predictor Debug";

    const double kMax_p_xz_noise = 1;
    const double kMax_p_y_noise = 1;
    const double kMax_p_x_speed_noise = 1000;
    const double kMax_p_y_speed_noise = 1000;
    const double kMax_p_x_acceleration_noise = 10000;
    const double kMax_p_y_acceleration_noise = 10000;
    const double kMax_m_x_noise = 10;
    const double kMax_m_y_noise = 10;
    const int kMax_m_z_noise = 1000;
    const double kMax_shoot_delay = 0.5;
    const double kDelta_pitch = 1;
    const double kDelta_yaw = 0.5;

public:
    void addTrackbar();

    void AlterPredictCovMeasureCov(ExtendedKalmanFilter<7,3>& ekf) const;

};


#endif  // PREDICTOR_ARMOR_DEBUG_H_
