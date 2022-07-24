//
// Created by screw on 2022/7/21.
//

#ifndef PREDICTOR_RUNE_KALMAN_H_
#define PREDICTOR_RUNE_KALMAN_H_

#include "lang-feature-extension/disable-constructors.h"
#include "digital-twin/facilities/power_rune.h"
#include "data-structure/communication.h"
#include "math-tools/ekf.h"

class RunePredictorKalman : NO_COPY, NO_MOVE {
public:

    RunePredictorKalman();

    ~RunePredictorKalman() = default;

    bool Initialize();

    [[nodiscard]]SendPacket Run(const PowerRune &power_rune, AimModes aim_modes, float bullet_speed);

private:
    ExtendedKalmanFilter<5,1> ekf_;

    double last_rad;

    double rotate_speed = -1141514.65742;

    void InitializeEKF(double rad);
};

#endif //PREDICTOR_RUNE_KALMAN_H_
