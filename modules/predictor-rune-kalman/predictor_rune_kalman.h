//
// Created by screw on 2022/7/21.
//

#ifndef PREDICTOR_RUNE_KALMAN_H_
#define PREDICTOR_RUNE_KALMAN_H_

#include "lang-feature-extension/disable-constructors.h"
#include "digital-twin/facilities/power_rune.h"
#include "data-structure/communication.h"

class RunePredictorKalman : NO_COPY, NO_MOVE {
public:

    RunePredictorKalman();

    ~RunePredictorKalman() = default;

    bool Initialize(const std::string &config_path);

    [[nodiscard]]SendPacket Run(const PowerRune &power_rune, AimModes aim_modes, float bullet_speed);

private:


};

#endif //PREDICTOR_RUNE_KALMAN_H_
