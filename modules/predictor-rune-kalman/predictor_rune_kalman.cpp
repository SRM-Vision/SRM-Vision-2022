//
// Created by screw on 2022/7/21.
//

#include "predictor_rune_kalman.h"

struct PredictFunction {
    PredictFunction() : delta_t(0) {}

//    template<typename T>

    double delta_t;
};


RunePredictorKalman::RunePredictorKalman() {

}

bool RunePredictorKalman::Initialize(const std::string &config_path) {
    return false;
}

SendPacket RunePredictorKalman::Run(const PowerRune &power_rune, AimModes aim_modes, float bullet_speed) {
    return SendPacket();
}
