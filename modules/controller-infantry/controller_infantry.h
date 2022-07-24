/**
 * Infantry controller header.
 * \author trantuan-20048607, screw-44
 * \date 2022.1.28
 * \warning NEVER include this file except in ./controller_infantry.cpp.
 */

#ifndef CONTROLLER_INFANTRY_H_
#define CONTROLLER_INFANTRY_H_

// Do NOT include anything to avoid being wrongly included.
#include "controller_infantry_debug.h"


class [[maybe_unused]] InfantryController final : public Controller {
public:
    bool Initialize() final;

    void Run() final;

private:

    /// Own registry in controller factory.
    [[maybe_unused]] static ControllerRegistry<InfantryController> infantry_controller_registry_;

    RuneDetectorNetwork rune_detector_network_;
    RunePredictor rune_predictor_;
    RunePredictorKalman rune_predictor_kalman_;
    PowerRune power_rune_;

    ControllerInfantryDebug controller_infantry_debug_;
};

#endif  // CONTROLLER_INFANTRY_H_
