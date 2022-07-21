/**
 * Infantry controller header.
 * \author trantuan-20048607, screw-44
 * \date 2022.1.28
 * \warning NEVER include this file except in ./controller_infantry.cpp.
 */

#ifndef CONTROLLER_INFANTRY_4_H_
#define CONTROLLER_INFANTRY_4_H_

// Do NOT include anything to avoid being wrongly included.
#include "controller_infantry4_debug.h"


class [[maybe_unused]] Infantry4Controller final : public Controller {
public:
    bool Initialize() final;

    void Run() final;

private:

    /// Own registry in controller factory.
    [[maybe_unused]] static ControllerRegistry<Infantry4Controller> infantry4_controller_registry_;

    RuneDetector rune_detector_;
    RuneDetectorNetwork rune_detector_network_;
    RunePredictor rune_predictor_;
    PowerRune power_rune_;

    ControllerInfantry4Debug controller_infantry4_debug_;
};

#endif  // CONTROLLER_INFANTRY_4_H_
