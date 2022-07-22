//
// Created by screw on 2022/7/14.
//

#ifndef PREDICTOR_RUNE_DEBUG_H_
#define PREDICTOR_RUNE_DEBUG_H_

#include "parameter-maintain/parameter-maintain.h"
#include "predictor-rune.h"
#include "debug-tools/trackbar.h"
#include "lang-feature-extension/attr-reader.h"

class RunePredictorDebug {
public:
    inline static RunePredictorDebug &Instance() {
        static RunePredictorDebug _;
        return _;
    }

    RunePredictorDebug() = default;

    /// Save Rune Predictor parameters to yaml
    void Save() {
        parameter_maintain_.savePredictorRuneParameters();
    }

private:
    ParameterMaintain parameter_maintain_{"infantry"};

    std::string trackbar_window_name_ = "Rune Predictor";
    const int kMaxCompensation = 4000;

public:
    ATTR_READER(parameter_maintain_.delta_u_, DeltaU)

    ATTR_READER(parameter_maintain_.delta_v_, DeltaV)

    ATTR_READER(parameter_maintain_.compensate_time_, CompensateTime)

    void addTrackbar();
};

#endif //PREDICTOR_RUNE_DEBUG_H_