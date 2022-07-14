//
// Created by screw on 2022/7/14.
//

#ifndef PREDICTOR_RUNE_DEBUG_H_
#define PREDICTOR_RUNE_DEBUG_H_

#include "parameter-maintain/parameter-maintain.h"
#include "predictor-rune.h"
#include "lang-feature-extension/attr-reader.h"


class RunePredictorDebug
{
public:
    inline static RunePredictorDebug &Instance() {
        static RunePredictorDebug _;
        return _;
    }

    RunePredictorDebug() = default;

    void Save()
    {
        parameter_maintain_.savePredictorRuneParameters();
    }

private:
    ParameterMaintain parameter_maintain_{"infantry"};

public:
    ATTR_READER(parameter_maintain_.rotational_speed_, RotationalSpeed)

};

#endif //PREDICTOR_RUNE_DEBUG_H_
