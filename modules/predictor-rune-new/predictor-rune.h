#pragma once

#include <ceres/ceres.h>
#include "data-structure/buffer.h"
#include "data-structure/frame.h"
#include "data-structure/communication.h"
#include "digital-twin/facilities/power_rune.h"

namespace predictor::rune {
    class RunePredictor : NO_COPY, NO_MOVE {
    public:
        RunePredictor() :
                debug_(false) {}

        ~RunePredictor() = default;

    private:
        bool debug_;
    };
}
