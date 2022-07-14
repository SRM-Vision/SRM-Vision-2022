//
// Created by lzy on 2022/7/14.
//

#ifndef PREDICTOR_OUTPOST_DEBUG_H_
#define PREDICTOR_OUTPOST_DEBUG_H_

#include <glog/logging.h>
#include "debug-tools/trackbar.h"
#include "parameter-maintain/parameter-maintain.h"

/**
 * \Brief Create trackbar to adjust params.
 * \note Maybe only used to measure the offset
 */
class OutpostPredictorDebug {
public:
    inline static OutpostPredictorDebug &Instance() {
        static OutpostPredictorDebug _;
        return _;
    }

    /**
     * \Brief Save the data in the param file.
     */
    void Save();

private:
    ParameterMaintain parameter_maintain_{"hero"};

public:
    ATTR_READER(parameter_maintain_.delta_pitch_up_, DeltaPitchUp);

    ATTR_READER(parameter_maintain_.delta_pitch_down_, DeltaPitchDown);

    ATTR_READER(parameter_maintain_.delta_yaw_left_, DeltaYawLeft);

    ATTR_READER(parameter_maintain_.delta_yaw_right_, DeltaYawRight);

    ATTR_READER(parameter_maintain_.outpost_shoot_delay_, ShootDelay);

public:
    void addTrackbar();

private:
    const std::string trackbar_windows_name_ = "Outpost Predictor Debug";
    // Maximum trackbar values.
    const double kMax_shoot_delay_ = 1;
    const double kMax_delta_pitch_up = 1;
    const double kMax_delta_pitch_down = 1;
    const double kMax_delta_yaw_left = 0.5;
    const double kMax_delta_yaw_right = 0.5;

};


#endif //PREDICTOR_OUTPOST_DEBUG_H_
