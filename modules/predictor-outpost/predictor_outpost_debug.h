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

    bool Initialize(const std::string &controller_type_name)
    {
        parameter_maintain_.ManualInit(controller_type_name);
        return true;
    }

    /**
     * \Brief Save the data in the param file.
     */
    void Save();

private:
    ParameterMaintain parameter_maintain_{"hero"};

public:
    ATTR_READER(parameter_maintain_.delta_pitch_60cm6m_, DeltaPitch60CM6M);

    ATTR_READER(parameter_maintain_.delta_pitch_20cm5m_, DeltaPitch20CM5M);

    ATTR_READER(parameter_maintain_.delta_pitch_0cm5m_, DeltaPitch0CM5M);

    ATTR_READER(parameter_maintain_.outpost_shoot_delay_60cm6m_, ShootDelay60CM6M);

    ATTR_READER(parameter_maintain_.outpost_shoot_delay_20cm5m_, ShootDelay20CM5M);

    ATTR_READER(parameter_maintain_.outpost_shoot_delay_0cm5m_, ShootDelay0CM5M);

public:
    void addTrackbar();

private:
    const std::string trackbar_windows_name_ = "Outpost Predictor Debug";
    // Maximum trackbar values.
    const double kMax_shoot_delay_ = 1;
    const double kMax_delta_pitch_up = 1;


};


#endif //PREDICTOR_OUTPOST_DEBUG_H_
