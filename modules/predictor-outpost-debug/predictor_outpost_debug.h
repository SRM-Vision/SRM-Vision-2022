//
// Created by lzy on 2022/7/14.
//

#ifndef PREDICTOR_OUTPOST_DEBUG_H_
#define PREDICTOR_OUTPOST_DEBUG_H_

#include <glog/logging.h>
#include "debug-tools/trackbar.h"
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
     * \Brief Collect armors, get center points and decide auto-shoot signal.
     * @param config_path the config_path of data file.
     * @param debug_use_trackbar whether use trackbar.
     * @return whether initialize successfully.
     */
    bool Initialize(const std::string &config_path, bool debug_use_trackbar = true);

    /**
     * \Brief Save the data in the param file.
     */
    void Save();

private:
    const std::string trackbar_windows_name_ = "Outpost Predictor Debug";
    cv::FileStorage config_;
    std::string config_path_;

public:
    ATTR_READER(delta_pitch_up_, DeltaPitchUp);

    ATTR_READER(delta_pitch_down_, DeltaPitchDown);

    ATTR_READER(delta_yaw_left_, DeltaYawLeft);

    ATTR_READER(delta_yaw_right_, DeltaYawRight);

    ATTR_READER(shoot_delay_, ShootDelay);

private:

    const double kMax_shoot_delay_ = 1;
    const double kMax_delta_pitch_up = 1;
    const double kMax_delta_pitch_down = 1;
    const double kMax_delta_yaw_left = 0.5;
    const double kMax_delta_yaw_right = 0.5;

    double shoot_delay_ = 0.02;
    double delta_pitch_up_ = 0.0;
    double delta_pitch_down_ = 0.0;
    double delta_yaw_left_ = 0.0;
    double delta_yaw_right_ = 0.0;
};


#endif //PREDICTOR_OUTPOST_DEBUG_H_
