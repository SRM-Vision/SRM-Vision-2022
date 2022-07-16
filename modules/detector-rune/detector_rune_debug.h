//
// Created by screw on 2022/7/14.
//

#ifndef DETECTOR_RUNE_DEBUG_H_
#define DETECTOR_RUNE_DEBUG_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <parameter-maintain/parameter-maintain.h>
#include "debug-tools/trackbar.h"

class RuneDetectorDebug{
public:
    inline static RuneDetectorDebug &Instance() {
        static RuneDetectorDebug _;
        return _;
    }

    RuneDetectorDebug() = default;

    void Initialize() {}

    /// Save Rune Detector parameters to yaml
    void Save()
    {
        parameter_maintain_.saveDetectorRuneParameters();
    }

    // parameter sector
private:
    ParameterMaintain parameter_maintain_{"infantry"};

public:
    ATTR_READER(parameter_maintain_.split_gray_thresh_, SplitGrayThresh)
    ATTR_READER(parameter_maintain_.min_bounding_box_area_, MinBoundingBoxArea)
    ATTR_READER(parameter_maintain_.min_fan_area_, MinFanArea)
    ATTR_READER(parameter_maintain_.max_fan_area_, MaxFanArea)
    ATTR_READER(parameter_maintain_.min_armor_area_, MinArmorArea)
    ATTR_READER(parameter_maintain_.max_armor_area_, MaxArmorArea)
    ATTR_READER(parameter_maintain_.min_R_area_, MinRArea)
    ATTR_READER(parameter_maintain_.max_R_area_, MaxRArea)
    ATTR_READER(parameter_maintain_.max_R_wh_deviation_, MaxRWHDeviation)
    ATTR_READER(parameter_maintain_.min_armor_wh_ratio_, MinArmorWHRatio)
    ATTR_READER(parameter_maintain_.max_armor_wh_ratio_, MaxArmorWHRatio)
    ATTR_READER(parameter_maintain_.min_fan_wh_ratio_, MinFanWHRatio)
    ATTR_READER(parameter_maintain_.max_fan_wh_ratio_, MaxFanWHRatio)
    ATTR_READER(parameter_maintain_.kernel_size_, KernelSize)
    ATTR_READER(parameter_maintain_.delta_u_, DeltaU)
    ATTR_READER(parameter_maintain_.delta_v_, DeltaV)


// trackbar sector
public:

    void addTrackbar();

private:
    std::string trackbar_window_name_ = "testing name";
    // Maximum trackbar values.
    const int kMaxThresholdTrackbar = 255;
    const int kMaxRuneAreaTrackbar = 10000;
    const int kMaxArmorAreaTrackbar = 5000;
    const int kMaxRAreaTrackbar = 2000;
    const int kMaxRatioTrackbar = 1000;
    const int kMaxCompensation = 2000;
};

#endif //DETECTOR_RUNE_DEBUG_H_
