/**
 * Power rune detector class header.
 * \author screw-44, LIYunzhe1408, LemonadeJJ
 * \date 2022.2.16
 */

#ifndef DETECTOR_RUNE_DEBUG_H_
#define DETECTOR_RUNE_DEBUG_H_

#include <glog/logging.h>
#include "lang-feature-extension/attr_reader.h"
#include "lang-feature-extension/disable_constructors.h"
#include "debug-tools/trackbar.h"

class RuneDetectorDebug : NO_COPY, NO_MOVE {
public:
    inline static RuneDetectorDebug &Instance() {
        static RuneDetectorDebug _;
        return _;
    }

    void Initialize(const std::string &config_path, bool debug_use_trackbar = false);

    /// Save Rune Detector parameters to yaml
    void Save();

    ATTR_READER(split_gray_thresh_, SplitGrayThresh)

    ATTR_READER(min_bounding_box_area_, MinBoundingBoxArea)

    ATTR_READER(min_fan_area_, MinFanArea)

    ATTR_READER(max_fan_area_, MaxFanArea)

    ATTR_READER(min_armor_area_, MinArmorArea)

    ATTR_READER(max_armor_area_, MaxArmorArea)

    ATTR_READER(min_R_area_, MinRArea)

    ATTR_READER(max_R_area_, MaxRArea)

    ATTR_READER(max_R_wh_deviation_, MaxRWHDeviation)

    ATTR_READER(min_armor_wh_ratio_, MinArmorWHRatio)

    ATTR_READER(max_armor_wh_ratio_, MaxArmorWHRatio)

    ATTR_READER(min_fan_wh_ratio_, MinFanWHRatio)

    ATTR_READER(max_fan_wh_ratio_, MaxFanWHRatio)

    ATTR_READER(kernel_size_, KernelSize)

    ATTR_READER(delta_u_, DeltaU)

    ATTR_READER(delta_v_, DeltaV)

private:
    const std::string trackbar_window_name_ = "Rune Detector Debug";
    cv::FileStorage config_;
    std::string config_path_;

    // Maximum trackbar values.
    const int kMaxThresholdTrackbar = 255;
    const int kMaxRuneAreaTrackbar = 10000;
    const int kMaxArmorAreaTrackbar = 5000;
    const int kMaxRAreaTrackbar = 2000;
    const int kMaxRatioTrackbar = 1000;
    const int kMaxCompensation = 100;

    // Trackbar value cache.
    int split_gray_thresh_{};                  ///< Binarization threshold.
    int min_bounding_box_area_{};
    int min_fan_area_{};
    int max_fan_area_{};
    int min_armor_area_{};
    int max_armor_area_{};
    int kernel_size_{};                        ///< MorphologyEx kernel size
    int min_R_area_{};
    int max_R_area_{};
    int max_R_wh_deviation_{};  ///< Maximum energy center encircle rect weight - height deviation.
    double min_armor_wh_ratio_{};             ///< Minimum armor weight / height ratio.
    double max_armor_wh_ratio_{};             ///< Maximum armor weight / height ratio.
    double min_fan_wh_ratio_{};      ///< Minimum fan rect weight / height ratio.
    double max_fan_wh_ratio_{};      ///< Maximum fan rect weight / height ratio.
    int delta_u_{};   ///< Horizontal ballistic compensation
    int delta_v_{};   ///< Vertical ballistic compensation
};

#endif  // DETECTOR_RUNE_DEBUG_H_
