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

    /// Save value to yaml
    [[maybe_unused]] void Save();

    ATTR_READER(split_gray_thresh_, SplitGrayThresh)

    ATTR_READER(min_bounding_box_area_, MinBoundingBoxArea)

    ATTR_READER(min_contour_area_, MinContourArea)

    ATTR_READER(max_contour_area_, MaxContourArea)

    ATTR_READER(min_armor_area_, MinArmorArea)

    ATTR_READER(max_armor_area_, MaxArmorArea)

    ATTR_READER(min_r_bounding_box_area_, MinRBoundingBoxArea)

    ATTR_READER(max_r_bounding_box_area_, MaxRBoundingBoxArea)

    ATTR_READER(max_encircle_r_rect_wh_deviation_, MaxEncircleRRectWHDeviation)

    ATTR_READER(min_armor_wh_ratio_, MinArmorWHRatio)

    ATTR_READER(max_armor_wh_ratio_, MaxArmorWHRatio)

    ATTR_READER(min_bounding_box_wh_ratio_, MinBoundingBoxWHRatio)

    ATTR_READER(max_bounding_box_wh_ratio_, MaxBoundingBoxWHRatio)

    ATTR_READER(channel_B_low_thresh_, LowBThresh)

    ATTR_READER(channel_G_low_thresh_, LowGThresh)

    ATTR_READER(channel_R_low_thresh_, LowRThresh)

    ATTR_READER(channel_B_high_thresh_, HighBThresh)

    ATTR_READER(channel_G_high_thresh_, HighGThresh)

    ATTR_READER(channel_R_high_thresh_, HighRThresh)

private:
    const std::string trackbar_window_name1_ = "Rune Detector Counter Debug";
    const std::string trackbar_window_name2_ = "Rune Detector Point Debug";
    cv::FileStorage config_;
    std::string config_path_;

    // Maximum trackbar values.
    const int kMaxThresholdTrackbar = 255;
    const int kMaxRuneAreaTrackbar = 20000;
    const int kMaxRAreaTrackbar = 10000;
    const int kMaxRatioTrackbar = 1000;

    // Trackbar value cache.
    int split_gray_thresh_{};                  ///< Binarization threshold.
    int channel_B_low_thresh_{};
    int channel_G_low_thresh_{};
    int channel_R_low_thresh_{};
    int channel_B_high_thresh_{};
    int channel_G_high_thresh_{};
    int channel_R_high_thresh_{};
    int min_bounding_box_area_{};
    int min_contour_area_{};
    int max_contour_area_{};
    int min_armor_area_{};
    int max_armor_area_{};
    int min_r_bounding_box_area_{};
    int max_r_bounding_box_area_{};
    int max_encircle_r_rect_wh_deviation_{};  ///< Maximum energy center encircle rect weight - height deviation.
    double min_armor_wh_ratio_{};             ///< Minimum armor weight / height ratio.
    double max_armor_wh_ratio_{};             ///< Maximum armor weight / height ratio.
    double min_bounding_box_wh_ratio_{};      ///< Minimum fan rect weight / height ratio.
    double max_bounding_box_wh_ratio_{};      ///< Maximum fan rect weight / height ratio.
};

#endif  // DETECTOR_RUNE_DEBUG_H_
