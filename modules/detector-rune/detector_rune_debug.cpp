//
// Created by screw on 2022/7/14.
//

#include "detector_rune_debug.h"

void RuneDetectorDebug::addTrackbar() {
    /// --- binary threshold ---
    debug::Trackbar<int>::Instance().AddTrackbar("Binary threshold (0-255):",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.split_gray_thresh_,
                                                 kMaxThresholdTrackbar);

    /// --- kernel size ---
    debug::Trackbar<int>::Instance().AddTrackbar("kernel size(0 - 10):",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.kernel_size_,
                                                 kMaxRatioTrackbar / 100);

    /// --- min bounding box area ---
    debug::Trackbar<int>::Instance().AddTrackbar("Min Bounding box (0-10000):",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.min_bounding_box_area_,
                                                 kMaxRuneAreaTrackbar);
    /// --- min contour area ---
    debug::Trackbar<int>::Instance().AddTrackbar("Min fan area (0-5000):",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.min_fan_area_,
                                                 kMaxArmorAreaTrackbar);

    /// --- max contour area ---
    debug::Trackbar<int>::Instance().AddTrackbar("Max fan area (0-5000)",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.max_fan_area_,
                                                 kMaxArmorAreaTrackbar);

    /// --- min armor w:h ---
    debug::Trackbar<double>::Instance().AddTrackbar("Min fan weight / height ratio (0-10000 / 1e4)",
                                                    trackbar_window_name_,
                                                    parameter_maintain_.min_fan_wh_ratio_,
                                                    kMaxRatioTrackbar * 1e-3);

    /// --- max armor w:h ---
    debug::Trackbar<double>::Instance().AddTrackbar("Max fan weight / height ratio (0-10000 / 1e4)",
                                                    trackbar_window_name_,
                                                    parameter_maintain_.max_fan_wh_ratio_,
                                                    kMaxRatioTrackbar * 1e-3);

    /// --- min armor area ---
    debug::Trackbar<int>::Instance().AddTrackbar("Min armor area (0-5000)",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.min_armor_area_,
                                                 kMaxArmorAreaTrackbar);

    /// --- max armor area ---
    debug::Trackbar<int>::Instance().AddTrackbar("Max armor area (0-5000)",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.max_armor_area_,
                                                 kMaxArmorAreaTrackbar);

    /// --- min R w:h ---
    debug::Trackbar<double>::Instance().AddTrackbar("Min armor weight / height ratio (0-10000 / 1e4)",
                                                    trackbar_window_name_,
                                                    parameter_maintain_.min_armor_wh_ratio_,
                                                    kMaxRatioTrackbar * 1e-3);

    /// --- max R w:h ---
    debug::Trackbar<double>::Instance().AddTrackbar("Max armor weight / height ratio (0-10000 / 1e4)",
                                                    trackbar_window_name_,
                                                    parameter_maintain_.max_armor_wh_ratio_,
                                                    kMaxRatioTrackbar * 1e-3);

    /// --- min R area ---
    debug::Trackbar<int>::Instance().AddTrackbar("Min R area (0-2000)",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.min_R_area_,
                                                 kMaxRAreaTrackbar);

    /// --- max R area ---
    debug::Trackbar<int>::Instance().AddTrackbar("Max R area (0-2000)",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.max_R_area_,
                                                 kMaxRAreaTrackbar);

    /// --- max R area ---
    debug::Trackbar<int>::Instance().AddTrackbar("R WH Deviation (0 - 10)",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.max_R_wh_deviation_,
                                                 kMaxRAreaTrackbar / 200);

    /// --- delta_u_ ---
    debug::Trackbar<int>::Instance().AddTrackbar("delta_u (0-100): ",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.delta_u_,
                                                 kMaxCompensation);

    /// --- delta_v_ ---
    debug::Trackbar<int>::Instance().AddTrackbar("delta_v (0-100): ",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.delta_v_,
                                                 kMaxCompensation);
}



