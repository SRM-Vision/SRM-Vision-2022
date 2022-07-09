#include "detector_rune_debug.h"

void RuneDetectorDebug::Initialize(const std::string &config_path, bool debug_use_trackbar) {
    config_path_ = config_path;
    // Open config file.
    try {
        config_.open(config_path_, cv::FileStorage::READ);
    } catch (const std::exception &) {
        LOG(ERROR) << "Failed to open rune detector config file" << config_path_ << ".";
    }
    // Read config data.
    try {
        config_["MIN_CONTOUR_AREA"] >> min_contour_area_;
        config_["MAX_CONTOUR_AREA"] >> max_contour_area_;
        config_["MIN_ARMOR_AREA"] >> min_armor_area_;
        config_["MAX_ARMOR_AREA"] >> max_armor_area_;
        config_["MIN_BOUNDING_BOX_AREA"] >> min_bounding_box_area_;
        config_["MIN_R_BOUNDING_BOX_AREA"] >> min_r_bounding_box_area_;
        config_["MAX_R_BOUNDING_BOX_AREA"] >> max_r_bounding_box_area_;
        config_["SPLIT_GRAY_THRESH"] >> split_gray_thresh_;
        config_["MIN_ARMOR_WH_RATIO"] >> min_armor_wh_ratio_;
        config_["MAX_ARMOR_WH_RATIO"] >> max_armor_wh_ratio_;
        config_["MIN_BOUNDING_BOX_WH_RATIO"] >> min_bounding_box_wh_ratio_;
        config_["MAX_BOUNDING_BOX_WH_RATIO"] >> max_bounding_box_wh_ratio_;
        config_["MAX_ENCIRCLE_R_RECT_WH_DEVIATION"] >> max_encircle_r_rect_wh_deviation_;
        config_["KERNEL_SIZE"] >> kernel_size_;
        config_["DELTA_U"] >> delta_u_;
        config_["DELTA_V"] >> delta_v_;
    } catch (std::exception &) {
        LOG(ERROR) << "Failed to load config of rune detector.";
    }
    config_.release();

    if (debug_use_trackbar) {
        // --- binary threshold ---
        debug::Trackbar<int>::Instance().AddTrackbar("Binary threshold (0-255):",
                                                     trackbar_window_name_,
                                                     split_gray_thresh_,
                                                     kMaxThresholdTrackbar);

        // --- kernel size ---
        debug::Trackbar<int>::Instance().AddTrackbar("kernel size(0 - 5):",
                                                     trackbar_window_name_,
                                                     kernel_size_,
                                                     kMaxRatioTrackbar / 200);

        // --- min bounding box area ---
        debug::Trackbar<int>::Instance().AddTrackbar("Min Bounding box (0-10000):",
                                                     trackbar_window_name_,
                                                     min_bounding_box_area_,
                                                     kMaxRuneAreaTrackbar);
        // --- min contour area ---
        debug::Trackbar<int>::Instance().AddTrackbar("Min contour area (0-10000):",
                                                     trackbar_window_name_,
                                                     min_contour_area_,
                                                     kMaxRuneAreaTrackbar);

        // --- max contour area ---
        debug::Trackbar<int>::Instance().AddTrackbar("Max contour area (0-10000)",
                                                     trackbar_window_name_,
                                                     max_contour_area_,
                                                     kMaxRuneAreaTrackbar);

        // --- min armor area ---
        debug::Trackbar<int>::Instance().AddTrackbar("Min armor area (0-10000)",
                                                     trackbar_window_name_,
                                                     min_armor_area_,
                                                     kMaxRuneAreaTrackbar);

        // --- max armor area ---
        debug::Trackbar<int>::Instance().AddTrackbar("Max armor area (0-10000)",
                                                     trackbar_window_name_,
                                                     max_armor_area_,
                                                     kMaxRuneAreaTrackbar);

        // --- min R area ---
        debug::Trackbar<int>::Instance().AddTrackbar("Min R area (0-2000)",
                                                     trackbar_window_name_,
                                                     min_r_bounding_box_area_,
                                                     kMaxRAreaTrackbar);

        // --- max R area ---
        debug::Trackbar<int>::Instance().AddTrackbar("Max R area (0-2000)",
                                                     trackbar_window_name_,
                                                     max_r_bounding_box_area_,
                                                     kMaxRAreaTrackbar);

        // --- min armor w:h ---
        debug::Trackbar<double>::Instance().AddTrackbar("Min armor weight / height ratio (0-10000)",
                                                        trackbar_window_name_,
                                                        min_armor_wh_ratio_,
                                                        kMaxRatioTrackbar * 1e-3);

        // --- max armor w:h ---
        debug::Trackbar<double>::Instance().AddTrackbar("Max armor weight / height ratio (0-10000)",
                                                        trackbar_window_name_,
                                                        max_armor_wh_ratio_,
                                                        kMaxRatioTrackbar * 1e-3);

        // --- min R w:h ---
        debug::Trackbar<double>::Instance().AddTrackbar("Min R weight / height ratio (0-10000)",
                                                        trackbar_window_name_,
                                                        min_bounding_box_wh_ratio_,
                                                        kMaxRatioTrackbar * 1e-3);

        // --- max R w:h ---
        debug::Trackbar<double>::Instance().AddTrackbar("Max R weight / height ratio (0-10000)",
                                                        trackbar_window_name_,
                                                        max_bounding_box_wh_ratio_,
                                                        kMaxRatioTrackbar * 1e-3);

        // --- delta_u_ ---
        debug::Trackbar<int>::Instance().AddTrackbar("delta_u (0-100): ",
                                                     trackbar_window_name_,
                                                     delta_u_,
                                                     kMaxCompensation);

        // --- delta_v_ ---
        debug::Trackbar<int>::Instance().AddTrackbar("delta_v (0-100): ",
                                                     trackbar_window_name_,
                                                     delta_v_,
                                                     kMaxCompensation);
    }
}

void RuneDetectorDebug::Save() {
    // Open config file.
    try {
        config_.open(config_path_, cv::FileStorage::WRITE);
    }
    catch (const std::exception &) {
        LOG(ERROR) << "Failed to open rune detector config file " << config_path_ << ".";
        return;
    }

    // Write config data.
    try {
        config_ << "MIN_CONTOUR_AREA" << min_contour_area_;
        config_ << "MAX_CONTOUR_AREA" << max_contour_area_;
        config_ << "MIN_ARMOR_AREA" << min_armor_area_;
        config_ << "MAX_ARMOR_AREA" << max_armor_area_;
        config_ << "MIN_R_BOUNDING_BOX_AREA" << min_r_bounding_box_area_;
        config_ << "MAX_R_BOUNDING_BOX_AREA" << max_r_bounding_box_area_;
        config_ << "SPLIT_GRAY_THRESH" << split_gray_thresh_;
        config_ << "MIN_ARMOR_WH_RATIO" << min_armor_wh_ratio_;
        config_ << "MAX_ARMOR_WH_RATIO" << max_armor_wh_ratio_;
        config_ << "MIN_BOUNDING_BOX_AREA" << min_bounding_box_area_;
        config_ << "MIN_BOUNDING_BOX_WH_RATIO" << min_bounding_box_wh_ratio_;
        config_ << "MAX_BOUNDING_BOX_WH_RATIO" << max_bounding_box_wh_ratio_;
        config_ << "MAX_ENCIRCLE_R_RECT_WH_DEVIATION" << max_encircle_r_rect_wh_deviation_;
        config_ << "KERNEL_SIZE" << kernel_size_;
        config_ << "DELTA_U" << delta_u_;
        config_ << "DELTA_V" << delta_v_;
    } catch (std::exception &) {
        LOG(ERROR) << "Failed to update config of rune detector.";
        return;
    }
    config_.release();
    LOG(INFO) << "Config of rune detector is updated.";
}
