/**
 * Power rune detector class header.
 * \author LemonadeJJ
 * \date 2022.1.16
 */

#ifndef DETECTOR_RUNE_H_
#define DETECTOR_RUNE_H_

#include <detector-rune-debug/detector_rune_debug.h>
#include "data-structure/frame.h"
#include "digital-twin/facilities/power_rune.h"
#include "debug-tools/painter.h"

class RuneDetector : NO_COPY, NO_MOVE {
public:
    [[maybe_unused]] explicit RuneDetector(Entity::Colors color = Entity::Colors::kBlue, bool debug = false);

    ~RuneDetector() = default;

    /**
     * \brief Run detector once.
     * \param [in] frame Input frame.
     * \return Output power rune data.
     */
    PowerRune Run(Frame &frame);

    /**
     * \brief Initialize detector.
     * \param [in] config_path Config file path.
     * \param [in] frame Input frame to decide image area.
     * \return Whether detector is ready.
     */
    bool Initialize(const std::string &config_path, const Frame &frame, bool debug_use_trackbar = false);

private:
    /// Split RGB channels of the frame
    void ImageSplit(cv::Mat &image);

    /// Frame morphological operation
    static void ImageMorphologyEx(cv::Mat &image);

    /// Decide rotation Direction
    void FindRotateDirection();

    /// Find energy mechanism center
    bool FindCenterR(cv::Mat &image);

    /// Find armour center
    bool FindArmorCenterP(cv::Mat &image);

    /// Find fan barycenter
    void FindFanCenterG();

    bool found_energy_center_r;  ///< Whether rune center R is found.
    bool found_armor_center_p;   ///< Whether armor center P is found.
    bool found_fan_center_g;     ///< Whether fan center G is found.
    bool debug_;                 ///< Debug flag.

    cv::Mat image_;              ///< Initial input image.
    cv::Point2f fan_rect_points_[4];
    std::vector<cv::Mat> image_channels_;
    std::vector<std::vector<cv::Point>> fan_contours_;
    std::vector<cv::Vec4i> fan_hierarchies_;
    cv::RotatedRect fan_encircle_rect_;
    cv::RotatedRect armor_encircle_rect_;
    const cv::Point2f offset_center_r_;

    cv::Point3f send_yaw_pitch_delay_;
    cv::Point2f energy_center_r_;
    cv::Point2f armor_center_p_;
    cv::Point2f fan_center_g_;
    Entity::Colors color_;
    cv::Point2f rtp_vec_;
    cv::Point2f rtg_vec_;
    int clockwise_;
};

#endif  // DETECTOR_RUNE_H_
