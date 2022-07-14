/**
 * Power rune detector class header.
 * \author LemonadeJJ
 * \date 2022.1.16
 */

#ifndef DETECTOR_RUNE_H_
#define DETECTOR_RUNE_H_

#include "data-structure/frame.h"
#include "digital-twin/facilities/power_rune.h"
#include "debug-tools/painter.h"
#include "detector_rune_debug.h"

class RuneDetector : NO_COPY, NO_MOVE {
public:
    [[maybe_unused]] explicit RuneDetector(Entity::Colors color = Entity::Colors::kRed, bool debug = true);

    ~RuneDetector() = default;

    /**
     * \brief Run detector once.
     * \param [in] frame Input original frame.
     * \return Output power rune data.
     */
    PowerRune Run(Entity::Colors color, Frame &frame);

    /**
     * \brief Initialize detector.
     * \param [in] config_path Config file path.
     * \return Whether detector is ready.
     */
    bool Initialize(const std::string &config_path);

private:
    /**
     * \brief Image preprocess.
     */
    void PreProcess();

    /**
     * \brief Decide rotating direction.
     */
    void FindRotateDirection();

    /**
     * \brief Find energy mechanism center R.
     */
    bool FindCenterR();

    /**
     * \brief Find armour center P.
     */
    bool FindArmorCenterP();

    /**
     * \brief Find fan barycenter G.
     */
    void FindFanCenterG();

    cv::Mat image_;  ///< Original image's duplicate.
    cv::Point2i ROI_tl_point_;  ///< Top and left corner of ROI rect.
    bool debug_;  ///< Debug flag.
    Entity::Colors color_;
    std::vector<cv::Mat> image_channels_;
    std::vector<std::vector<cv::Point>> fan_contours_;
    std::vector<cv::Vec4i> fan_hierarchies_;  ///< Outline's hierarchy.

    int clockwise_;  ///< Rotation direction, -1 for anti-clockwise , 1 for clockwise.
    int frame_lost_;  ///< Lost frame when detecting.
    float rune_radius_;
    cv::RotatedRect fan_encircle_rect_;  ///< Enclosing rectangle of a fan.
    cv::RotatedRect armor_encircle_rect_;  ///< Enclosing rectangle of a fan's armor.
    bool found_energy_center_r;  ///< Whether rune center R is found.
    bool found_armor_center_p;  ///< Whether armor center P is found.
    cv::Point2f energy_center_r_;
    cv::Point2f armor_center_p_;
    cv::Point2f last_energy_center_r;
    cv::Point2f fan_center_g_;
    cv::Point2f r_offset_;  ///< Compensate energy_center_r by last frame's point when lost frame.
    cv::Point2f p_offset_;  ///< Compensate armor_center_p by last frame's point when lost frame.
    cv::Point2f rtp_vec_;  ///< Energy_center_r to armor_center_p, for calculating predicted point.
    cv::Point2f rtg_vec_;  ///< Energy_center_r to fan_center_g, for calculating angle and palstance.

    cv::Point3f send_yaw_pitch_delay_;

    const float kMaxDeviation = 10;
    const int kMaxFrameLost = 5;
    const float kMaxRatio = 0.1;
};

#endif  // DETECTOR_RUNE_H_
