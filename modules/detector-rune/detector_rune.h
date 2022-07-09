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
    [[maybe_unused]] explicit RuneDetector(Entity::Colors color = Entity::Colors::kRed, bool debug = true);

    ~RuneDetector() = default;

    /**
     * \brief Run detector once.
     * \param [in] frame Input frame.
     * \return Output power rune data.
     */
    PowerRune Run(Entity::Colors color, Frame &frame);

    /**
     * \brief Initialize detector.
     * \param [in] config_path Config file path.
     * \param [in] frame Input frame to decide image area.
     * \return Whether detector is ready.
     */
    bool Initialize(const std::string &config_path, const Frame &frame, bool debug_use_trackbar = false);

private:
    /**
     * \brief Filtrate object by HSV
     * \param [in] image Incoming image.
     */
    void ImageSplit(cv::Mat &image);

    /**
     * \brief Morphological operation
     * \param [in] image Incoming image.
     */
    static void ImageMorphologyEx(cv::Mat &image);

    /**
     * \brief Decide rotation Direction
     */
    void FindRotateDirection();

    /**
     * \brief Find energy mechanism center
     * \param [in] image Incoming image
     */
    bool FindCenterR(cv::Mat &image);

    /**
     * \brief Find armour center
     * \param [in] image Incoming image
     */
    bool FindArmorCenterP(cv::Mat &image);

    /**
     * \brief Find fan barycenter
     * \param [in] image Incoming image
     */
    void FindFanCenterG();

    bool found_energy_center_r;  ///< Whether rune center R is found.
    bool found_armor_center_p;   ///< Whether armor center P is found.
    bool found_fan_center_g;     ///< Whether fan center G is found.
    bool debug_;                 ///< Debug flag.

    cv::Mat image_;                   ///< Original image's duplicate.
    std::vector<cv::Mat> image_channels_;
    cv::Point2f fan_rect_points_[4];  ///< Four vertices of the enclosing rectangle of a fan

    std::vector<std::vector<cv::Point>> fan_contours_;

    std::vector<cv::Vec4i> fan_hierarchies_;   ///< Outline of the hierarchy
    cv::RotatedRect fan_encircle_rect_;        ///< Enclosing rectangle of a fan
    cv::RotatedRect armor_encircle_rect_;      ///< Enclosing rectangle of a fan's armor

    cv::Point3f send_yaw_pitch_delay_;
    cv::Point2f energy_center_r_;
    cv::Point2f armor_center_p_;
    cv::Point2f r_offset_;    ///< Compensate energy_center_r by last frame's point when lost frame.
    cv::Point2f p_offset_;    ///< Compensate armor_center_p by last frame's point when lost frame.
    cv::Point2f fan_center_g_;
    Entity::Colors color_;
    cv::Point2f rtp_vec_;    ///< energy_center_r to armor_center_p, for calculate predicted point
    cv::Point2f rtg_vec_;    ///< energy_center_r to fan_center_g, for calculate angle and palstance
    int clockwise_;          ///< Rotation direction, -1 for anti-clockwise , 1 for clockwise
    int frame_lost_;         ///< Lost frame when detecting
};

#endif  // DETECTOR_RUNE_H_
