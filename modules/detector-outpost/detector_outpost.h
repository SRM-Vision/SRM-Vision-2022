/**
 * Power rune detector class header.
 * \author LIYunzhe1408
 * \date 2022.5.2
 */
#ifndef DETECTOR_OUTPOST_H_
#define DETECTOR_OUTPOST_H_
#include "data-structure/buffer.h"
#include "digital-twin/facilities/outpost.h"
#include "../digital-twin/battlefield.h"
#include "debug-tools/painter.h"

#include <utility>
#include <queue>

struct DetectedData
{
    std::vector<Armor> out_post_armors{};

    cv::Point2f outpost_center{};
    coordinate::TranslationVector outpost_center_3d{};
    coordinate::TranslationVector shoot_point{};
    double center_distance{};
    cv::Point2f corners0;
    cv::Point2f corners1;
    cv::Point2f corners2;
    cv::Point2f corners3;

    bool spining = false;
    bool perpared = false;

    int going_armor = -1;
    int coming_armor = -1;

    int is_clockwise{};                   // 1 is clockwise, -1 is anti-clockwise

};
class OutpostDataDetector : NO_COPY, NO_MOVE
{
public:
    OutpostDataDetector();
    void SetColor(const Entity::Colors& outpost_color)  { color_ = outpost_color;   };
    bool Initialize(const std::string &config_path);
    DetectedData Run(const Battlefield& battlefield);
    ~OutpostDataDetector() = default;

    ATTR_READER_REF(outpost_center_, OutpostCenter)

    ATTR_READER_REF(going_armor_, GoingArmor)

    ATTR_READER_REF(coming_armor_, ComingArmor)

    ATTR_READER_REF(center_distance_, OutpostCenterDistance)

    ATTR_READER_REF(clockwise_, Clockwise)

    ATTR_READER_REF(spining_, Spining)

private:
    bool is_checked_clockwise = false;

    static constexpr double max_jump_yaw_{0.05};
    static constexpr double max_jump_period_1{0.8};
    static constexpr double max_jump_period_2{1.2};

    void Clear();
    void IsClockwise();
    void FindBiggestArmor();
    void DecideComingGoing();
    void IsSpining(Armor armor, const uint64_t& now_timestamp);

    Entity::Colors color_;

    double jump_period_{0}; // yaw jump period
    int jump_count_{0};
    double last_yaw_{0};
    double last_yaw_jump_delta_{0};


    // Past status data
    double last_armor_x_;
    double max_area_buff;
    double max_area_;


    uint64_t last_jump_time_{0};
    coordinate::TranslationVector last_jump_position_{0,0,0};
    int disappear_buff_;


    // Send to predictor
    std::vector<Armor> detected_armors_in_this_frame_{};

    cv::Point2f outpost_center_;                            // 在图片中的
    coordinate::TranslationVector center_3D;                // 世界坐标系下
    coordinate::TranslationVector shoot_point_;             // 相机坐标下
    cv::Point2f outpost_corner_[4]{};
    double center_distance_;

    bool spining_ = false;
    bool prepared_ = false;

    int going_armor_ = -1;
    int coming_armor_ = -1;
    int clockwise_ = 0;                   // 1 is clockwise, -1 is anti-clockwise

};

#endif //DETECTOR_OUTPOST_H_
