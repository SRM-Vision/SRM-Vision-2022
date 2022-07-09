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

#include <utility>

struct SendToOutpostPredictor
{
    SendToOutpostPredictor():going_center_point2d(0,0), outpost_center(0,0), coming_center_point2d(0,0),
                             going_center_point3d(0,0,0), outpost_center_3d(0,0,0),coming_center_point3d(0,0,0),
                             is_clockwise(0), center_distance(0), bullet_speed(0){}
    void UpdateInfo(cv::Point2f going_center, cv::Point2f center,cv::Point2f coming_center,
                    coordinate::TranslationVector going_center_3d,coordinate::TranslationVector center_3d, coordinate::TranslationVector coming_center_3d,
                    int clockwise, double distance, float bulletspeed, coordinate::TranslationVector shootpoint);
    cv::Point2f going_center_point2d;
    coordinate::TranslationVector going_center_point3d;
    cv::Point2f outpost_center;
    coordinate::TranslationVector outpost_center_3d;
    cv::Point2f coming_center_point2d;
    coordinate::TranslationVector coming_center_point3d;
    coordinate::TranslationVector shoot_point;
    int is_clockwise;                   // 1 is clockwise, -1 is anti-clockwise
    double center_distance;
    float bullet_speed;
};
class OutpostDetector : NO_COPY, NO_MOVE
{
public:
    OutpostDetector();
    void SetColor(const Entity::Colors& outpost_color){color_ = outpost_color;};
    bool Initialize(const std::string &config_path);
    SendToOutpostPredictor Run(const Battlefield& battlefield);
    ~OutpostDetector() = default;

    ATTR_READER_REF(outpost_center_, OutpostCenter)

    ATTR_READER_REF(going_center_point_2D, GoingArmorCenter2D)

    ATTR_READER_REF(going_center_point_3D, GoingArmorCenter3D)

    ATTR_READER_REF(coming_center_point_2D, ComingArmorCenter2D)

    ATTR_READER_REF(coming_center_point_3D, ComingArmorCenter3D)

    ATTR_READER_REF(center_distance_, OutpostCenterDistance)

    ATTR_READER_REF(clockwise_, Clockwise)

private:
    const double kVertical_threshold_ = 3;
    bool is_checked_clockwise = false;
    bool outdated_ = true;
    std::chrono::high_resolution_clock::time_point start_time_;

    std::vector<Armor> detected_armors_in_this_frame_;
    void Clear();
    void IsClockwise();
    void FindBiggestArmor();
    void DecideComingGoing();
    Outpost outpost_;
    Entity::Colors color_;

    // Past status data
    double last_armor_x_;
    double max_area_;

    // Send to predictor
    cv::Point2f outpost_center_;                            // 在图片中的
    coordinate::TranslationVector shoot_point_;             // 相机坐标下
    coordinate::TranslationVector center_3D;                // 世界坐标系下
    cv::Point2f going_center_point_2D;
    coordinate::TranslationVector going_center_point_3D;    // 世界坐标系下
    cv::Point2f coming_center_point_2D;
    coordinate::TranslationVector coming_center_point_3D;   // 世界坐标系下
    int disappear_buff;
    int clockwise_ = 0;                   // 1 is clockwise, -1 is anti-clockwise
    double center_distance_;


};

#endif //DETECTOR_OUTPOST_H_
