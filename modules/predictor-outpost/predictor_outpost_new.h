//
// Created by lzy on 2022/7/13.
//

#ifndef PREDICTOR_OUTPOST_NEW_H_
#define PREDICTOR_OUTPOST_NEW_H_

#endif //PREDICTOR_OUTPOST_NEW_H_

#include <data-structure/communication.h>
#include "../digital-twin/battlefield.h"
#include "debug-tools/painter.h"
#include "predictor-armor/spin_detector.h"
#include "predictor-armor-debug/predictor_armor_debug.h"

class OutpostPredictorNew{
public:
    OutpostPredictorNew(){
//        debug::Trackbar<double>::Instance().AddTrackbar("delay_time:",
//                                                        "outpost",
//                                                        delay_time_,
//                                                        1);
    };
    ~OutpostPredictorNew() = default;
    SendPacket Run(Battlefield battlefield, float bullet_speed = 16);
    void SetColor(const Entity::Colors& enemy_color)    {   enemy_color_ = enemy_color;}
    void GetROI(cv::Rect &roi_rect, const cv::Mat &src_image);

private:
    int FindBiggestArmor(const std::vector<Armor> &armors);
    void DecideComingGoing();
    void IsClockwise();
    void UpdateROICorners(const Armor& armor);

    Outpost outpost_;
    SpinDetector spin_detector_{0.05, 1.2,0.125,SpinDetector::kSpherical};

    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point ready_time_{};

    Entity::Colors enemy_color_;
    bool checked_clockwise_;
    int clockwise_;

    cv::Point2f roi_corners_[4];
    double last_armor_x_;

    bool ready_fire_ = false;
    bool prepared = false;
    bool need_init_ = true;

    double biggest_area_;
    double delay_time_=0.1;

    float armor_yaw_{};
    float armor_pitch_{};

    int buff;
    int roi_buff_;
};

