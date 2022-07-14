/**
 * Outpost predictor class header.
 * \author Lzy20020320
 * \date 2022.7.13
 */


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

private:
    /**
     * \brief Find the armor with the biggest area.
     * @param [in] armors. All detected armors.
     * @return index of the armor with biggest area.
     */
    int FindBiggestArmor(const std::vector<Armor> &armors);

    /**
     * \Brief Decide the coming/going armor in different rotating cases.
     * @Details In one or two armors cases, compare 'armor center x' with 'outpost center x' to decide coming/going.
     */
    void DecideComingGoing();

    /**
     * \Brief Judge rotate direction.
     * @Details Calculate difference value of contiguous armor centers' x.
     * @Note Variable 'clockwise' is ought to be valued as 1 (rotate left) or -1 (rotate right).
     */
    void IsClockwise();

    Outpost outpost_;
    SpinDetector spin_detector_{0.05, 1.2,0.125,SpinDetector::kSpherical};

    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point ready_time_{};

    Entity::Colors enemy_color_;
    bool checked_clockwise_ = false;
    int clockwise_          = 0;       ///< 向右转逆，向左转顺

    cv::Point2f roi_corners_[4];
    double last_armor_x_;

    bool ready_fire_ = false;
    bool prepared = false;
    bool need_init_ = true;

    double biggest_area_ = 0;
    double delay_time_   = 0.1;

    float armor_yaw_{};
    float armor_pitch_{};

    int buff;
    int roi_buff_;
};

