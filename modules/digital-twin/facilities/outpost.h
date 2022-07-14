/**
 * Outpost definition header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef OUTPOST_H_
#define OUTPOST_H_

#include <data-structure/communication.h>
#include "../facility.h"

class Outpost : public Facility {
public:
    Outpost(Colors color=Entity::kBlue,
            double health=500) :
            Facility(color, health, kOutpost) {}


    cv::Point2f center_point_;
    coordinate::TranslationVector center_point_camera_;
    cv::Point2f coming_center_;
    cv::Point2f going_center_;
    double distance;
    float shoot_delay;
    int clockwise = 0; ///< 向右转逆，向左转顺

};

#endif  // OUTPOST_H_
