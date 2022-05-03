/**
 * Outpost definition header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef OUTPOST_H_
#define OUTPOST_H_

#include "../facility.h"

class Outpost : public Facility {
public:
    Outpost(Colors color=Entity::kBlue,
            double health=500) :
            Facility(color, health, kOutpost) {}
    cv::Point2f center_point_;
    cv::Point2f coming_center_;
    double distance;
    float shoot_delay;
    int clockwise = 0; // clockwise is 1, anti-clockwise = -1

//    static Outpost last_outpost_dataflow_[100];
};

#endif  // OUTPOST_H_
