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
    Outpost(Colors color,
            double health) :
            Facility(color, health, kOutpost) {}

    cv::Point2f center_point;
    float shoot_delay;
};

#endif  // OUTPOST_H_
