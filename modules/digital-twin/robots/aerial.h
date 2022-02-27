/**
 * Aerial robot header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef AERIAL_H_
#define AERIAL_H_

#include "../robot.h"

class Aerial : public Robot {
public:
    Aerial(Colors color, double health) :
            Robot(color, health, kAerial) {}
};

#endif  // AERIAL_H_
