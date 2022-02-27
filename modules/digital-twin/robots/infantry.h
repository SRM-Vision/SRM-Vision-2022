/**
 * Infantry robot header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef INFANTRY_H_
#define INFANTRY_H_

#include "../robot.h"

class Infantry : public Robot {
public:
    Infantry(Colors color,
             double health,
             RobotTypes type) :
            Robot(color, health, type) {
        assert(3 <= type && type <= 5);
    }
};

#endif  // INFANTRY_H_
