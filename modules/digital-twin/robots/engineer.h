/**
 * Engineer robot header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef ENGINEER_H_
#define ENGINEER_H_

#include "../robot.h"

class Engineer : public Robot {
public:
    Engineer(Colors color, double health) :
            Robot(color, health, kEngineer) {}
};

#endif  // ENGINEER_H_
