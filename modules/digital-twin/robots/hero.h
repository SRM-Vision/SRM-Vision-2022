/**
 * Hero robot header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef HERO_H_
#define HERO_H_

#include "../robot.h"

class Hero final : public Robot {
public:
    Hero(Colors color, double health) :
            Robot(color, health, kHero) {}
};

#endif  // HERO_H_
