/**
 * Sentry robot header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef SENTRY_H_
#define SENTRY_H_

#include "../robot.h"

class Sentry : public Robot {
public:
    Sentry(Colors color, double health) :
            Robot(color, health, kSentry) {}
};

#endif  // SENTRY_H_
