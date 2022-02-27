/**
 * Unit definition header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef UNIT_H_
#define UNIT_H_

#include "entity.h"

class Unit : public Entity {
public:
    ATTR_READER(health_, Health)

    Unit(Colors color,
         double health) :
            Entity(color),
            health_(health) {}

    Unit() : Entity(), health_(0) {}

protected:
    double health_;
};

#endif  // UNIT_H_
