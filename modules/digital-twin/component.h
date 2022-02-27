/**
 * Component definition header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef COMPONENT_H_
#define COMPONENT_H_

#include "entity.h"

class Component : public Entity {
public:
    enum ComponentType {
        kArmor = 0,
        SIZE [[maybe_unused]] = 1
    };

    ATTR_READER(type_, Type)

    Component(Colors color,
              ComponentType type) :
            Entity(color),
            type_(type) {}

protected:
    ComponentType type_;
};

#endif  // COMPONENT_H_
