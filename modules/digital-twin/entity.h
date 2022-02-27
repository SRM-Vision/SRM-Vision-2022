/**
 * Entity definition header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef ENTITY_H_
#define ENTITY_H_

#include "lang-feature-extension/attr_reader.h"
#include "lang-feature-extension/self_tag.h"

class Entity {
public:
    enum Colors {
        kNone = -1,
        kBlue = 0,
        kRed = 1,
        kGrey = 2,
        kPurple = 3,
        SIZE [[maybe_unused]] = 4
    };

    ATTR_READER(color_, Color)

    explicit Entity(Colors color) :
            color_(color) {};

    Entity() : color_(kNone) {}

protected:
    Colors color_;
};

#endif  // ENTITY_H_
