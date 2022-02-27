/**
 * Base definition header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef BASE_H_
#define BASE_H_

#include "../facility.h"

class Base : public Facility {
public:
    Base(Colors color,
         double health) :
            Facility(color, health, kBase) {}
};

#endif  // BASE_H_
