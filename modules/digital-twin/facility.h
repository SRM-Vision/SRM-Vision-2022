/**
 * Facility definition header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef FACILITY_H_
#define FACILITY_H_

#include "components/armor.h"
#include "unit.h"

class Facility : public Unit {
public:
    enum FacilityTypes {
        kNone = -1,
        kBase = 0,
        kOutpost = 1,
        kRadarStation = 2,
        kPowerRune = 3,
        SIZE [[maybe_unused]] = 4
    };

    ATTR_READER(type_, Type)

    ATTR_READER_REF(bottom_armors_, BottomArmors)

    ATTR_READER_REF(top_armors_, TopArmors)

    Facility(Colors color,
             double health,
             FacilityTypes type) :
            Unit(color, health),
            type_(type) {}

    Facility() : Unit(), type_(kNone) {}

    inline void AddBottomArmor(const Armor &armor) { bottom_armors_.push_back(armor); }

    inline void AddTopArmor(const Armor &armor) { top_armors_.push_back(armor); }

protected:
    FacilityTypes type_;

    std::vector<Armor> bottom_armors_;
    std::vector<Armor> top_armors_;
};

#endif  // FACILITY_H_
