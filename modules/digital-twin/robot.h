/**
 * Robot definition header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "components/armor.h"
#include "unit.h"

class Robot : public Unit {
public:
    enum RobotTypes {
        kSentry = 0,
        kHero = 1,
        kEngineer = 2,
        kInfantry3 = 3,
        kInfantry4 = 4,
        kInfantry5 = 5,
        kAerial = 6,
        SIZE [[maybe_unused]] = 7
    };

    ATTR_READER(type_, Type)

    ATTR_READER_REF(armors_, Armors)

    Robot(Colors color,
          double health,
          RobotTypes type) :
            Unit(color, health),
            type_(type) {}

    inline void AddArmor(const Armor &armor) { armors_.push_back(armor); }

protected:
    RobotTypes type_;

    std::vector<Armor> armors_;
};

#endif  // ROBOT_H_
