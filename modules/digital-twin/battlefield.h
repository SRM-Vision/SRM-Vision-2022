/**
 * Digital twin main header.
 * \author trantuan-20048607, screw-44
 * \date 2022.1.28
 * \details Include this file in digital-twin module for complete function.
 */

#ifndef BATTLEFIELD_H_
#define BATTLEFIELD_H_

#include "robots/engineer.h"
#include "robots/hero.h"
#include "robots/infantry.h"
#include "robots/sentry.h"
#include "robots/aerial.h"
#include "facilities/base.h"
#include "facilities/outpost.h"
#include "facilities/power_rune.h"
#include "facilities/radar_station.h"

/// \brief Battlefield simulation data model.
class Battlefield {
public:
    ATTR_READER(time_stamp_, TimeStamp)

    ATTR_READER_REF(yaw_pitch_roll_, YawPitchRoll)

    ATTR_READER(bullet_speed_, BulletSpeed)

    ATTR_READER_REF(robots_, Robots)

    ATTR_READER_REF(facilities_, Facilities)

    Battlefield() :
            time_stamp_(0),
            bullet_speed_(15),
            yaw_pitch_roll_{0,0,0},
            robots_(),
            facilities_() {}

    /**
     * \brief Constructor with complete battlefield information.
     * \param time_stamp Time stamp from its source for tracking.
     * \param armors Armor sources.
     */
    Battlefield(uint64_t time_stamp,
                float bullet_speed,
                const std::array<float,3> yaw_pitch_roll,
                const std::vector<Armor> &armors);

private:
    uint64_t time_stamp_;  ///< Time stamp from its source for tracking.
    float bullet_speed_;
    std::array<float,3> yaw_pitch_roll_;  ///< Gyroscope attitude Euler angle
    std::unordered_map<Entity::Colors,
            std::unordered_map<Robot::RobotTypes,
                    std::shared_ptr<Robot>>> robots_;  ///< Robot data, stored in color and type order.
    std::unordered_map<Entity::Colors,
            std::unordered_map<Facility::FacilityTypes,
                    std::shared_ptr<Facility>>> facilities_;  ///< Facility data, stored in color and type order.
};

#endif  // BATTLEFIELD_H_
