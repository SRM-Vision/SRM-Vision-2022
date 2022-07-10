#include "battlefield.h"

#define ARMOR_CONFIDENCE_HIGH_THRESH 0.3
#define ARMOR_CONFIDENCE_LOW_THRESH 0.3
#define ARMOR_SQUARED_CENTER_DISTANCE_THRESH 144
#define ARMOR_SQUARED_TRANSLATION_VECTOR_WORLD_DISTANCE_THRESH 0.125

#define ADD_ARMOR_TO_ROBOT(_type, _class)                                                          \
    if (robots_[armor.Color()][_type].get() == nullptr)                                            \
        robots_[armor.Color()][_type] = std::make_shared<_class>(armor.Color(), 0);                \
    if (armor.Confidence() > ARMOR_CONFIDENCE_HIGH_THRESH) {                                       \
        robots_[armor.Color()][_type].get()->AddArmor(armor);                                      \
    } else if (last_battlefield_data_.robots_[armor.Color()][_type].get() != nullptr               \
                && armor.Confidence() > ARMOR_CONFIDENCE_LOW_THRESH) {                             \
        for (auto &armor_last:                                                                     \
                last_battlefield_data_.robots_[armor.Color()][_type].get()->Armors()) {            \
            auto center_delta = armor.Center() - armor_last.Center();                              \
            auto tv_world_delta = armor.TranslationVectorWorld() -                                 \
                                  armor_last.TranslationVectorWorld();                             \
            if (center_delta.dot(center_delta) < ARMOR_SQUARED_CENTER_DISTANCE_THRESH &&           \
                tv_world_delta.norm() < ARMOR_SQUARED_TRANSLATION_VECTOR_WORLD_DISTANCE_THRESH) {  \
                robots_[armor.Color()][_type].get()->AddArmor(armor);                              \
            }                                                                                      \
        }                                                                                          \
    }                                                                                              \
    if (robots_[armor.Color()][_type].get()->Armors().empty())                                     \
        robots_[armor.Color()].erase(_type);                                                       \
    if (robots_[armor.Color()].empty())                                                            \
        robots_.erase(armor.Color());                                                              \
    break;

#define ADD_ARMOR_TO_INFANTRY(_type, _class)                                                       \
    if (robots_[armor.Color()][_type].get() == nullptr)                                            \
        robots_[armor.Color()][_type] = std::make_shared<_class>(armor.Color(), 0, _type);         \
    if (armor.Confidence() > ARMOR_CONFIDENCE_HIGH_THRESH) {                                       \
        robots_[armor.Color()][_type].get()->AddArmor(armor);                                      \
    } else if (last_battlefield_data_.robots_[armor.Color()][_type].get() != nullptr               \
                && armor.Confidence() > ARMOR_CONFIDENCE_LOW_THRESH) {                             \
        for (auto &armor_last:                                                                     \
                last_battlefield_data_.robots_[armor.Color()][_type].get()->Armors()) {            \
            auto center_delta = armor.Center() - armor_last.Center();                              \
            auto tv_world_delta = armor.TranslationVectorWorld() -                                 \
                                  armor_last.TranslationVectorWorld();                             \
            if (center_delta.dot(center_delta) < ARMOR_SQUARED_CENTER_DISTANCE_THRESH &&           \
                tv_world_delta.norm() < ARMOR_SQUARED_TRANSLATION_VECTOR_WORLD_DISTANCE_THRESH) {  \
                robots_[armor.Color()][_type].get()->AddArmor(armor);                              \
            }                                                                                      \
        }                                                                                          \
    }                                                                                              \
    if (robots_[armor.Color()][_type].get()->Armors().empty())                                     \
        robots_[armor.Color()].erase(_type);                                                       \
    if (robots_[armor.Color()].empty())                                                            \
        robots_.erase(armor.Color());                                                              \
    break;

// TODO! Add classification of bottom and top armors.
#define ADD_ARMOR_TO_FACILITY_WITH_HEALTH(_type, _class)                                                          \
    if (facilities_[armor.Color()][_type].get() == nullptr)                                                       \
        facilities_[armor.Color()][_type] = std::make_shared<_class>(armor.Color(), 0);                           \
    if (armor.Confidence() > ARMOR_CONFIDENCE_HIGH_THRESH) {                                                      \
        facilities_[armor.Color()][_type].get()->AddBottomArmor(armor);                                           \
    } else if (last_battlefield_data_.facilities_[armor.Color()][_type].get() != nullptr                          \
                && armor.Confidence() > ARMOR_CONFIDENCE_LOW_THRESH) {                                            \
        for (auto &armor_last: last_battlefield_data_.facilities_[armor.Color()][_type].get()->BottomArmors()) {  \
            auto center_delta = armor.Center() - armor_last.Center();                                             \
            auto tv_world_delta = armor.TranslationVectorWorld() -                                                \
                                  armor_last.TranslationVectorWorld();                                            \
            if (center_delta.dot(center_delta) < ARMOR_SQUARED_CENTER_DISTANCE_THRESH &&                          \
                tv_world_delta.norm() < ARMOR_SQUARED_TRANSLATION_VECTOR_WORLD_DISTANCE_THRESH) {                 \
                facilities_[armor.Color()][_type].get()->AddBottomArmor(armor);                                   \
            }                                                                                                     \
        }                                                                                                         \
    }                                                                                                             \
    break;

Battlefield::Battlefield(uint64_t time_stamp,
                         float bullet_speed,
                         const std::array<float, 3> yaw_pitch_roll,
                         const std::vector<Armor> &armors) :
        time_stamp_(time_stamp),
        bullet_speed_(bullet_speed),
        yaw_pitch_roll_{yaw_pitch_roll[0], yaw_pitch_roll[1], yaw_pitch_roll[2]},
        robots_(),
        facilities_() {

    /// Store last battlefield data for eliminating shakes.
    static Battlefield last_battlefield_data_;
    // static Outpost last_outpost_dataflow_[10];

    for (auto &armor: armors) {
        switch (armor.ID()) {
            case 0:
            ADD_ARMOR_TO_ROBOT(Robot::RobotTypes::kSentry, Sentry)
            case 1:
            ADD_ARMOR_TO_ROBOT(Robot::RobotTypes::kHero, Hero)
            case 2:
            ADD_ARMOR_TO_ROBOT(Robot::RobotTypes::kEngineer, Engineer)
            case 3:
            ADD_ARMOR_TO_INFANTRY(Robot::RobotTypes::kInfantry3, Infantry)
            case 4:
            ADD_ARMOR_TO_INFANTRY(Robot::RobotTypes::kInfantry4, Infantry)
            case 5:
            ADD_ARMOR_TO_INFANTRY(Robot::RobotTypes::kInfantry5, Infantry)
            case 6:
            ADD_ARMOR_TO_FACILITY_WITH_HEALTH(Facility::FacilityTypes::kBase, Base);
            default:
            ADD_ARMOR_TO_FACILITY_WITH_HEALTH(Facility::FacilityTypes::kOutpost, Outpost);
        }
    }

    // Copy self to last_battlefield_data_, previous data will be automatically deleted.
    last_battlefield_data_ = SELF;
}
