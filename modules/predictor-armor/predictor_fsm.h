//
// Created by xiguang on 2022/3/11.
//

#ifndef PREDICTOR_FSM_H_
#define PREDICTOR_FSM_H_

#include "fsm-base/fsm_base_types.h"
#include "fsm-base/event.h"
#include "fsm-base/machine_operation_event.h"
#include "fsm-base/state.h"
#include "fsm-base/transition.h"
#include "fsm-base/transition_predicate.h"
#include "fsm-base/timeout_predicate.h"
#include "fsm-base/machine.h"
#include "fsm-base/machine_set.h"
#include "digital-twin/components/armor.h"
#include "digital-twin/entity.h"
#include "digital-twin/robot.h"

class ArmorPredictor;

/// divided by how many armor there is and whether opening antitop mode.
enum class ArmorEventType {
    kOne,
    kTwoWithoutAntiTop,
    kTwoWithAntiTop
};

class ArmorEvent : public fsm::EventTemplate<ArmorEventType> {
public:
    using underlying_type = std::underlying_type<ArmorEventType>::type;
    // underlying_type means the hidden type of enum type, such as int.

    ArmorEvent(ArmorEventType type, const fsm::MachineBaseSharedPtr &machine)
            : fsm::EventTemplate<ArmorEventType>(type, machine) {
    }

    std::ostream &ToStream(std::ostream &str) const override {
        return str << "ArmorEvent | " << static_cast<underlying_type>(type_);
    }

    [[nodiscard]] std::string ToString() const override {
        std::ostringstream os;
        os << "ArmorEvent | " << static_cast<underlying_type>(type_);
        std::string str = os.str();
        os.str("");
        return str;
    }
};

class ArmorMachine : public fsm::StateMachine {
public:
    friend class ArmorPredictor;

    friend class fsm::State;

    typedef std::unordered_map<Entity::Colors, std::unordered_map<Robot::RobotTypes, std::shared_ptr<Robot>>> RobotMap;

    struct StateBits {
        int target_selected;      ///< Where a target is selected.
        bool same_target;         ///< Target to shoot is same as the one on last time.
        bool same_id;             ///< Target is not the same but has same id.
        bool switch_armor;        ///< Switch target to lock to another.
        bool need_init;           ///< Missed conditions to keep locking previous target.
    };

    explicit ArmorMachine(const std::string &name);

    void Initialize() override;

    fsm::StateSharedPtr one_;
    fsm::StateSharedPtr two_without_antitop_;
    fsm::StateSharedPtr two_with_antitop_;

    fsm::TransitionSharedPtr one_two_without_antitop_;
    fsm::TransitionSharedPtr one_two_with_antitop_;
    fsm::TransitionSharedPtr two_without_antitop_one_;
    fsm::TransitionSharedPtr two_with_antitop_one_;
    fsm::TransitionSharedPtr two_without_antitop_two_with_antitop_;
    fsm::TransitionSharedPtr two_with_antitop_two_without_antitop_;
    fsm::TransitionSharedPtr one_one_;
    fsm::TransitionSharedPtr two_without_antitop_two_without_antitop_;
    fsm::TransitionSharedPtr two_with_antitop_two_with_antitop_;

private:
    std::shared_ptr<Armor> target_; ///< Target selected in this round.
    bool exist_enemy_, exist_grey_;
    bool is_transiting = true; /// if machine is transiting.

    const RobotMap *robots_;

};

#endif //PREDICTOR_FSM_H_
