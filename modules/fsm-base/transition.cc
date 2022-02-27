#include <algorithm>
#include "state.h"
#include "transition.h"
#include "transition_predicate.h"
#include "machine.h"

namespace fsm {
    [[maybe_unused]] TransitionSharedPtr Transition::MakeTransition(const char *name,
                                                                    const StateSharedPtr &from,
                                                                    const StateSharedPtr &to,
                                                                    IPredicateSharedPtr predicate) {
        TransitionSharedPtr transition(new Transition(name, from, to, std::move(predicate)));
        if (transition)
            transition->Init();
        return transition;
    }

    [[maybe_unused]] TransitionSharedPtr Transition::MakeTransition(const char *name,
                                                                    const StateSharedPtr &from,
                                                                    IPredicateSharedPtr predicate) {
        TransitionSharedPtr transition(new Transition(name, from, std::move(predicate)));
        if (transition)
            transition->Init();
        return transition;
    }

    Transition::Transition(const char *name,
                           const StateSharedPtr &from,
                           const StateSharedPtr &to,
                           IPredicateSharedPtr pred)
            : transition_type_(TransitionType::kNormalTransition),
              name_(std::string(nullptr != name ? name : "")),
              predicate_(std::move(pred)),
              to_(to),
              hold_from_(from),
              hold_to_(to),
              is_valid_(true) {}

    Transition::Transition(const char *name,
                           const StateSharedPtr &to_from,
                           IPredicateSharedPtr pred)
            : transition_type_(TransitionType::kInternalTransition),
              name_(std::string(nullptr != name ? name : "")),
              predicate_(std::move(pred)),
              hold_from_(to_from),
              hold_to_(to_from),
              is_valid_(true) {}

    void Transition::Init() {
        StateSharedPtr hold_from_state = hold_from_.lock();
        if (hold_from_state) {
            auto &transitions = hold_from_state->transitions_;
            if (std::find(transitions.begin(),
                          transitions.end(),
                          shared_from_this()) == transitions.end()) {
                transitions.push_back(shared_from_this());
                to_ = hold_to_;
            }
        }
    }

    bool Transition::IsMatch(const EventSharedPtr &event,
                             const MachineBase &machine) {
        return predicate_ && (*predicate_)(event, machine);
    }

    [[maybe_unused]] void Transition::ClearActions() { OnTransition = nullptr; }

    [[maybe_unused]] NonTransitiveActionSharedPtr
    NonTransitiveAction::MakeNonTransitiveAction(const char *name,
                                                 ActionMachine &owner_machine,
                                                 IPredicateSharedPtr predicate) {
        NonTransitiveActionSharedPtr non_transition(new NonTransitiveAction(name, std::move(predicate)));
        if (non_transition)
            non_transition->Init(owner_machine);
        return non_transition;
    }

    NonTransitiveAction::NonTransitiveAction(const char *name,
                                             IPredicateSharedPtr predicate)
            : name_(std::string(nullptr != name ? name : "")),
              predicate_(std::move(predicate)) {}

    void NonTransitiveAction::Init(ActionMachine &owner_machine) {
        if (std::find(owner_machine.non_transitive_actions_.begin(),
                      owner_machine.non_transitive_actions_.end(),
                      shared_from_this()) == owner_machine.non_transitive_actions_.end())
            owner_machine.non_transitive_actions_.push_back(shared_from_this());
    }

    bool NonTransitiveAction::IsMatch(const EventSharedPtr &event,
                                      const MachineBase &machine) {
        return predicate_ && (*predicate_)(event, machine);
    }

    [[maybe_unused]] void NonTransitiveAction::ClearActions() { OnAction = nullptr; }

}
