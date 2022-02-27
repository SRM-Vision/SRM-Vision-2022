#ifndef FSM_BASE_TYPES_H_
#define FSM_BASE_TYPES_H_

#include <memory>
#include <vector>
#include "lang-feature-extension/attr_reader.h"
#include "lang-feature-extension/self_tag.h"
#include "lang-feature-extension/disable_constructor.h"

namespace fsm {
    class MachineBase;

    class ActionMachine;

    class MachineSet;

    class MachineSetHandler;

    class Event;

    class IPredicate;

    class State;

    class Transition;

    class NonTransitiveAction;

    typedef std::shared_ptr<MachineBase> MachineBaseSharedPtr;
    typedef std::weak_ptr<MachineBase> MachineBaseWeakPtr;
    typedef std::vector<MachineBaseWeakPtr> MachineBaseWeakPtrVec;

    typedef std::shared_ptr<MachineSet> MachineSetSharedPtr;
    typedef std::weak_ptr<MachineSet> MachineSetWeakPtr;

    typedef std::shared_ptr<MachineSetHandler> MachineSetHandlerSharedPtr;

    typedef std::shared_ptr<Event> EventSharedPtr;

    typedef std::shared_ptr<IPredicate> IPredicateSharedPtr;

    typedef std::shared_ptr<State> StateSharedPtr;
    typedef std::weak_ptr<State> StateWeakPtr;

    typedef std::shared_ptr<Transition> TransitionSharedPtr;
    typedef std::shared_ptr<NonTransitiveAction> NonTransitiveActionSharedPtr;

    [[maybe_unused]] typedef std::shared_ptr<ActionMachine> ActionMachineSharedPtr;
}

#endif  // FSM_BASE_TYPES_H_
