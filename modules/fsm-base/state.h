#ifndef STATE_H_
#define STATE_H_

#include <functional>
#include "fsm_base_types.h"

namespace fsm {
    class StateMachine;

    typedef std::function<void(MachineBase &, const StateSharedPtr &)> StateEvent;

    class State : public std::enable_shared_from_this<State> {
        friend class StateMachine;

        friend class Transition;

    public:
        ATTR_READER_REF(name_, name)

        ATTR_READER(timeout_, timeout)

        [[maybe_unused]] static StateSharedPtr MakeState(StateMachine &owner, const char *name, time_t timeout = 0);

        [[maybe_unused]] static StateSharedPtr MakeState(StateMachine &owner, const State &copy);

        State &operator=(const State &) = delete;

        virtual ~State() = default;

        [[maybe_unused]] inline void SetTimeout(time_t timeout) { timeout_ = timeout; }

        [[maybe_unused]] void ClearActions();

        inline bool operator==(const State &state) const { return this == &state; }

        inline bool operator!=(const State &state) const { return this != &state; }

        StateEvent OnEnter;
        StateEvent OnExit;

    private:
        State(StateMachine &owner, const char *name, time_t timeout = 0);

        State(StateMachine &owner, const State &state);

        std::string name_;
        time_t timeout_;

        std::vector<TransitionSharedPtr> transitions_;
    };
}

#endif  // STATE_H_
