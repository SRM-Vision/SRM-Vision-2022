#include "state.h"
#include "machine.h"

namespace fsm {
    [[maybe_unused]] StateSharedPtr State::MakeState(StateMachine &owner, const char *name, time_t timeout) {
        StateSharedPtr state(new State(owner, name, timeout));
        if (state)
            owner.states_.push_back(state);
        return state;
    }

    [[maybe_unused]] StateSharedPtr State::MakeState(StateMachine &owner, const State &copy) {
        StateSharedPtr state(new State(owner, copy));
        if (state)
            owner.states_.push_back(state);
        return state;
    }

    [[maybe_unused]] State::State(StateMachine &owner, const char *name, time_t timeout)
            : name_(std::string(nullptr != name ? name : "")), timeout_(timeout) {}

    State::State(StateMachine &owner, const State &state)
            : name_(state.name_), timeout_(state.timeout_) {}

    [[maybe_unused]] void State::ClearActions() {
        OnEnter = nullptr;
        OnExit = nullptr;
    }
}
