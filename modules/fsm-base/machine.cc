#include <glog/logging.h>
#include "state.h"
#include "event.h"
#include "transition.h"
#include "machine.h"
#include "machine_set.h"

uint64_t GetTime() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
}

namespace fsm {
    [[maybe_unused]] bool StateMachine::ForceState(const StateSharedPtr &state) {
        return InternalSetState<false>(state);
    }

    [[maybe_unused]] bool StateMachine::SetStartState(const StateSharedPtr &state) {
        return InternalSetState<false>(state);
    }

    [[maybe_unused]] bool StateMachine::SetMetaState(const StateSharedPtr &state) {
        return InternalSetState<true>(state);
    }

    [[maybe_unused]] void StateMachine::ClearMetaState() { meta_state_.reset(); }

    template<bool is_meta_state>
    inline bool StateMachine::InternalSetState(const StateSharedPtr &state) {
        if (states_.end() != std::find(states_.begin(), states_.end(), state)) {
            (is_meta_state ? meta_state_ : current_state_) = state;
            return true;
        } else return false;
    }


    void StateMachine::SetTimeout(uint64_t timeout) {
        timeout_ = timeout != 0 ? GetTime() + timeout : 0;
    }

    uint64_t StateMachine::timeout() const {
        return timeout_;
    }

    bool StateMachine::TimedOut() const {
        return timeout_ != 0 && GetTime() > timeout_;
    }

    bool StateMachine::Process(EventSharedPtr event) {
        DLOG(INFO) << "Process state machine name: " << name_
                   << " | type: "
                   << type_.name() << " | current state: "
                   << (current_state_ ? current_state_->name_ : "null") << ".";

        return ProcessNormalStateTransition(event) || ProcessMetaStateTransition(event);
    }

    bool StateMachine::ProcessNormalStateTransition(const EventSharedPtr &event) {
        if (current_state_) {
            TransitionSharedPtr transition;

            for (auto i = current_state_->transitions_.begin(); i != current_state_->transitions_.end(); ++i) {
                transition = *i;
                StateSharedPtr to_status = transition->to_.lock();

                if (transition->IsMatch(event, SELF)) {
                    LOG(INFO) << "Event (match) type: " << event->ToString()
                              << " | event type: " << type_.name()
                              << " | name: " << name_ << ".";

                    if (transition->transition_type() == Transition::TransitionType::kNormalTransition) {
                        std::stringstream status;
                        status << "at exit action: (" << type_.name()
                               << " " << name_ << ") "
                               << current_state_->name();

                        DLOG(INFO) << "Entering " << status.str() << ".";
                        try {
                            current_state_->OnExit(SELF, current_state_);
                            DLOG(INFO) << "Exited " << status.str() << ".";
                        } catch (...) {
                            LOG(ERROR) << "Caught exception " << status.str() << ".";
                        }

                        status.str("");

                        LOG(INFO) << "Normal transition from: " << current_state_->name()
                                  << " -> "
                                  << (to_status ? to_status->name() : "no_status")
                                  << ", in machine ("
                                  << type_.name() << " " << name_ << ").";
                        DLOG(INFO) << "Transition: ["
                                   << transition->name()
                                   << "].";
                    } else {
                        LOG(INFO) << "Internal transition from state: " << current_state_->name()
                                  << ", in machine"
                                  << " (" << type_.name() << " " << name_
                                  << ").";
                        DLOG(INFO) << "Transition: ["
                                   << transition->name()
                                   << "].";
                    }

                    try {
                        transition->OnTransition(SELF,
                                                 current_state_,
                                                 std::static_pointer_cast<ITransition>(transition),
                                                 event,
                                                 to_status);
                        if (transition->transition_type() == Transition::TransitionType::kNormalTransition) {
                            DLOG(INFO) << "Exited normal transition: ["
                                       << transition->name()
                                       << "] from: " << current_state_->name()
                                       << " -> " << (to_status ? to_status->name() : "no_status")
                                       << ", in machine"
                                       << " (" << type_.name() << " " << name_ << ").";
                        } else {
                            DLOG(INFO) << "Exited internal transition: ["
                                       << transition->name()
                                       << "] from state: " << current_state_->name()
                                       << ", in Machine"
                                       << " (" << type_.name() << " " << name_ << ").";
                        }
                    } catch (...) {
                        if (transition->transition_type() == Transition::TransitionType::kNormalTransition) {
                            LOG(ERROR) << "Caught exception at normal transition action: ["
                                       << transition->name()
                                       << "] from: " << current_state_->name()
                                       << " -> " << (to_status ? to_status->name() : "no_status")
                                       << ", in machine"
                                       << " (" << type_.name() << " " << name_ << ").";
                        } else {
                            LOG(ERROR) << "Caught exception at internal transition action: ["
                                       << transition->name()
                                       << "] from state: " << current_state_->name()
                                       << ", in machine"
                                       << " (" << type_.name() << " " << name_ << ").";
                        }
                    }

                    previous_state_ = current_state_;
                    current_state_ = transition->to_.lock();

                    if (transition->transition_type() == Transition::TransitionType::kNormalTransition) {
                        {
                            MachineSetSharedPtr machine_set = event->machine_set_.lock();
                            if (machine_set) {
                                machine_set->UpdateTimeoutMachine(SELF, current_state_->timeout());
                                SetTimeout(current_state_->timeout());
                            }

                            DLOG(INFO) << "Entering enter action: (" << type_.name()
                                       << " " << name_
                                       << ") " << (to_status ? to_status->name()
                                                             : "no_status") << ".";

                            try {
                                current_state_->OnEnter(SELF, current_state_);
                                DLOG(INFO) << "Exited enter action: (" << type_.name()
                                           << " " << name_ << ") "
                                           << (to_status ? to_status->name()
                                                         : "no_status") << ".";
                            } catch (...) {
                                LOG(ERROR) << "Caught exception at enter action: (" << type_.name() << " "
                                           << name_ << ") "
                                           << current_state_->name() << ".";
                            }
                        }

                        return true;
                    }
                } else {
                    DLOG(WARNING) << "No known transition match.";
                }
            }
        }

        return false;
    }

    bool StateMachine::ProcessMetaStateTransition(const EventSharedPtr &event) {
        // Check meta transitions AFTER specific transitions.
        if (meta_state_ && current_state_) {
            for (auto i = meta_state_->transitions_.begin();
                 i != meta_state_->transitions_.end(); ++i) {
                TransitionSharedPtr transition = *i;
                StateSharedPtr to_status = transition->to_.lock();

                if (transition->IsMatch(event, SELF)) {
                    if (transition->transition_type() == Transition::TransitionType::kNormalTransition) {
                        std::stringstream status;
                        status << "at exit action: (" << type_.name()
                               << " " << name_ << ") "
                               << current_state_->name();

                        DLOG(INFO) << "Entering meta " << status.str() << ".";
                        try {
                            current_state_->OnExit(SELF, current_state_);
                            DLOG(INFO) << "Exited meta " << status.str() << ".";
                        } catch (...) {
                            LOG(ERROR) << "Caught exception " << status.str() << ".";
                        }
                    }

                    LOG(INFO) << "Meta transition from: Empty -> " << (to_status ? to_status->name() : "no_status")
                              << ", in machine"
                              << " (" << type_.name() << " " << name_ << ").";
                    DLOG(INFO) << "Meta Transition: [" << transition->name()
                               << "].";

                    try {
                        transition->OnTransition(
                                SELF,
                                current_state_,
                                transition,
                                event,
                                to_status);
                        DLOG(INFO) << "Exited Meta Transition: [" << transition->name()
                                   << "] from: Empty -> "
                                   << (to_status ? to_status->name() : "no_status")
                                   << ", in machine"
                                   << " (" << type_.name() << " " << name_ << ").";
                    } catch (...) {
                        LOG(ERROR) << "Caught exception at Meta Transition action: [" << transition->name()
                                   << "] from: Empty -> "
                                   << (to_status ? to_status->name()
                                                 : "no_status")
                                   << ", in machine"
                                   << " (" << type_.name() << " "
                                   << name_ << ").";
                    }

                    current_state_ = to_status;

                    // Transitions to meta-state do not change the current state.
                    if (transition->transition_type() == Transition::TransitionType::kNormalTransition) {
                        MachineSetSharedPtr machine_set = event->machine_set_.lock();
                        // Reset timer only on entry.
                        if (machine_set &&
                            to_status) {
                            machine_set->UpdateTimeoutMachine(SELF, to_status->timeout());
                            SetTimeout(to_status->timeout());
                        }

                        DLOG(INFO) << "Entering meta enter action: (" << type_.name() << " "
                                   << name_ << ") "
                                   << current_state_->name() << ".";
                        try {
                            current_state_->OnEnter(SELF, current_state_);
                            DLOG(INFO) << "Exited meta enter action: (" << type_.name() << " "
                                       << name_ << ") "
                                       << current_state_->name() << ".";
                        } catch (...) {
                            LOG(ERROR) << "Caught exception at meta enter action: (" << type_.name() << " "
                                       << name_ << ") "
                                       << current_state_->name() << ".";
                        }
                    }
                    return true;
                }
            }
        }

        return false;
    }

    bool ActionMachine::Process(EventSharedPtr event) {
        DLOG(INFO) << "Process action machine name: " << name().c_str() << " | type:"
                   << type().name().c_str() << ".";

        for (const auto &non_transitive_transition: non_transitive_actions_) {
            if (non_transitive_transition->IsMatch(event, SELF)) {
                LOG(INFO) << "Non-transitive action: " << non_transitive_transition->name()
                          << " in "
                          << " (" << type_.name() << " " << name_ << ").";

                try {
                    non_transitive_transition->OnAction(SELF,
                                                        std::static_pointer_cast<ITransition>(
                                                                non_transitive_transition),
                                                        event);
                    DLOG(INFO) << "Exited Non-transitive action: " << non_transitive_transition->name()
                               << " in "
                               << " (" << type_.name() << " " << name_ << ").";
                } catch (...) {
                    LOG(ERROR) << "Caught exception at Non-transitive action: " << non_transitive_transition->name()
                               << " in (" << type_.name() << " "
                               << name_ << ").";
                }
                return true;
            }
        }
        return false;
    }
}
