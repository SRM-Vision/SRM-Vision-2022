#include <glog/logging.h>
#include "timeout_event.h"
#include "machine_operation_event.h"
#include "machine_set.h"
#include "machine_set_handler.h"

namespace fsm {
    [[maybe_unused]] MachineSetSharedPtr MachineSet::MakeMachineSet() {
        return MachineSetSharedPtr(new MachineSet());
    }

    MachineSet::~MachineSet() {
        event_queue_.Clear();
        StopBackground();
    }

    [[maybe_unused]] void MachineSet::Enqueue(EventSharedPtr event) {
        event->machine_set_ = shared_from_this();
        if (event_handler_ && event_handler_->OnEventEnqueue(event) == MachineSetHandler::HandleResult::kSkip) {}
        else {
            event_queue_.Add(event);
        }
    }

    [[maybe_unused]] void MachineSet::RegisterHandler(MachineSetHandlerSharedPtr handler) {
        event_handler_ = std::move(handler);
    }

    void MachineSet::AddMachine(const MachineBaseSharedPtr &machine) {
        if (machine_set_.insert(machine).second)
            machine_list_.push_back(machine);
    }

    void MachineSet::RemoveMachine(const MachineBaseSharedPtr &machine) {
        if (machine_set_.erase(machine))
            for (auto i = machine_list_.begin(); i != machine_list_.end(); ++i)
                if (i->get() == machine.get()) {
                    machine_list_.erase(i);
                    return;
                }
    }

    MachineBaseSharedPtr MachineSet::machine(const MachineType &type, const std::string &name) {
        MachineBaseSharedPtr found = nullptr;
        for (auto &i: machine_list_)
            if (i->type() == type && i->name() == name) {
                found = i;
                break;
            }
        return found;
    }

    void MachineSet::InternalProcessTimeoutEvent() {
        if (!event_handler_)
            for (auto &machine: machine_list_)
                if (machine->TimedOut()) {
                    machine->SetTimeout(0);
                    std::shared_ptr<TimeoutEvent> timeout_event =
                            std::make_shared<TimeoutEvent>(shared_from_this(),
                                                           machine->type(),
                                                           machine->name());
                    machine->Process(timeout_event);
                }
    }

    [[maybe_unused]] void MachineSet::Process() {
        while (event_queue_.MessageAvailable())
            Process(event_queue_.Next());
        InternalProcessTimeoutEvent();
    }

    bool MachineSet::ProcessTargetMachineEvent(const EventSharedPtr &event) {
        bool handled = false;

        for (auto i = event->target_machines_.begin(), e = event->target_machines_.end();
             i != e; ++i) {
            auto machine = i->lock();
            if (machine) {
                auto found = machine_set_.find(machine);
                if (found != machine_set_.end()) {
                    DLOG(INFO) << __FUNCTION__ << " | target machine found: " << (*found)->name() << ".";
                    handled |= (*found)->Process(event);
                }
            }
        }

        return handled;
    }

    bool MachineSet::ProcessNoTargetMachineEvent(const EventSharedPtr &event) {
        bool handled = false;

        for (auto i = machine_list_.rbegin(), e = machine_list_.rend();
             i != e; ++i) {
            handled |= (*i)->Process(event);
        }

        return handled;
    }

    void MachineSet::Process(const EventSharedPtr &event) {
        try {
            auto operate_machine_event = std::dynamic_pointer_cast<MachineOperationEvent>(event);
            if (operate_machine_event) {
                if (MachineOperator::kAdd == operate_machine_event->type()) {
                    AddMachine(operate_machine_event->machine());
                } else if (MachineOperator::kRemove == operate_machine_event->type()) {
                    RemoveMachine(operate_machine_event->machine());
                }

                return;
            }

            LOG(INFO) << "Handling event: " << event->ToString() << ".";

            bool handled = event->target_machines_.empty() ?
                           ProcessNoTargetMachineEvent(event) : ProcessTargetMachineEvent(event);

            if (!handled) {
                LOG(ERROR) << "Unhandled event: " << event->ToString() << ".";
            }
        } catch (std::exception &e) {
            LOG(ERROR) << __FUNCTION__ << " | caught exception: " << e.what() << ".";
            OnProcessError();
        } catch (...) {
            LOG(ERROR) << __FUNCTION__ << " | caught unknown exception.";
            OnProcessError();
        }
    }

    void MachineSet::ProcessTimeoutMachine(const MachineBaseSharedPtr &machine) {
        auto found = machine_set_.find(machine);
        if (found != machine_set_.end()) {
            auto state_machine = std::dynamic_pointer_cast<StateMachine>(*found);
            if (state_machine) {
                std::shared_ptr<TimeoutEvent> timeout_event = std::make_shared<TimeoutEvent>(
                        shared_from_this(),
                        state_machine->type(),
                        state_machine->name());
                state_machine->SetTimeout(0);
                try {
                    state_machine->Process(timeout_event);
                } catch (std::exception &e) {
                    LOG(ERROR) << __FUNCTION__ << " | caught exception: " << e.what() << ".";
                    OnProcessError();
                } catch (...) {
                    LOG(ERROR) << __FUNCTION__ << " | caught unknown exception.";
                    OnProcessError();
                }
            }
        }
    }

    void MachineSet::UpdateTimeoutMachine(const MachineBase &machine, time_t timeout) {
        if (event_handler_ && timeout > 0) {
            event_handler_->OnUpdateMachineTimeOut(shared_from_this(), machine, timeout);
        }
    }

    [[maybe_unused]] void MachineSet::StartBackground(unsigned int sleep_time) {
        if (!background_thread_) {
            background_thread_ = std::make_unique<std::thread>([&, sleep_time]() {
                while (!background_thread_stop_flag_.load()) {
                    EventSharedPtr event = event_queue_.Next(sleep_time);
                    if (event) {
                        Process(event);
                    }
                    InternalProcessTimeoutEvent();
                }
            });
        }
    }

    void MachineSet::StopBackground() {
        if (background_thread_ &&
            !background_thread_stop_flag_.load()) {
            background_thread_stop_flag_.store(true);
            background_thread_->join();
        }
    }
}
