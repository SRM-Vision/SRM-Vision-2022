#ifndef MACHINE_SET_H_
#define MACHINE_SET_H_

#include <set>
#include <thread>
#include <functional>
#include <atomic>
#include "event_queue.h"
#include "machine.h"

namespace fsm {
    class MachineSetHandler;

    class MachineBase;

    class MachineType;

    class Event;

    class MachineSet : public std::enable_shared_from_this<MachineSet> {
    public:
        [[maybe_unused]] static MachineSetSharedPtr MakeMachineSet();

        ~MachineSet();

        [[maybe_unused]] void RegisterHandler(MachineSetHandlerSharedPtr handler);

        [[maybe_unused]] void Enqueue(EventSharedPtr event);

        [[maybe_unused]] [[maybe_unused]] void StartBackground(unsigned int sleep_time);

        void StopBackground();

        [[maybe_unused]] [[maybe_unused]] void Process();

        void Process(const EventSharedPtr &event);

        void ProcessTimeoutMachine(const MachineBaseSharedPtr &machine);

        [[maybe_unused]] MachineBaseSharedPtr machine(const MachineType &type, const std::string &name);

        void UpdateTimeoutMachine(const MachineBase &machine, time_t timeout);

        [[maybe_unused]] inline bool HasHandler() const { return static_cast<bool>(event_handler_); }

        typedef std::function<void()> NotifyEvent;
        NotifyEvent OnProcessError;

    private:
        MachineSet()
                : owner_thread_id_(std::this_thread::get_id()),
                  background_thread_stop_flag_(false) {
        }

        void AddMachine(const MachineBaseSharedPtr &machine);

        void RemoveMachine(const MachineBaseSharedPtr &machine);

        bool ProcessTargetMachineEvent(const EventSharedPtr &event);

        bool ProcessNoTargetMachineEvent(const EventSharedPtr &event);

        void InternalProcessTimeoutEvent();

        typedef std::vector<MachineBaseSharedPtr> MachinePtrList;
        typedef std::set<MachineBaseSharedPtr> MachinePtrSet;

        MachinePtrList machine_list_;
        MachinePtrSet machine_set_;

        EventQueue<EventSharedPtr> event_queue_;

        MachineSetHandlerSharedPtr event_handler_;

        [[maybe_unused]] std::thread::id owner_thread_id_;

        std::atomic<bool> background_thread_stop_flag_;
        std::unique_ptr<std::thread> background_thread_;
    };
}

#endif  // MACHINE_SET_H_
