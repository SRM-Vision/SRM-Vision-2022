#ifndef MACHINE_SET_HANDLER_H_
#define MACHINE_SET_HANDLER_H_

#include "fsm_base_types.h"

namespace fsm {
    class Event;

    class MachineSet;

    class MachineBase;

    class MachineSetHandler {
    public:
        enum class HandleResult {
            kSkip [[maybe_unused]],      ///< If skipped, do not call MachineSet's function.
            kContinue [[maybe_unused]]   ///< If continue, call MachineSet's function.
        };

    public:
        MachineSetHandler() = default;

        virtual ~MachineSetHandler() = default;

        [[maybe_unused]] static void ProcessEvent(const EventSharedPtr &event);

        [[maybe_unused]] static void ProcessTimeOutMachine(const MachineSetSharedPtr &machine_set,
                                                           const MachineBaseSharedPtr &machine);

    public:
        [[maybe_unused]] virtual HandleResult OnEventEnqueue(const EventSharedPtr &event) = 0;

        [[maybe_unused]] virtual int OnUpdateMachineTimeOut(const MachineSetSharedPtr &machine_set,
                                                            const MachineBase &machine,
                                                            time_t seconds) = 0;
    };

    class [[maybe_unused]] IMachineSetExternalHandler {
    public:
        virtual ~IMachineSetExternalHandler() = default;

        [[maybe_unused]] virtual void RegisterHandler(const MachineSetHandlerSharedPtr &handler) = 0;
    };
}

#endif  // MACHINE_SET_HANDLER_H_
