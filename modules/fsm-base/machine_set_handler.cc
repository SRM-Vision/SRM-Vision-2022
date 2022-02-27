#include "event.h"
#include "machine_set.h"
#include "machine_set_handler.h"

namespace fsm {
    [[maybe_unused]] void MachineSetHandler::ProcessEvent(const EventSharedPtr &event) {
        if (event && event->machine_set())
            event->machine_set()->Process(event);
    }

    [[maybe_unused]] void MachineSetHandler::ProcessTimeOutMachine(
            const MachineSetSharedPtr &machine_set,
            const MachineBaseSharedPtr &machine) {
        if (machine_set)
            machine_set->ProcessTimeoutMachine(machine);
    }
}
