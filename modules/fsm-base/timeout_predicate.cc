#include "timeout_predicate.h"
#include "timeout_event.h"
#include "machine.h"

namespace fsm {
    [[maybe_unused]] TimeoutPredicate::TimeoutPredicate(const MachineType &machine_type)
            : source_machine_type_(machine_type) {
    }

    bool TimeoutPredicate::operator()(const EventSharedPtr &event,
                                      const MachineBase &machine) {
        auto timeout_event = std::dynamic_pointer_cast<TimeoutEvent>(event);
        return (timeout_event &&
                source_machine_type_ == timeout_event->machine_type() &&
                machine.name() == timeout_event->machine_name());
    }
}
