#ifndef TIMEOUT_EVENT_H_
#define TIMEOUT_EVENT_H_

#include <sstream>
#include "event.h"
#include "fsm_base_types.h"
#include "machine_type.h"

namespace fsm {
    class MachineSet;

    class MachineType;

    enum class TimeoutEventType {
        kTimeout
    };

    class TimeoutEvent : public EventTemplate<TimeoutEventType> {
    public:
        ATTR_READER_REF(machine_type_, machine_type)

        ATTR_READER_REF(machine_name_, machine_name)

        TimeoutEvent(const MachineSetSharedPtr &machine_set,
                     const MachineType &machine_type,
                     std::string machine_name)
                : EventTemplate<TimeoutEventType>(TimeoutEventType::kTimeout, machine_set),
                  machine_type_(machine_type),
                  machine_name_(std::move(machine_name)) {}

        std::ostream &ToStream(std::ostream &ostream) const override;

        [[nodiscard]] std::string ToString() const override;

    private:
        MachineType machine_type_;
        std::string machine_name_;
    };
}

#endif  // TIMEOUT_EVENT_H_
