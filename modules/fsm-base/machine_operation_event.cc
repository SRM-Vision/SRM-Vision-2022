#include "machine.h"
#include "machine_operation_event.h"

namespace fsm {
    [[maybe_unused]] std::ostream &MachineOperationEvent::ToStream(std::ostream &ostream) const {
        ostream << static_cast<const char *>(type_ == MachineOperator::kAdd ?
                                             "AddMachineEvent" : "RemoveMachineEvent")
                << static_cast<const char *>("[")
                << static_cast<const char *>(" ") << machine_->name()
                << static_cast<const char *>("]");
        return ostream;
    }

    [[nodiscard]] std::string MachineOperationEvent::ToString() const {
        std::stringstream string_stream;
        string_stream << static_cast<const char *>(type_ == MachineOperator::kAdd ?
                                                   "AddMachineEvent" : "RemoveMachineEvent")
                      << static_cast<const char *>("[")
                      << static_cast<const char *>(" ") << machine_->name()
                      << static_cast<const char *>("]");
        return string_stream.str();
    }
}
