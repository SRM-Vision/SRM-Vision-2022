#include "timeout_event.h"

namespace fsm {
    std::ostream &TimeoutEvent::ToStream(std::ostream &ostream) const {
        ostream << static_cast<const char *>("TimeoutEvent[") << machine_type_.name()
                << static_cast<const char *>(", ") << machine_name_
                << static_cast<const char *>("]");
        return ostream;
    }

    [[nodiscard]] std::string TimeoutEvent::ToString() const {
        std::stringstream str;
        str << static_cast<const char *>("TimeoutEvent[") << machine_type_.name()
            << static_cast<const char *>(", ") << machine_name_
            << static_cast<const char *>("]");

        return str.str();
    }
}
