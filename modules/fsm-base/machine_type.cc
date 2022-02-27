#include <atomic>
#include "machine_type.h"

namespace fsm {
    static std::atomic<unsigned int> kMachineTypeIdCount(0);

    MachineType::MachineType(std::string name)
            : id_(++kMachineTypeIdCount),
              name_(std::move(name)) {
    }
}
