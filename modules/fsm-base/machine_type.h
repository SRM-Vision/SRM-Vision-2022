#ifndef MACHINE_TYPE_H_
#define MACHINE_TYPE_H_

#include <string>
#include "lang-feature-extension/attr_reader.h"
#include "lang-feature-extension/self_tag.h"
#include "lang-feature-extension/disable_constructor.h"

namespace fsm {
    class MachineType {
    public:
        ATTR_READER(id_, id)

        ATTR_READER_REF(name_, name)

        explicit MachineType(std::string name);

        MachineType(const MachineType &machine_type) = default;

        ~MachineType() = default;

        inline bool operator==(const MachineType &machine_type) const { return id_ == machine_type.id_; }

        inline bool operator!=(const MachineType &machine_type) const { return id_ != machine_type.id_; }

    private:
        unsigned int id_;
        std::string name_;
    };
}

#endif  // MACHINE_TYPE_H_
