#ifndef TIMEOUT_PREDICATE_H_
#define TIMEOUT_PREDICATE_H_

#include "transition_predicate.h"
#include "machine_type.h"

namespace fsm {

    class [[maybe_unused]] TimeoutPredicate : public IPredicate {
    public:
        [[maybe_unused]] explicit TimeoutPredicate(const MachineType &machine_type);

        bool operator()(const EventSharedPtr &event,
                        const MachineBase &machine) override;

    private:
        MachineType source_machine_type_;
    };
}

#endif  // TIMEOUT_PREDICATE_H_
