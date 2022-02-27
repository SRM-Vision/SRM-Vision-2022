#ifndef TRANSITION_PREDICATE_H_
#define TRANSITION_PREDICATE_H_

#include "fsm_base_types.h"

namespace fsm {
    class IPredicate {
    public:
        virtual ~IPredicate() = default;

        virtual bool operator()(const EventSharedPtr &,
                                const MachineBase &machine) = 0;
    };

    template<typename EventType>
    class [[maybe_unused]] SimplePredicate : public IPredicate {
    public:
        typedef typename EventType::Type EventEnum;

        [[maybe_unused]] explicit SimplePredicate(EventEnum type)
                : type_(type) {}

        inline bool operator()(const EventSharedPtr &event,
                               const MachineBase &machine) override {
            auto levent = std::dynamic_pointer_cast<EventType>(event);
            return (levent && levent->type() == type_);
        }

    private:
        EventEnum type_;
    };
}

#endif  // TRANSITION_PREDICATE_H_
