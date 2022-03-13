/**
 * \brief Event and event template class header.
 * \author trantuan-20048607
 * \date 2022.2.18
 */

#ifndef EVENT_H_
#define EVENT_H_

#include "fsm_base_types.h"

namespace fsm {
    class MachineBase;

    class MachineSet;

    /**
     * \brief Base class of all types of event.
     * \warning Use EventTemplate INSTEAD OF Event to define custom event type!
     */
    class Event {
        friend class MachineSet;

        friend class StateMachine;

        friend class ActionMachine;

    public:
        VIRTUAL_ATTR_READER(MachineSetSharedPtr, machine_set_.lock(), machine_set)

        /**
         * \brief Convert event to stream for outputting and logging.
         * \param [in,out] ostream Stream to operate.
         * \return Stream itself.
         */
        virtual std::ostream &ToStream(std::ostream &ostream) const = 0;

        /**
         * \brief Convert event to string for outputting and logging.
         * \return Event information in string.
         */
        [[nodiscard]] virtual std::string ToString() const = 0;

        explicit Event(const MachineSetSharedPtr &machine_set)
                : machine_set_(machine_set) {}

        virtual ~Event() = default;

    protected:
        MachineSetWeakPtr machine_set_;
        MachineBaseWeakPtrVec target_machines_;
    };

    /**
     * \brief Template for custom event type.
     * \tparam EventType An enumerate type including all types of this custom event.
     * \details To define a custom type of event, you should first define an enum that
     *   includes all possible type of this event. For example, there's a food vending
     *   machine that requires user's logging in its own account. In this case, there are
     *   4 possible event types that are LOGIN, LOGIN_OK, LOGIN_FAILED, LOGOUT. Then, you
     *   should pack them into a enumerate type and use this as EventType to create a
     *   custom event type for food vending machine. You shouldn't create another type for
     *   this machine, though custom Event class and StateMachine class are not one-to-one
     *   correspondent. Here is code for this example:
     *
     * \code{.cpp}
     *   enum class FoodEventType {
     *       kLogin,
     *       kLoginFailed,
     *       kLoginOK,
     *       kLogout,
     *   };
     *   class FoodEvent : public fsm::EventTemplate<FoodEventType> {
     *   public:
     *       using underlying_type = std::underlying_type<FoodEventType>::type;
     *
     *       FoodEvent(FoodEventType type, const fsm::MachineBaseSharedPtr &machine)
     *               : fsm::EventTemplate<FoodEventType>(type, machine) {}
     *
     *       std::ostream &ToStream(std::ostream &str) const override { ... }
     *
     *       std::string ToString() const override { ... }
     *   };
     * \endcode
     *
     * \warning You MUST realize functions ToString() and ToStream() or you will get errors.
     */
    template<class EventType,
            class = typename std::enable_if<std::is_enum<EventType>::value>::type>
    class EventTemplate : public Event {
        friend class MachineSet;

        friend class StateMachine;

        friend class ActionMachine;

    public:
        typedef EventType Type;

        ATTR_READER_REF(type_, type)

        explicit EventTemplate(Type type)
                : Event(MachineSetSharedPtr()),
                  type_(type) {
        }

        [[maybe_unused]] EventTemplate(Type type, const MachineBaseWeakPtrVec &target_machines)
                : Event(MachineSetSharedPtr()),
                  type_(type) {
            target_machines_ = target_machines;
        }

        [[maybe_unused]] EventTemplate(Type type, const MachineBaseSharedPtr &target_machine)
                : Event(MachineSetSharedPtr()),
                  type_(type) {
            if (target_machine)
                target_machines_.emplace_back(target_machine);
        }

        EventTemplate(Type type, const MachineSetSharedPtr &machine_set)
                : Event(machine_set),
                  type_(type) {
        }

        ~EventTemplate() override = default;

    protected:
        Type type_;
    };

    inline std::ostream &operator<<(std::ostream &str, const Event &event) { return event.ToStream(str); }
}

#endif  // EVENT_H_
