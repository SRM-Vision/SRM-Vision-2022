#ifndef TRANSITION_H_
#define TRANSITION_H_

#include <functional>
#include "fsm_base_types.h"

namespace fsm {
    class MachineType;

    class ITransition : NO_COPY, NO_MOVE {
    public:
        virtual ~ITransition() = default;

        virtual bool IsMatch(const EventSharedPtr &event,
                             const MachineBase &machine) = 0;

        [[nodiscard]] [[maybe_unused]] virtual bool IsValid() const = 0;

        [[nodiscard]] virtual const std::string &name() const = 0;
    };

    typedef std::shared_ptr<ITransition> ITransitionSharedPtr;

    typedef std::function<
    void(MachineBase
    &,
    [[maybe_unused]] const StateSharedPtr &from_state,
            ITransitionSharedPtr,
            EventSharedPtr,
    [[maybe_unused]]
    const StateSharedPtr &to_state
    )>
    TransitionFireType;

    typedef std::function<void(MachineBase & , ITransitionSharedPtr, EventSharedPtr)> ActionFireType;

    class Transition : public ITransition,
                       public std::enable_shared_from_this<Transition> {
        friend class StateMachine;

    public:
        OVERRIDE_ATTR_READER(const std::string &, name_, name)

        ATTR_READER(transition_type_, transition_type)

        enum class TransitionType {
            kNormalTransition,
            kInternalTransition
        };

        [[maybe_unused]] [[maybe_unused]] static TransitionSharedPtr MakeTransition(const char *name,
                                                                                    const StateSharedPtr &from,
                                                                                    const StateSharedPtr &to,
                                                                                    IPredicateSharedPtr predicate);

        [[maybe_unused]] [[maybe_unused]] static TransitionSharedPtr MakeTransition(const char *name,
                                                                                    const StateSharedPtr &toFrom,
                                                                                    IPredicateSharedPtr predicate);

        ~Transition() override = default;

        inline bool IsValid() const override { return predicate_ && is_valid_; }

        bool IsMatch(const EventSharedPtr &event,
                     const MachineBase &machine) override;


        [[maybe_unused]] void ClearActions();

        TransitionFireType OnTransition;

    private:
        Transition(const char *name,
                   const StateSharedPtr &from,
                   const StateSharedPtr &to,
                   IPredicateSharedPtr pred);

        Transition(const char *name,
                   const StateSharedPtr &to_from,
                   IPredicateSharedPtr pred);

        void Init();

        TransitionType transition_type_;
        std::string name_;
        IPredicateSharedPtr predicate_;

        StateWeakPtr to_;
        StateWeakPtr hold_from_;
        StateWeakPtr hold_to_;

        bool is_valid_;
    };


    class NonTransitiveAction : public ITransition,
                                public std::enable_shared_from_this<NonTransitiveAction> {
        friend class ActionMachine;

    public:
        [[maybe_unused]] [[maybe_unused]] static NonTransitiveActionSharedPtr
        MakeNonTransitiveAction(const char *name,
                                ActionMachine &owner_machine,
                                IPredicateSharedPtr predicate);

        ~NonTransitiveAction() override = default;

        NonTransitiveAction &operator=(const NonTransitiveAction &) = delete;

        OVERRIDE_ATTR_READER(const std::string &, name_, name)

        inline bool IsValid() const override { return static_cast<bool>(predicate_); }

        bool IsMatch(const EventSharedPtr &event,
                     const MachineBase &machine) override;

        [[maybe_unused]] [[maybe_unused]] void ClearActions();

        ActionFireType OnAction;

    private:
        NonTransitiveAction(const char *name,
                            IPredicateSharedPtr predicate);

        void Init(ActionMachine &ownerMachine);

        std::string name_;
        IPredicateSharedPtr predicate_;
    };
}

#endif  // TRANSITION_H_
