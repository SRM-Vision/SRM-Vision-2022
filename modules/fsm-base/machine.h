#ifndef MACHINE_H_
#define MACHINE_H_

#include <type_traits>
#include "fsm_base_types.h"
#include "machine_type.h"

namespace fsm {
    class StateMachine;

    /**
     * \brief Generate a custom state machine.
     * \tparam M Machine type.
     * \tparam Args Additional template args.
     * \param args Additional function args.
     * \return A shared pointer to newly generated machine.
     */
    template<typename M, typename... Args,
            class = typename std::enable_if<std::is_base_of<StateMachine, typename std::decay<M>::type>::value>::type>
    [[maybe_unused]] std::shared_ptr<typename std::decay<M>::type> MakeStateMachine(Args &&... args) {
        using RealType = typename std::decay<M>::type;
        std::shared_ptr<RealType> machine = std::make_shared<RealType>(std::forward<Args>(args)...);
        if (machine) { machine->Initialize(); }
        return machine;
    }

    /// \brief Base class of all types of machine.
    class MachineBase {
    public:
        VIRTUAL_ATTR_READER(uint64_t, 0, timeout)

        VIRTUAL_ATTR_READER(bool, false, TimedOut)

        ATTR_READER_REF(type_, type)

        ATTR_READER_REF(name_, name)

        explicit MachineBase(std::string name)
                : type_(name),
                  name_(std::move(name)) {}

        virtual ~MachineBase() = default;

        virtual bool Process(EventSharedPtr) = 0;

        virtual void SetTimeout(uint64_t timeout) = 0;

        virtual void Initialize() = 0;

    protected:
        MachineType type_;
        std::string name_;
    };

    /**
     * \brief State machine class.
     * \details All custom state machine types inherit this class. You should add your own
     *   states and transitions into the custom class. For example, there's a food vending
     *   machine that requires user's logging in its own account. In this case, there are
     *   3 states: STARTUP, LOGGING, WELCOME; and 5 transitions: STARTUP->LOGGING, LOGGING
     *   ->WELCOME, LOGGING->STARTUP, WELCOME->STARTUP, WELCOME->END. You should define then
     *   as public StateSharedPtr and TransitionSharedPtr member. After this, you MUST write
     *   your own initialization function to correctly initialize it. Commonly, states and
     *   transitions will be made by a making function in initializing process. Note that initialization
     *   function do NOT accept any parameters, instead, these params can be passed into and
     *   processed in constructor. See event.h for how to define events. Here is code for this
     *   example:
     *
     * \code{.cpp}
     *   class FoodMachine : public fsm::StateMachine {
     *   public:
     *       explicit FoodMachine(const std::string &name);
     *
     *   public:
     *       void Initialize() override;
     *
     *   public:
     *       fsm::StateSharedPtr startup_;
     *       fsm::StateSharedPtr logging_;
     *       fsm::StateSharedPtr welcome_;
     *
     *       fsm::TransitionSharedPtr startup_logging_;
     *       fsm::TransitionSharedPtr logging_welcome_;
     *       fsm::TransitionSharedPtr logging_startup_;
     *       fsm::TransitionSharedPtr welcome_startup_;
     *       fsm::TransitionSharedPtr welcome_timeout_;
     *   };
     *
     *   FoodMachine::FoodMachine(const std::string &name)
     *           : StateMachine(name) {
     *   }
     *
     *   void FoodMachine::Initialize() {
     *       startup_ = fsm::State::MakeState(*this, "STARTUP");
     *       logging_ = fsm::State::MakeState(*this, "LOGGING");
     *       welcome_ = fsm::State::MakeState(*this, "WELCOME", 5000);
     *
     *       startup_logging_ = fsm::Transition::MakeTransition("STARTUP -> LOGGING <== LOGIN",
     *                                                          startup_,
     *                                                          logging_,
     *                                                          std::make_shared<fsm::SimplePredicate<FoodEvent>>
     *                                                                  (FoodEventType::kLogin));
     *       logging_welcome_ = fsm::Transition::MakeTransition("LOGGING -> WELCOME <== LOGIN OK",
     *                                                          logging_,
     *                                                          welcome_,
     *                                                          std::make_shared<fsm::SimplePredicate<FoodEvent>>
     *                                                                  (FoodEventType::kLoginOK));
     *       logging_startup_ = fsm::Transition::MakeTransition("LOGGING -> STARTUP <== LOGIN FAILED",
     *                                                          logging_,
     *                                                          startup_,
     *                                                          std::make_shared<fsm::SimplePredicate<FoodEvent>>
     *                                                                  (FoodEventType::kLoginFailed));
     *       welcome_startup_ = fsm::Transition::MakeTransition("WELCOME -> STARTUP <== LOGOUT",
     *                                                          welcome_,
     *                                                          startup_,
     *                                                          std::make_shared<fsm::SimplePredicate<FoodEvent>>
     *                                                                  (FoodEventType::kLogout));
     *       welcome_timeout_ = fsm::Transition::MakeTransition("WELCOME TIMEOUT",
     *                                                          welcome_,
     *                                                          welcome_,
     *                                                          std::make_shared<fsm::TimeoutPredicate>(type_));
     *   }
     * \endcode
     */
    class StateMachine : public MachineBase {
        friend class State;

    public:
        ATTR_READER_REF(current_state_, current_state)

        ATTR_READER_REF(previous_state_, previous_state)

        explicit StateMachine(std::string name)
                : MachineBase(std::move(name)),
                  timeout_(0) {}

        StateMachine &operator=(const StateMachine &) = delete;

        ~StateMachine() override = default;

        /**
         * \brief Force state machine into a state.
         * \param [in] state Target state.
         * \return Whether machine entered this state.
         * \warning This function may break the balance in this machine and cause unexpected result.
         */
        [[maybe_unused]] bool ForceState(const StateSharedPtr &state);

        /**
         * \brief Set the initial state.
         * \param [in] state Target state.
         * \return Whether machine entered this state.
         */
        [[maybe_unused]] bool SetStartState(const StateSharedPtr &state);

        [[maybe_unused]] bool SetMetaState(const StateSharedPtr &state);

        [[maybe_unused]] void ClearMetaState();

        /**
         * \brief Set the timeout manually.
         * \param timeout Timeout in milliseconds.
         * \warning It seems that you should not call this function.
         */
        void SetTimeout(uint64_t timeout) override;

        [[nodiscard]] uint64_t timeout() const override;

        [[nodiscard]] bool TimedOut() const override;

        bool Process(EventSharedPtr) override;

    private:
        bool ProcessNormalStateTransition(const EventSharedPtr &event);

        bool ProcessMetaStateTransition(const EventSharedPtr &event);

        template<bool is_meta_state>
        bool InternalSetState(const StateSharedPtr &state);

    protected:
        StateMachine(const StateMachine &state_machine)
                : MachineBase(state_machine),
                  timeout_(state_machine.timeout_) {
        }

        uint64_t timeout_;  ///< DEADLINE of an operation, not real timeout.
        StateSharedPtr current_state_;
        StateSharedPtr previous_state_;

        typedef std::vector<StateSharedPtr> StateListType;
        StateListType states_;

        StateSharedPtr meta_state_;
    };

    class ActionMachine : public MachineBase {
        friend class NonTransitiveAction;

    public:
        explicit ActionMachine(std::string name)
                : MachineBase(std::move(name)) {}

        bool Process(EventSharedPtr) override;

    protected:
        typedef std::vector<NonTransitiveActionSharedPtr> ActionListType;
        ActionListType non_transitive_actions_;
    };
}

#endif  // MACHINE_H_
