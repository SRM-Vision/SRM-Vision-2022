#include <string>
#include <iostream>
#include <glog/logging.h>
#include "fsm-base/fsm_base_types.h"
#include "fsm-base/event.h"
#include "fsm-base/machine_operation_event.h"
#include "fsm-base/state.h"
#include "fsm-base/transition.h"
#include "fsm-base/transition_predicate.h"
#include "fsm-base/timeout_predicate.h"
#include "fsm-base/machine.h"
#include "fsm-base/machine_set.h"

enum class FoodEventType {
    kLogin,
    kLoginFailed,
    kLoginOK,
    kLogout,
};

class FoodEvent : public fsm::EventTemplate<FoodEventType> {
public:
    using underlying_type = std::underlying_type<FoodEventType>::type;

    FoodEvent(FoodEventType type, const fsm::MachineBaseSharedPtr &machine)
            : fsm::EventTemplate<FoodEventType>(type, machine) {
    }

    std::ostream &ToStream(std::ostream &str) const override {
        return str << "FoodEvent | " << static_cast<underlying_type>(type_);
    }

    [[nodiscard]] std::string ToString() const override {
        std::ostringstream os;
        os << "FoodEvent | " << static_cast<underlying_type>(type_);
        std::string str = os.str();
        os.str("");
        return str;
    }
};

class FoodMachine : public fsm::StateMachine {
public:
    explicit FoodMachine(const std::string &name);

    void Initialize() override;

    fsm::StateSharedPtr startup_;
    fsm::StateSharedPtr logging_;
    fsm::StateSharedPtr welcome_;

    fsm::TransitionSharedPtr startup_logging_;
    fsm::TransitionSharedPtr logging_welcome_;
    fsm::TransitionSharedPtr logging_startup_;
    fsm::TransitionSharedPtr welcome_startup_;
    fsm::TransitionSharedPtr welcome_timeout_;
};

FoodMachine::FoodMachine(const std::string &name)
        : StateMachine(name) {}

void FoodMachine::Initialize() {
    startup_ = fsm::State::MakeState(*this, "STARTUP");
    logging_ = fsm::State::MakeState(*this, "LOGGING");
    welcome_ = fsm::State::MakeState(*this, "WELCOME", 5000);

    startup_logging_ = fsm::Transition::MakeTransition("STARTUP -> LOGGING <== LOGIN",
                                                       startup_,
                                                       logging_,
                                                       std::make_shared<fsm::SimplePredicate<FoodEvent>>
                                                               (FoodEventType::kLogin));
    logging_welcome_ = fsm::Transition::MakeTransition("LOGGING -> WELCOME <== LOGIN OK",
                                                       logging_,
                                                       welcome_,
                                                       std::make_shared<fsm::SimplePredicate<FoodEvent>>
                                                               (FoodEventType::kLoginOK));
    logging_startup_ = fsm::Transition::MakeTransition("LOGGING -> STARTUP <== LOGIN FAILED",
                                                       logging_,
                                                       startup_,
                                                       std::make_shared<fsm::SimplePredicate<FoodEvent>>
                                                               (FoodEventType::kLoginFailed));
    welcome_startup_ = fsm::Transition::MakeTransition("WELCOME -> STARTUP <== LOGOUT",
                                                       welcome_,
                                                       startup_,
                                                       std::make_shared<fsm::SimplePredicate<FoodEvent>>
                                                               (FoodEventType::kLogout));
    welcome_timeout_ = fsm::Transition::MakeTransition("WELCOME TIMEOUT",
                                                       welcome_,
                                                       welcome_,
                                                       std::make_shared<fsm::TimeoutPredicate>(type_));
}

int main([[maybe_unused]] int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_log_dir = "../../log";
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_max_log_size = 16;

    fsm::MachineSetSharedPtr machine_set = fsm::MachineSet::MakeMachineSet();
    if (machine_set) {
        // Start event process thread.
        machine_set->StartBackground(500);

        // Add food machine into machine set.
        std::shared_ptr<FoodMachine> food_machine = fsm::MakeStateMachine<FoodMachine>("Food Machine #1");
        if (food_machine) {
            food_machine->SetStartState(food_machine->startup_);

            // Set states' callback.
            food_machine->startup_->OnEnter = [&](fsm::MachineBase &machine,
                                                  const fsm::StateSharedPtr &state) {
                std::cout << "Enter " << state->name() << std::endl;
            };
            food_machine->startup_->OnExit = [&](fsm::MachineBase &machine,
                                                 const fsm::StateSharedPtr &state) {
                std::cout << "Exit " << state->name() << std::endl;
            };

            food_machine->logging_->OnEnter = [&](fsm::MachineBase &machine,
                                                  const fsm::StateSharedPtr &state) {
                std::cout << "Enter " << state->name() << std::endl;
            };
            food_machine->logging_->OnExit = [&](fsm::MachineBase &machine,
                                                 const fsm::StateSharedPtr &state) {
                std::cout << "Exit " << state->name() << std::endl;
            };

            food_machine->welcome_->OnEnter = [&](fsm::MachineBase &machine,
                                                  const fsm::StateSharedPtr &state) {
                std::cout << "Enter " << state->name() << std::endl;
            };
            food_machine->welcome_->OnExit = [&](fsm::MachineBase &machine,
                                                 const fsm::StateSharedPtr &state) {
                std::cout << "Exit " << state->name() << std::endl;
            };

            food_machine->startup_logging_->OnTransition = [&](fsm::MachineBase &machine,
                                                               const fsm::StateSharedPtr &from_state,
                                                               const fsm::ITransitionSharedPtr &transition,
                                                               const fsm::EventSharedPtr &event,
                                                               const fsm::StateSharedPtr &to_state) {
                std::cout << transition->name()
                          << " | "
                          << from_state->name()
                          << " -> "
                          << to_state->name() << std::endl;
            };
            food_machine->logging_welcome_->OnTransition = [&](fsm::MachineBase &,
                                                               const fsm::StateSharedPtr &from_state,
                                                               const fsm::ITransitionSharedPtr &transition,
                                                               const fsm::EventSharedPtr &event,
                                                               const fsm::StateSharedPtr &to_state) {
                std::cout << transition->name()
                          << " | "
                          << from_state->name()
                          << " -> "
                          << to_state->name() << std::endl;
            };
            food_machine->logging_startup_->OnTransition = [&](fsm::MachineBase &,
                                                               const fsm::StateSharedPtr &from_state,
                                                               const fsm::ITransitionSharedPtr &transition,
                                                               const fsm::EventSharedPtr &event,
                                                               const fsm::StateSharedPtr &to_state) {
                std::cout << transition->name()
                          << " | "
                          << from_state->name()
                          << " -> "
                          << to_state->name() << std::endl;
            };
            food_machine->welcome_startup_->OnTransition = [&](fsm::MachineBase &,
                                                               const fsm::StateSharedPtr &from_state,
                                                               const fsm::ITransitionSharedPtr &transition,
                                                               const fsm::EventSharedPtr &event,
                                                               const fsm::StateSharedPtr &to_state) {
                std::cout << transition->name()
                          << " | "
                          << from_state->name()
                          << " -> "
                          << to_state->name() << std::endl;
            };
            food_machine->welcome_timeout_->OnTransition = [&](fsm::MachineBase &,
                                                               const fsm::StateSharedPtr &from_state,
                                                               const fsm::ITransitionSharedPtr &transition,
                                                               const fsm::EventSharedPtr &event,
                                                               const fsm::StateSharedPtr &to_state) {
                std::cout << transition->name()
                          << " | "
                          << from_state->name()
                          << " -> "
                          << to_state->name() << std::endl;
            };

            // Enqueue events.
            machine_set->Enqueue(std::make_shared<fsm::MachineOperationEvent>(
                    fsm::MachineOperator::kAdd,
                    food_machine));
            machine_set->Enqueue(std::make_shared<FoodEvent>(
                    FoodEventType::kLogin,
                    food_machine));
            machine_set->Enqueue(std::make_shared<FoodEvent>(
                    FoodEventType::kLoginOK,
                    food_machine));
            machine_set->Enqueue(std::make_shared<FoodEvent>(
                    FoodEventType::kLogout,
                    food_machine));
        }
    }

    std::cin.get();

    return 0;
}
