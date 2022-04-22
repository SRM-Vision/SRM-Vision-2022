//
// Created by xiguang on 2022/3/11.
//

#include "predictor_fsm.h"


ArmorMachine::ArmorMachine(const std::string &name) :
        StateMachine(name) {}

void ArmorMachine::Initialize() {
    one_ = fsm::State::MakeState(*this, "ONE");
    two_without_antitop_ = fsm::State::MakeState(*this, "TWO WITHOUT ANTITOP");
    two_with_antitop_ = fsm::State::MakeState(*this, "TWO WITH ANTITOP");

    one_two_without_antitop_ = fsm::Transition::MakeTransition("ONE -> TWO WITHOUT ANTITOP",
                                                               one_,
                                                               two_without_antitop_,
                                                               std::make_shared<fsm::SimplePredicate<ArmorEvent>>
                                                                       (ArmorEventType::kTwoWithoutAntiTop));
    two_without_antitop_one_ = fsm::Transition::MakeTransition("TWO WITHOUT ANTITOP ->ONE",
                                                               two_without_antitop_,
                                                               one_,
                                                               std::make_shared<fsm::SimplePredicate<ArmorEvent>>
                                                                       (ArmorEventType::kOne));
    one_two_with_antitop_ = fsm::Transition::MakeTransition("ONE -> TWO WITH ANTITOP",
                                                            one_,
                                                            two_with_antitop_,
                                                            std::make_shared<fsm::SimplePredicate<ArmorEvent>>
                                                                    (ArmorEventType::kTwoWithAntiTop));
    two_with_antitop_two_without_antitop_ = fsm::Transition::MakeTransition("TWO WITH ANTITOP -> TWO WITHOUT ANTITOP",
                                                                            two_with_antitop_,
                                                                            two_without_antitop_,
                                                                            std::make_shared<fsm::SimplePredicate<ArmorEvent>>
                                                                                    (ArmorEventType::kTwoWithoutAntiTop));
    two_without_antitop_two_with_antitop_ = fsm::Transition::MakeTransition("TWO WITHOUT ANTITOP -> TWO WITH ANTITOP",
                                                                            two_without_antitop_,
                                                                            two_with_antitop_,
                                                                            std::make_shared<fsm::SimplePredicate<ArmorEvent>>
                                                                                    (ArmorEventType::kTwoWithAntiTop));
    one_one_ = fsm::Transition::MakeTransition("ONE -> ONE",
                                               one_,
                                               one_,
                                               std::make_shared<fsm::SimplePredicate<ArmorEvent>>
                                                       (ArmorEventType::kOne));
    two_without_antitop_two_without_antitop_ = fsm::Transition::MakeTransition(
            "TWO WITHOUT ANTITOP -> TWO WITHOUT ANTITOP",
            two_without_antitop_,
            two_without_antitop_,
            std::make_shared<fsm::SimplePredicate<ArmorEvent>>
                    (ArmorEventType::kTwoWithoutAntiTop));
    two_with_antitop_two_with_antitop_ = fsm::Transition::MakeTransition("TWO WITH ANTITOP -> TWO WITH ANTITOP",
                                                                         two_with_antitop_,
                                                                         two_with_antitop_,
                                                                         std::make_shared<fsm::SimplePredicate<ArmorEvent>>
                                                                                 (ArmorEventType::kTwoWithAntiTop));
    two_with_antitop_one_ = fsm::Transition::MakeTransition("TWO WITH ANTITOP -> ONE",
                                                            two_with_antitop_,
                                                            one_,
                                                            std::make_shared<fsm::SimplePredicate<ArmorEvent>>
                                                                    (ArmorEventType::kOne));
}


