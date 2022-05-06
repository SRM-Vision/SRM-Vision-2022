/**
 * Hero controller header.
 * \author LIYunzhe1408
 * \date 2022.2.11
 * \warning NEVER include this file except in ./controller_hero.cpp.
 */

#ifndef CONTROLLER_HERO_H_
#define CONTROLLER_HERO_H_
class [[maybe_unused]] HeroController final : public Controller {
public:
    bool Initialize() final;

    void Run() final;

private:
    /// Own registry in controller factory.
    [[maybe_unused]] static ControllerRegistry<HeroController> hero_controller_registry_;


    OutpostDetector outpost_detector_;
    Outpost outpost_;
    OutpostPredictor outpost_predictor_;
};

#endif  // CONTROLLER_HERO_H_
