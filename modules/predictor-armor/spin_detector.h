//
// Created by xiguang on 2022/7/11.
//

#ifndef SPIN_DETECTOR_H_
#define SPIN_DETECTOR_H_

#include <digital-twin/components/armor.h>

class SpinDetector {
public:

    /**
     * @brief SPHERICAL mode uses yaw to judge spinning, FLAT mode uses x.
     */
    enum Mode{FLAT,
            SPHERICAL,
            SIZE};

    SpinDetector() = delete;

    /**
     * @brief Used for judge slow spin and high spin. Default parameters are only available for SPHERICAL mode.
     * @param mode SPHERICAL mode uses yaw to judge spinning, FLAT mode uses x.
     * @param min_jump_yaw_x when delta yaw/x bigger than that, consider it`s a jump.
     * @param slow_jump_period_max low speed spin must faster than this. The unit is radians or pixels.
     * @param quick_jump_period_max high speed spin must faster than this. The unit is radians or pixels.
     * @param quick_jump_period_min high speed spin must slower than this. The unit is radians or pixels.
     */
    explicit SpinDetector(Mode mode = Mode::SPHERICAL, double min_jump_yaw_x = 0.05, double slow_jump_period_max = 1.2,
                          double quick_jump_period_max = 0.625, double quick_jump_period_min = 0.125): min_jump_yaw_x_(min_jump_yaw_x),
                                                                                              slow_jump_period_max_(slow_jump_period_max),
                                                                                              quick_jump_period_max_(quick_jump_period_max),
                                                                                              quick_jump_period_min_(quick_jump_period_min),
                                                                                              mode_(mode){}

    /**
     * @brief Used to judge whether to spin.
     * @param mode SPHERICAL mode uses yaw to judge spinning, FLAT mode uses x.
     * @param min_jump_yaw_x when delta yaw/x bigger than that, consider it`s a jump.
     * @param jump_period_max spin must faster than this. The unit is radians or pixels.
     * @param jump_period_min spin must slower than this. The unit is radians or pixels.
     */
    explicit SpinDetector(Mode mode = Mode::SPHERICAL, double min_jump_yaw_x = 0.05, double jump_period_max = 1.2, double jump_period_min = 0.125):
            min_jump_yaw_x_(min_jump_yaw_x),
            slow_jump_period_max_(jump_period_max),
            quick_jump_period_max_(jump_period_min),
            quick_jump_period_min_(jump_period_min),
            mode_(mode){}

    /**
     * @brief Update spinning status.
     * @param armor current target armor
     * @param current_time current timestamp
     * @return updated spinning status
     */
    bool Update(const Armor &armor, uint64_t current_time);

    /**
     * @brief When has no armor was detected, reset.
     */
    void Reset();

    ATTR_READER(is_slow_spin_, IsSlowPin)

    ATTR_READER(is_quick_spin_, IsQuickSpin)

    ATTR_READER(clockwise_, Clockwise)

    ATTR_READER(jump_period_, JumpPeriod)

    ATTR_READER_REF(last_jump_position_, LastJumpPosition)

    ATTR_READER_REF(last_jump_time_, LastJumpTime)

    [[nodiscard]] bool IsSpin() const{return is_slow_spin_ || is_quick_spin_;}

private:

    double min_jump_yaw_x_{0.05}; // when delta yaw/x bigger than that, consider it`s a jump.
    double slow_jump_period_max_ {1.2};   // low speed spin must faster than this
    double quick_jump_period_max_ {0.625};   // high speed spin must faster than this
    double quick_jump_period_min_ {0.125};

    int clockwise_{-1};    /// -1 mean invalid, 0 mean anticlockwise, 1 mean clockwise. Look from above.
    bool is_slow_spin_{false};
    bool is_quick_spin_{false};

    int jump_count_{0};
    double last_yaw_x_{0};    // yaw/x in last frame
    double last_yaw_x_delta_{0};  // yaw/x delta when last jump

    uint64_t last_jump_time_{0};
    double jump_period_{0}; // yaw/x jump period
    coordinate::TranslationVector last_jump_position_{0,0,0};

    int reverse_buffer{0};  // when not jump, direction of yaw/x delta must opposite of jumping direction.

    Mode mode_{SPHERICAL};

};


#endif //SPIN_DETECTOR_H_
