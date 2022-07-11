//
// Created by xiguang on 2022/4/29.
//

#ifndef ANTITOP_DETECTOR_RENEW_H_
#define ANTITOP_DETECTOR_RENEW_H_

#include "predict_armor.h"

class AntiTopDetectorRenew{
public:
    AntiTopDetectorRenew() = default;

    explicit AntiTopDetectorRenew(const uint64_t& timestamp): timestamp_(timestamp),last_jump_time_(timestamp){}

    /**
     * @brief update anti-top state
     * @param new_armor_num the current number of armors on one robot.
     * @param now_timestamp current timestamp.
     */
    void UpdateTop(const int& new_armor_num,const uint64_t& now_timestamp);

    /**
     * @brief update the clockwise state in corresponding robot.
     * @param armors one or two armors in the robot.
     */
    void UpdateClockwise(const std::vector<Armor> &armors);

    void UpdateJumpTop(PredictArmor &predict_armor, uint64_t current_time,
                       const std::array<float, 3> &yaw_pitch_roll);

    void Reset(){
        is_high_top_ = is_low_top_ = false;
        clockwise_ = -1;
        jump_period_ = top_period_ = 0;
        jump_count_ = 0;
        last_jump_position_ = Eigen::Vector3d{0,0,0};
        last_yaw_ = 0;
        last_yaw_jump_delta_ = 0;
        is_top_ = false;
    }

    ATTR_READER(is_low_top_, IsLowTop)

    ATTR_READER(is_high_top_, IsHighTop)

    ATTR_READER(is_top_,IsTop)

    ATTR_READER_REF(clockwise_,Clockwise)

    ATTR_READER_REF(top_period_, TopPeriod)


private:
    static constexpr double low_top_period_min_ = 5;   // low speed top must faster than this
    static constexpr double high_top_period_min_ = 2.5;   // high speed top must faster than this
    [[maybe_unused]] static constexpr double high_top_period_max_ = 0.5;

    static constexpr double max_jump_yaw_{0.05};
    static constexpr double max_jump_period_{0.8};
    static constexpr double allow_follow_range_{0.6};

    int clockwise_{-1};    /// -1 mean invalid, 0 mean anticlockwise, 1 mean clockwise. Look from above.
    int armor_num_{0};
    bool is_low_top_{false};
    bool is_high_top_{false};
    double top_period_{0};
    uint64_t timestamp_{0};

    bool is_top_{false};

    double jump_period_{0}; // yaw jump period
    int jump_count_{0};
    double last_yaw_{0};
    double last_yaw_jump_delta_{0};

    uint64_t last_jump_time_{0};
    coordinate::TranslationVector last_jump_position_{0,0,0};
};

#endif //ANTITOP_DETECTOR_RENEW_H_
