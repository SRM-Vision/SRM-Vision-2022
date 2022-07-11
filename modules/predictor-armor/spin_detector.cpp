//
// Created by xiguang on 2022/7/11.
//

#include <chrono>
#include "spin_detector.h"

/// when not jump, yaw/x delta must opposite of jumping direction.
const int kMaxReverseSpinBuffer{20};

bool SpinDetector::Update(const Armor &armor, const uint64_t current_time) {
    double time_after_last_jump{algorithm::Duration(last_jump_time_, current_time)};

    DLOG(INFO) << "TIME AFTER JUMP: " << time_after_last_jump;

    // If it exceeds the maximum period and has not turned, it is not top.
    if((is_slow_spin_ || is_quick_spin_) && time_after_last_jump > slow_jump_period_max_){
        is_slow_spin_ = is_quick_spin_ = false;
        jump_count_ = 0;
    }else if(time_after_last_jump > quick_jump_period_max_ && is_quick_spin_){
        is_slow_spin_ = true;
        is_quick_spin_ = false;
    }

    double current_yaw_x, yaw_delta;
    if(mode_ == Mode::SPHERICAL){
        current_yaw_x = std::atan2(armor.TranslationVectorWorld()(0, 0),
                                   armor.TranslationVectorWorld()(2,0));
        yaw_delta = algorithm::shortest_angular_distance(last_yaw_x_, current_yaw_x);
    }else{
        current_yaw_x = armor.Center().x;
        yaw_delta = current_yaw_x - last_yaw_x_;
    }

    if(abs(yaw_delta) > min_jump_yaw_x_){
        reverse_buffer = 0;
        ++jump_count_;
        if(jump_count_ > 1 && std::signbit(last_yaw_x_delta_) == std::signbit(yaw_delta)){
            if(time_after_last_jump > quick_jump_period_max_ && time_after_last_jump < slow_jump_period_max_) {
                is_slow_spin_ = true;
                is_quick_spin_ = false;
                jump_period_ = time_after_last_jump;
            }else if(time_after_last_jump > quick_jump_period_min_){
                is_quick_spin_ = true;
                is_slow_spin_ = false;
                jump_period_ = time_after_last_jump;
            }
            // if time_after_last_jump < quick_jump_period_min_, consider it that flicker.
        }
        if(std::signbit(yaw_delta)) // if yaw_delta is negative.
            clockwise_ = 0;
        else
            clockwise_ = 1;
        last_jump_time_ = current_time;
        last_yaw_x_delta_ = yaw_delta;
        last_jump_position_ = armor.TranslationVectorWorld();
    }else if(std::signbit(yaw_delta) == std::signbit(last_yaw_x_delta_)){   // not jump but yaw delta has the same direction.
        ++reverse_buffer;
        if(reverse_buffer > kMaxReverseSpinBuffer){
            reverse_buffer = 0;
            jump_count_ = 0;
            is_quick_spin_ = is_slow_spin_ = false;
        }
    }else{
        reverse_buffer = 0;
    }

    last_yaw_x_ = current_yaw_x;

    return IsSpin();
}

void SpinDetector::Reset() {
    jump_count_ = 0;
    last_yaw_x_ = 0;
    reverse_buffer = 0;
}
