//
// Created by xiguang on 2022/7/10.
//

#include "digital-twin/components/armor.h"
#include "math-tools/algorithms.h"
#include "antitop_detector_renew.h"

void AntiTopDetectorRenew::UpdateTop(const int &new_armor_num, const uint64_t &now_timestamp) {
    auto new_top_period{algorithm::NanoSecondsToSeconds(timestamp_, now_timestamp) * 8};
    if(new_armor_num != armor_num_) {  //  from one armor to one armor is the quartet cycles. It`s 1/8 period.
        top_period_ = new_top_period;
        timestamp_ = now_timestamp;
        armor_num_ = new_armor_num;
    }else if(new_top_period > top_period_){  // if exceed period and armor num not change.
        top_period_ = new_top_period;
    }

    if(top_period_ < low_top_period_min_ && top_period_ > high_top_period_min_) {
        is_low_top_ = true;
        is_high_top_ = false;
    }else if(top_period_ < high_top_period_min_ && top_period_ > high_top_period_max_) {
        is_low_top_ = false;
        is_high_top_ = true;
    }else{
        is_low_top_ = is_high_top_ = false;
    }
}


void AntiTopDetectorRenew::UpdateClockwise(const std::vector <Armor> &armors) {
    static double center_x{-1};
    if(armors.size() == 1){
        center_x = armors.front().Center().x;
    }else if(armors.size() == 2){
        if(abs(center_x - armors.front().Center().x) > abs(center_x - armors.back().Center().x)){
            // the front is new one
            if(armors.front().Center().x > center_x){
                // new one is on the right
                clockwise_ = 1;
            }else{
                // new one is on the left
                clockwise_ = 0;
            }
        }else{
            // the back is new one
            if(armors.back().Center().x > center_x){
                // new one is on the right
                clockwise_ = 1;
            }else{
                // new one is on the left
                clockwise_ = 0;
            }
        }
        DLOG(INFO) << "CLOCKWISE: " << clockwise_;
    }
}

void AntiTopDetectorRenew::UpdateJumpTop(PredictArmor &predict_armor, uint64_t current_time,
                                         const std::array<float, 3> &yaw_pitch_roll) {
    double time_after_jump{algorithm::NanoSecondsToSeconds(last_jump_time_, current_time)};

    DLOG(INFO) << "TIME AFTER JUMP: " << time_after_jump;

    // If it exceeds the maximum period and has not turned, it is not top.
    if(time_after_jump > max_jump_period_ && !is_high_top_ && !is_low_top_){
        is_top_ = false;
        jump_count_ = 0;
    }

    DLOG(INFO) << "JUMP PERIOD: " << jump_period_;

    // When tracking the same armor plate
    double current_yaw{std::atan2(predict_armor.TranslationVectorWorld()(0,0),
                                  predict_armor.TranslationVectorWorld()(2,0))};
    double yaw_delta{algorithm::ShortestAngularDistance(last_yaw_, current_yaw)};
    DLOG(INFO) << "YAW DELTA: " << yaw_delta;
    // When jump enough
    if(std::abs(yaw_delta) > max_jump_yaw_){
        ++jump_count_;
        if(jump_count_ > 1 && std::signbit(yaw_delta) == std::signbit(last_yaw_jump_delta_)){
            is_top_ = true;
            jump_period_ = time_after_jump;
        }

        last_jump_time_ = current_time;
        last_yaw_jump_delta_ = yaw_delta;
        last_jump_position_ = predict_armor.TranslationVectorWorld();
    }

    last_yaw_ = current_yaw;

    if(is_top_){
        if(time_after_jump / jump_period_ < allow_follow_range_){
            predict_armor.fire_ = true;
        }else{
            predict_armor.translation_vector_world_ << last_jump_position_;
            predict_armor.predict_world_vector_ << last_jump_position_;
            predict_armor.UpdateShootPointAndPredictCam(yaw_pitch_roll);
        }
    }
}
