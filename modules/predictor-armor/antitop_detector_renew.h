//
// Created by xiguang on 2022/4/29.
//

#ifndef ANTITOP_DETECTOR_RENEW_H_
#define ANTITOP_DETECTOR_RENEW_H_

class AntiTopDetectorRenew{
public:
    AntiTopDetectorRenew() = default;

    explicit AntiTopDetectorRenew(const uint64_t& timestamp): timestamp_(timestamp){}

    void UpdateTop(const int& new_armor_num,const uint64_t& now_timestamp){
        auto new_top_period{double(now_timestamp - timestamp_) * 8 * 1e-9};
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
        }else if(top_period_ < high_top_period_min_) {
            is_low_top_ = false;
            is_high_top_ = true;
        }else{
            is_low_top_ = is_high_top_ = false;
        }
    }

    /**
     * @brief update the clockwise state in corresponding robot.
     * @param armors one or two armors in the robot.
     */
    void UpdateClockwise(const std::vector<Armor> &armors){
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

    void Reset(){
        is_high_top_ = is_low_top_ = false;
        clockwise_ = -1;
        top_period_ = 0;
    }

    ATTR_READER(is_low_top_, IsLowTop);
    ATTR_READER(is_high_top_, IsHighTop);
    ATTR_READER_REF(clockwise_,Clockwise);
    ATTR_READER_REF(top_period_, TopPeriod);

private:
    static constexpr double low_top_period_min_ = 5;   // low speed top must faster than this
    static constexpr double high_top_period_min_ = 2.5;   //  high speed top must faster than this
    [[maybe_unused]] static constexpr double high_top_period_max_ = 0.5;

    int clockwise_{-1};    /// -1 mean invalid, 0 mean anticlockwise, 1 mean clockwise. Look from above.
    int armor_num_{0};
    bool is_low_top_{false};
    bool is_high_top_{false};
    double top_period_{0};
    uint64_t timestamp_{0};
};

#endif //ANTITOP_DETECTOR_RENEW_H_
