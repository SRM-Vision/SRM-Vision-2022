//
// Created by xiguang on 2022/4/29.
//

#ifndef ANTITOP_DETECTOR_RENEW_H_
#define ANTITOP_DETECTOR_RENEW_H_

class AntiTopDetectorRenew{
public:
    AntiTopDetectorRenew() = default;

    explicit AntiTopDetectorRenew(const uint64_t& timestamp): timestamp_(timestamp){}

    double UpdateTop(const int& new_armor_num,const uint64_t& now_timestamp){
        if(new_armor_num == 1 && armor_num_ == 2) {  //  from one armor to one armor is the quartet cycles
            top_period_ = double(now_timestamp - timestamp_) * 4 * 1e-9;
            timestamp_ = now_timestamp;
        }
        armor_num_ = new_armor_num;
//        DLOG(INFO) << "top period: " << top_period_;
        if(top_period_ > kLowTopPeriodThreshold && top_period_ < kHighTopPeriodThreshold) {
            is_low_top_ = true;
            is_high_top_ = false;
        }else if(top_period_ > kHighTopPeriodThreshold) {
            is_low_top_ = false;
            is_high_top_ = true;
        }else{
            is_low_top_ = is_high_top_ = false;
        }
    }

    ATTR_READER(is_low_top_, IsLowTop);
    ATTR_READER(is_high_top_, IsHighTop);

private:
    static constexpr double kLowTopPeriodThreshold = 2;   // low speed top must faster than this
    static constexpr double kHighTopPeriodThreshold = 5;   //  high speed top must faster than this

    int armor_num_{0};
    bool is_low_top_{false};
    bool is_high_top_{false};
    double top_period_{0};
    uint64_t timestamp_{0};
};

#endif //ANTITOP_DETECTOR_RENEW_H_
