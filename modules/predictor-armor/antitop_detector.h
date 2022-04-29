#ifndef ANTITOP_DETECTOR_H_
#define ANTITOP_DETECTOR_H_

#include <iostream>
#include "lang-feature-extension/attr_reader.h"

constexpr double kTopPeriodThreshold = 10;  ///< Max TOP period.

class AntiTopDetector {
public:
    AntiTopDetector() = default;

    ATTR_READER(lasting_time_, TopPeriod)

    ATTR_READER(initialized_, Initialized)

    inline double GetTopPeriod() {
        lasting_time_ = 0;
        for (auto time: lasting_time_once_)
            lasting_time_ += time;
        return lasting_time_;
    }

    inline void Initialize(double last_time_stamp) {
        last_time_stamp_ = last_time_stamp;
        last_rotation_detection_ = false;
        switch_times_ = 0;
        lasting_time_ = 0;
        top_inited_ = false;
        initialized_ = true;
        lasting_time_once_[0] = 0;
        lasting_time_once_[1] = 0;
        lasting_time_once_[2] = 0;
        lasting_time_once_[3] = 0;
        time_record_ = 0;
    }

    //
    bool IsTop(bool same_robot, bool rotation_detection, bool switch_armor, double timestamp) {
        if (!initialized_) {
            Initialize(timestamp);
            return false;
        }
        if (!top_inited_)
            return TopInitialize(same_robot, rotation_detection, switch_armor, timestamp);
        else
            return ContinueJudge(same_robot, rotation_detection, switch_armor, timestamp);
    }

private:
    bool ContinueJudge(bool same_robot, bool rotation_detection, bool switch_armor, double timestamp) {
        // Not the same bot.
        if (!same_robot) {
            top_inited_ = false;
            return false;
        }
        // Rotating direction changed.
        if ((!rotation_detection && last_rotation_detection_)
            || (rotation_detection && !last_rotation_detection_)) {
            last_rotation_detection_ = rotation_detection;
            top_inited_ = false;
            return false;
        }
        last_rotation_detection_ = rotation_detection;
        time_record_ += (timestamp - last_time_stamp_) * 1e-9;
        last_time_stamp_ = timestamp;
        if (time_record_ > kTopPeriodThreshold / 4) {
            top_inited_ = false;
            return false;
        }
        if (switch_armor) {
            if (switch_times_ >= 4)
                switch_times_ = 0;
            else
                ++switch_times_;
            lasting_time_once_[switch_times_ - 1] = time_record_;
            time_record_ = 0;
        }
        return true;
    }

    bool TopInitialize(bool same_robot, bool rotation_detection, bool switch_armor, double timestamp) {
        if (!same_robot) return false;                           // 不是同一个机器人
        if ((!rotation_detection && last_rotation_detection_)     // 转向改变
            || (rotation_detection && !last_rotation_detection_)) {
            last_rotation_detection_ = rotation_detection;
            return false;
        }
        last_rotation_detection_ = rotation_detection;
        if (switch_armor)
            ++switch_times_;                                          // 同一辆车切换了装甲板
        if (1 <= switch_times_ && switch_times_ <= 4) {                 // 一个周期内累积时间，时间过长说明不是小陀螺
            lasting_time_once_[switch_times_ - 1] += (timestamp - last_time_stamp_) * 1e-9;
            lasting_time_ += (timestamp - last_time_stamp_) * 1e-9;
            last_time_stamp_ = timestamp;
            if (lasting_time_ > kTopPeriodThreshold) {
                switch_times_ = 0;
                lasting_time_ = 0;
                lasting_time_once_[0] = 0;
                lasting_time_once_[1] = 0;
                lasting_time_once_[2] = 0;
                lasting_time_once_[3] = 0;
                return false;
            }
        }
        if (switch_times_ == 5 && lasting_time_ <= kTopPeriodThreshold) {
            lasting_time_ = 0;
            switch_times_ = 0;
            top_inited_ = true;
            return true;
        }
        return false;
    }

    double lasting_time_{};
    bool top_inited_{};
    bool initialized_ = false;
    double last_time_stamp_{};
    bool last_rotation_detection_{};  // anticlockwise_: 1, clockwise_: 0
    double lasting_time_once_[4]{};
    double time_record_{};

    int switch_times_{};
};

#endif  // ANTITOP_DETECTOR_H_
