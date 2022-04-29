#ifndef ANTITOP_DETECTOR_H_
#define ANTITOP_DETECTOR_H_

#include <iostream>
#include "lang-feature-extension/attr_reader.h"

const double kTopPeriodThreshold = 10;   // 小陀螺最大的周期，超过该周期不是小陀螺
class AntitopDetector {
public:
    AntitopDetector() = default;

    ATTR_READER(lasting_time, TopReriod)

    ATTR_READER(initialized, IsInit)

    inline double GetTopPeriod() {
        lasting_time = 0;
        for (auto time: lasting_time_once)
            lasting_time += time;
        return lasting_time;
    }

    inline void Initialize(double last_time_stamp) {
        last_time_stamp_ = last_time_stamp;
        last_rotation_detection_ = false;
        switch_times_ = 0;
        lasting_time = 0;
        top_inited = false;
        initialized = true;
        lasting_time_once[0] = 0;
        lasting_time_once[1] = 0;
        lasting_time_once[2] = 0;
        lasting_time_once[3] = 0;
        time_record = 0;
    }

    //
    bool Is_Top(bool same_robot, bool rotation_detection, bool switch_armor, double timestamp) {
        if (!initialized) {
            Initialize(timestamp);
            return false;
        }
        if (!top_inited)
            return top_initialize(same_robot, rotation_detection, switch_armor, timestamp);
        else
            return continue_judge(same_robot, rotation_detection, switch_armor, timestamp);
    }

private:
    bool continue_judge(bool same_robot, bool rotation_detection, bool switch_armor, double timestamp) {
        if (!same_robot) {
            top_inited = false;
            return false;
        }                              // 不是同一个机器人
        if ((!rotation_detection && last_rotation_detection_)                                // 转向改变
            || (rotation_detection && !last_rotation_detection_)) {
            last_rotation_detection_ = rotation_detection;
            top_inited = false;
            return false;
        }
        last_rotation_detection_ = rotation_detection;
        time_record += (timestamp - last_time_stamp_) * 1e-9;
        last_time_stamp_ = timestamp;
        if (time_record > kTopPeriodThreshold / 4) {
            top_inited = false;
            return false;
        }
        if (switch_armor) {
            if (switch_times_ >= 4)
                switch_times_ = 0;
            else
                ++switch_times_;
            lasting_time_once[switch_times_ - 1] = time_record;
            time_record = 0;


        }
        return true;
    }

    bool top_initialize(bool same_robot, bool rotation_detection, bool switch_armor, double timestamp) {
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
            lasting_time_once[switch_times_ - 1] += (timestamp - last_time_stamp_) * 1e-9;
            lasting_time += (timestamp - last_time_stamp_) * 1e-9;
            last_time_stamp_ = timestamp;
            if (lasting_time > kTopPeriodThreshold) {
                switch_times_ = 0;
                lasting_time = 0;
                lasting_time_once[0] = 0;
                lasting_time_once[1] = 0;
                lasting_time_once[2] = 0;
                lasting_time_once[3] = 0;
                return false;
            }
        }
        if (switch_times_ == 5 && lasting_time <= kTopPeriodThreshold) {
            lasting_time = 0;
            switch_times_ = 0;
            top_inited = true;
            return true;
        }
        return false;
    }


    double lasting_time{};
    bool top_inited{};
    bool initialized = false;
    double last_time_stamp_{};
    bool last_rotation_detection_{};   //  anticlockwise_ : 1,clockwise_ : 0
    double lasting_time_once[4]{};
    double time_record{};

    int switch_times_{};
};

#endif  // ANTITOP_DETECTOR_H_
