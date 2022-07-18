/**
 * Power rune definition header.
 * \author trantuan-20048607, LIYunzhe1408
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef POWER_RUNE_H_
#define POWER_RUNE_H_

#include <utility>

#include "../facility.h"


// FIXME [LIYunzhe1408] ?
class PowerRune : public Facility {
public:
    ATTR_READER(clockwise_, Clockwise)

    ATTR_READER(rtp_vec_, RtpVec)

    ATTR_READER(center_r_, CenterR)

    ATTR_READER(armor_center_p_, ArmorCenterP)

    ATTR_READER(image_center_, ImageCenter)

    ATTR_READER(time_gap_, TimeGap)

    ATTR_READER(current_time_, CurrentTime)

    PowerRune() : Facility(), clockwise_(0), time_gap_(0) {}

    [[maybe_unused]] PowerRune(Colors color,
                               int clockwise,
                               double time_gap,
                               double current_time,
                               cv::Point2f rtp_vec,
                               cv::Point2f center_r,
                               cv::Point2f armor_center_p,
                               cv::Point2f image_center) :
            Facility(color, 0, kPowerRune),
            clockwise_(clockwise),
            time_gap_(time_gap),
            current_time_(current_time),
            rtp_vec_(std::move(rtp_vec)),
            center_r_(std::move(center_r)),
            armor_center_p_(std::move(armor_center_p)),
            image_center_(std::move(image_center)){}

private:
    int clockwise_;
    double time_gap_;
    double current_time_;

    cv::Point2f center_r_;
    cv::Point2f armor_center_p_;
    cv::Point2f rtp_vec_;
    cv::Point2f image_center_;
};

#endif  // POWER_RUNE_H_
