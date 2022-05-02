/**
 * Power rune definition header.
 * \author trantuan-20048607, LIYunzhe1408
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef POWER_RUNE_H_
#define POWER_RUNE_H_

#include "../facility.h"

class PowerRune : public Facility {
public:
    ATTR_READER(clockwise_, Clockwise)

    ATTR_READER_REF(rtg_vec_, RtgVec)

    ATTR_READER_REF(rtp_vec_, RtpVec)

    ATTR_READER_REF(center_r_, CenterR)

    ATTR_READER_REF(armor_center_p_, ArmorCenterP)

    ATTR_READER_REF(fan_center_g_, FanCenterG)

    ATTR_READER_REF(send_yaw_pitch_delay_, SendYawPitchDelay)

    ATTR_READER_REF(image_center_, ImageCenter)

    PowerRune() : Facility(), clockwise_(0) {}

    [[maybe_unused]] PowerRune(Colors color,
                               int clockwise,
                               cv::Point2f rtp_vec,
                               cv::Point2f rtg_vec,
                               cv::Point2f center_r,
                               cv::Point2f armor_center_p,
                               cv::Point2f fan_center_g,
                               cv::Point3f send_yaw_pitch_delay) :
            Facility(color, 0, kPowerRune),
            clockwise_(clockwise),
            rtp_vec_(rtp_vec),
            rtg_vec_(rtg_vec),
            center_r_(center_r),
            armor_center_p_(armor_center_p),
            fan_center_g_(fan_center_g),
            send_yaw_pitch_delay_(send_yaw_pitch_delay) {}

private:
    int clockwise_;

    cv::Point2f rtp_vec_;
    cv::Point2f rtg_vec_;

    cv::Point2f center_r_;
    cv::Point2f armor_center_p_;
    cv::Point2f fan_center_g_;

    cv::Point3f send_yaw_pitch_delay_;
    cv::Point2f image_center_;
};

#endif  // POWER_RUNE_H_
