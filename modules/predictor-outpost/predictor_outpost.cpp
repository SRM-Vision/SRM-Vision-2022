#include "predictor_outpost.h"


bool OutpostPredictor::Initialize(const std::string &config_path, bool debug) {
    OutpostPredictorDebug::Instance().Initialize(config_path, debug);
}


SendPacket OutpostPredictor::Run(Battlefield battlefield, float bullet_speed) {
    /*
     * Initialize
     */
    outpost_.ClearBottomArmor();
    SendPacket send_packet{0, 0, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0, 0};


    /*
     * Collect armors.
     * TODO Add height judgment.
     */
    auto facilities = battlefield.Facilities();
    auto robots = battlefield.Robots();
    for (auto &robot: robots[enemy_color_]) {
        for (auto &armor: robot.second->Armors())
            outpost_.AddBottomArmor(armor);
    }
    for (auto &facility: facilities[enemy_color_]) {
        for (auto &armor: facility.second->BottomArmors())
            outpost_.AddBottomArmor(armor);
    }

    /*
     * Decide mode.
     */
    if (outpost_.BottomArmors().empty() && prepared_) {
        return {0, 0, 0,
                10, 0,
                0, 0, 0, 0,
                0, 0, 0, 0, 0};
    }
    if (outpost_.BottomArmors().empty()) {
        return {0, 0, 0,
                0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0, 0};
    }

    /*
     * Set spin start time
     */
    if (need_init_) {
        start_time_ = std::chrono::high_resolution_clock::now();
        need_init_ = false;
    }

    /*
     * Decide rotate direction and coming/going armor.
     */
    if (clockwise_ <= 7 && clockwise_ >= -7 && !checked_clockwise_)
        IsClockwise();
    else
        DecideComingGoing();


    auto current_time_chrono = std::chrono::high_resolution_clock::now();
    /// time_gap is time period from start spinning time to now.
    double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time_chrono -
                                                                              start_time_)).count();
    /// biggest_armor is the armor with biggest area in horizon.
    int biggest_armor = FindBiggestArmor(outpost_.BottomArmors());

    /*
     * Detect for a specific time to decide center point.
     */
    if (time_gap * 1e-3 < kFindBiggestArmorTime && !prepared_) {
        if (outpost_.BottomArmors()[biggest_armor].Area() > biggest_area_)
            biggest_area_ = outpost_.BottomArmors()[biggest_armor].Area();
        auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(
                outpost_.BottomArmors()[biggest_armor].TranslationVectorCam());
        send_packet.yaw = shoot_point_spherical(0, 0), send_packet.pitch = shoot_point_spherical(1, 0);

    } else if (!prepared_) {
        auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(
                outpost_.BottomArmors()[biggest_armor].TranslationVectorCam());
        send_packet.yaw = shoot_point_spherical(0, 0), send_packet.pitch = shoot_point_spherical(1, 0);

        if (kAreaThreshold * biggest_area_ < outpost_.BottomArmors()[biggest_armor].Area()) {
            aim_buff_ += 2;
        } else if (kAreaThresholdLow * biggest_area_ < outpost_.BottomArmors()[biggest_armor].Area()) {
            ++aim_buff_;
        } else aim_buff_ = 0;
        if (aim_buff_ > kAimBuff) {
            outpost_.center_point_ = outpost_.BottomArmors()[biggest_armor].Center();
            prepared_ = true;
            start_time_ = std::chrono::high_resolution_clock::now();
        }
    }

    /*
     * Ready to auto-shoot.
     */
    if (prepared_) {
        double time_gap2 = (static_cast<std::chrono::duration<double, std::milli>>(current_time_chrono -
                                                                                   start_time_)).count();

        if (time_gap2 * 1e-3 > 0.1)                // Send communication signal every 0.1s.
            send_packet.distance_mode = 10;
        /// Pixel distance is to calculate shoot time
        double pixel_distance = abs(outpost_.center_point_.x - outpost_.coming_center_.x);
        if (pixel_distance < 10 && !ready_fire_) {
            ready_time_ = std::chrono::high_resolution_clock::now();
            ready_fire_ = true;
        }
        if (ready_fire_) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time -
                                                                                      ready_time_)).count();
            shoot_delay_time_ = OutpostPredictorDebug::Instance().ShootDelay();
            if (shoot_delay_time_ < time_gap * 1e-3) {
                send_packet.fire = 1;
                ready_fire_ = false;
            }
        }
    }

    if (!prepared_) {
        send_packet.pitch += 0.11;
        send_packet.yaw += 0.04;
    }
    return send_packet;

}


int OutpostPredictor::FindBiggestArmor(const std::vector<Armor> &armors) {
    if (armors.size() == 1)
        return 0;
    int biggest_id = 0;
    double biggest_area = 0;
    for (int i = 0; i < armors.size(); i++) {
        if (armors[i].Area() > biggest_area)
            biggest_id = i;
        biggest_area = armors[i].Area();
    }
    return biggest_id;
}

void OutpostPredictor::DecideComingGoing() {
    if (outpost_.BottomArmors().size() == 1) {
        if (clockwise_ == 1) {
            if (outpost_.BottomArmors()[0].Center().x <= outpost_.center_point_.x) {
                outpost_.going_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.coming_center_ = cv::Point2f(0x3f3f3f3f, 0x3f3f3f3f);
            }
            if (outpost_.BottomArmors()[0].Center().x > outpost_.center_point_.x) {
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_ = cv::Point2f(0x3f3f3f3f, 0x3f3f3f3f);
            }
        } else if (clockwise_ == -1) {
            if (outpost_.BottomArmors()[0].Center().x >= outpost_.center_point_.x) {
                outpost_.going_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.coming_center_ = cv::Point2f(0x3f3f3f3f, 0x3f3f3f3f);
            }
            if (outpost_.BottomArmors()[0].Center().x < outpost_.center_point_.x) {
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_ = cv::Point2f(0x3f3f3f3f, 0x3f3f3f3f);
            }
        }
    } else {
        if (clockwise_ == 1) {
            if (outpost_.BottomArmors()[0].Center().x > outpost_.BottomArmors()[1].Center().x) {
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_ = outpost_.BottomArmors()[1].Center();
            } else {
                outpost_.coming_center_ = outpost_.BottomArmors()[1].Center();
                outpost_.going_center_ = outpost_.BottomArmors()[0].Center();
            }
        } else if (clockwise_ == -1) {
            if (outpost_.BottomArmors()[0].Center().x > outpost_.BottomArmors()[1].Center().x) {
                outpost_.coming_center_ = outpost_.BottomArmors()[1].Center();
                outpost_.going_center_ = outpost_.BottomArmors()[0].Center();
            } else {
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_ = outpost_.BottomArmors()[1].Center();
            }
        }
    }
}

void OutpostPredictor::IsClockwise() {
    LOG(INFO) << "nums of armor" << outpost_.BottomArmors().size();
    double this_armor_x;
    if (outpost_.BottomArmors().size() == 1)
        this_armor_x = outpost_.BottomArmors()[0].Center().x;
    else {
        for (int i = 0; i < outpost_.BottomArmors().size() - 1; ++i)
            if (outpost_.BottomArmors()[i].Center().x > outpost_.BottomArmors()[i + 1].Center().x)
                this_armor_x = outpost_.BottomArmors()[i + 1].Center().x;
    }
    double diff = this_armor_x - last_armor_x_;
    if (diff < 0)
        clockwise_++;
    else if (diff > 0)
        clockwise_--;
    else
        LOG(INFO) << "Outpost clockwise something wrong";
    if (clockwise_ > 7) {
        LOG(INFO) << "Outpost is Clockwise";
        clockwise_ = 1;
        checked_clockwise_ = true;
    } else if (clockwise_ < -7) {
        LOG(INFO) << "Outpost is anti-Clockwise";
        clockwise_ = -1;
        checked_clockwise_ = true;
    }
}

void OutpostPredictor::Clear() {
    outpost_.ClearBottomArmor();
    outpost_.ClearTopArmor();


    checked_clockwise_ = false;
    clockwise_ = 0;

    last_armor_x_ = 0;

    ready_fire_ = false;
    prepared_ = false;
    need_init_ = true;

    aim_buff_ = 0;
}
