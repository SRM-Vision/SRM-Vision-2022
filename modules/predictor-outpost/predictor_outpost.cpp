#include "predictor_outpost.h"

const cv::Size kZoomRatio = {16, 30};

bool OutpostPredictor::Initialize() {
#if !NDEBUG
    OutpostPredictorDebug::Instance().addTrackbar();
#endif
    return true;
}


SendPacket OutpostPredictor::OldRun(Battlefield battlefield) {
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
        UpdateROICorners(outpost_.BottomArmors()[biggest_armor]);
        auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(
                outpost_.BottomArmors()[biggest_armor].TranslationVectorCam());
        send_packet.yaw = shoot_point_spherical(0, 0), send_packet.pitch = shoot_point_spherical(1, 0);

    } else if (!prepared_) {
        auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(
                outpost_.BottomArmors()[biggest_armor].TranslationVectorCam());
        UpdateROICorners(outpost_.BottomArmors()[biggest_armor]);
        send_packet.yaw = shoot_point_spherical(0, 0), send_packet.pitch = shoot_point_spherical(1, 0);
        DLOG(INFO) << "outpost distance： " << outpost_.BottomArmors()[biggest_armor].Distance();
        DLOG(INFO) << "armor length： " << norm(outpost_.BottomArmors()[biggest_armor].Corners()[0] -
                                               outpost_.BottomArmors()[biggest_armor].Corners()[1]);
        DLOG(INFO) << "armor height： " << outpost_.BottomArmors()[biggest_armor].TranslationVectorWorld();

        if (kAreaThreshold * biggest_area_ < outpost_.BottomArmors()[biggest_armor].Area()) {
            aim_buff_ += 2;
        } else if (kAreaThresholdLow * biggest_area_ < outpost_.BottomArmors()[biggest_armor].Area()) {
            ++aim_buff_;
        } else aim_buff_ = 0;

        if (aim_buff_ > kAimBuff) {
            outpost_center_ = outpost_.BottomArmors()[biggest_armor].Center();
            last_armor_x_ = outpost_.BottomArmors()[biggest_armor].Center().x;
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
        if (pixel_distance < 15 && !ready_fire_) {
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

    if (!prepared_ && send_packet.pitch != 0) {
        send_packet.pitch += 0.08;
        send_packet.yaw += 0.02;
        send_packet.yaw = send_packet.yaw + OutpostPredictorDebug::Instance().DeltaYawLeft() -
                          OutpostPredictorDebug::Instance().DeltaYawRight();
        send_packet.pitch = send_packet.pitch + OutpostPredictorDebug::Instance().DeltaPitchDown() -
                            OutpostPredictorDebug::Instance().DeltaPitchUp();

    }
    fire_ = send_packet.fire;

    send_packet.check_sum = send_packet.yaw + send_packet.pitch + send_packet.delay +
                            float(send_packet.fire) + float(send_packet.distance_mode) +
                            float(send_packet.point1_x) + float(send_packet.point1_y) +
                            float(send_packet.point2_x) + float(send_packet.point2_y) +
                            float(send_packet.point3_x) + float(send_packet.point3_y) +
                            float(send_packet.point4_x) + float(send_packet.point4_y);

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
                outpost_.coming_center_ = cv::Point2f(0, 0);
            }
            if (outpost_.BottomArmors()[0].Center().x > outpost_.center_point_.x) {
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_ = cv::Point2f(0, 0);
            }
        } else if (clockwise_ == -1) {
            if (outpost_.BottomArmors()[0].Center().x >= outpost_.center_point_.x) {
                outpost_.going_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.coming_center_ = cv::Point2f(0, 0);
            }
            if (outpost_.BottomArmors()[0].Center().x < outpost_.center_point_.x) {
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_ = cv::Point2f(0, 0);
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
    DLOG(INFO) << "nums of armor" << outpost_.BottomArmors().size();
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
        DLOG(INFO) << "Outpost clockwise something wrong";
    if (clockwise_ > 7) {
        DLOG(INFO) << "Outpost is Clockwise";
        clockwise_ = 1;
        checked_clockwise_ = true;
    } else if (clockwise_ < -7) {
        DLOG(INFO) << "Outpost is anti-Clockwise";
        clockwise_ = -1;
        checked_clockwise_ = true;
    }
}

cv::Rect OutpostPredictor::GetROI(const cv::Mat &src_image) {
    cv::Rect roi_rect = {};
    if (outpost_.BottomArmors().empty()) {
        ++roi_buff_;
        if (roi_buff_ > 30) {
            return roi_rect;
        }
    } else roi_buff_ = 0;
    int width = abs(roi_corners_[0].x - roi_corners_[1].x) < abs(roi_corners_[1].x - roi_corners_[2].x) ?
                int(abs(roi_corners_[1].x - roi_corners_[2].x)) : int(abs(roi_corners_[0].x - roi_corners_[1].x));
    int height = abs(roi_corners_[0].y - roi_corners_[1].y) < abs(roi_corners_[1].y - roi_corners_[2].y) ?
                 int(abs(roi_corners_[1].y - roi_corners_[2].y)) : int(abs(roi_corners_[0].y - roi_corners_[1].y));
    int x = int(cv::min(cv::min(roi_corners_[0].x, roi_corners_[1].x), cv::min(roi_corners_[2].x, roi_corners_[3].x)));
    int y = int(cv::min(cv::min(roi_corners_[0].y, roi_corners_[1].y), cv::min(roi_corners_[2].y, roi_corners_[3].y)));
    cv::Rect target_right{x, y, width, height};
    auto zoom_size = cv::Size(target_right.width * kZoomRatio.width,
                              target_right.height * kZoomRatio.height);
    roi_rect = target_right + zoom_size;
    roi_rect -= cv::Point((zoom_size.width >> 1), (zoom_size.height >> 1));
    roi_rect = roi_rect & cv::Rect(0, 0, src_image.cols, src_image.rows);
    return roi_rect;

}

void OutpostPredictor::UpdateROICorners(const Armor &armor) {
    for (int i = 0; i < 4; i++) {
        roi_corners_[i] = armor.Corners()[i];
    }
}

double OutpostPredictor::GetBulletFlyTime(const float &bullet_speed, const Armor &armor) {
    auto f = trajectory_solver::AirResistanceModel();
    f.SetParam(0.26, 994, 32, 0.0425, 0.041);
    auto a = trajectory_solver::BallisticModel();
    a.SetParam(f, 31);
    auto solver = trajectory_solver::PitchAngleSolver();
    auto target_x = sqrt(armor.TranslationVectorWorld()[0] * armor.TranslationVectorWorld()[0] +
                         armor.TranslationVectorWorld()[2] * armor.TranslationVectorWorld()[2]);
    solver.SetParam(a, 16, 0.45, 1.3, 7);
    solver.UpdateParam(1.3, 7);
    auto res = solver.Solve(-CV_PI / 6, CV_PI / 3, 0.01, 16);
    return res.y();
}

SendPacket OutpostPredictor::NewRun(Battlefield battlefield, const float &bullet_speed, int width) {
    outpost_.ClearBottomArmor();
    SendPacket send_packet{0, 0, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0, 0};

    send_packet.distance_mode = 10;
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
    if (outpost_.BottomArmors().empty()) return send_packet;


    if (clockwise_ <= 7 && clockwise_ >= -7 && !checked_clockwise_)
        IsClockwise();
    else
        DecideComingGoing();


    int biggest_armor = FindBiggestArmor(outpost_.BottomArmors());
    UpdateROICorners(outpost_.BottomArmors()[biggest_armor]);
    DLOG(INFO) << "bulletFlyTime" << GetBulletFlyTime(bullet_speed, outpost_.BottomArmors()[biggest_armor]);

    auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(
            outpost_.BottomArmors()[biggest_armor].TranslationVectorCam());
    send_packet.yaw = 0, send_packet.pitch = shoot_point_spherical(1, 0);
    DLOG(INFO) << "outpost distance： " << outpost_.BottomArmors()[biggest_armor].Distance();
    DLOG(INFO) << "armor length： " << norm(outpost_.BottomArmors()[biggest_armor].Corners()[0] -
                                           outpost_.BottomArmors()[biggest_armor].Corners()[1]);

    int mid_x = width / 2;

    if (!ready_fire_ && ((abs(mid_x - outpost_.coming_center_.x) < 15) || abs(mid_x - outpost_.going_center_.x) < 15)) {
        ready_time_ = std::chrono::high_resolution_clock::now();
        ready_fire_ = true;
    }
    if (ready_fire_) {
        auto current_time = std::chrono::high_resolution_clock::now();
        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time -
                                                                                  ready_time_)).count();
        auto bullet_fly_time = GetBulletFlyTime(bullet_speed, outpost_.BottomArmors()[biggest_armor]);
        shoot_delay_time_ = 5.0 / 6.0 - bullet_fly_time;
        if (shoot_delay_time_ < time_gap * 1e-3) {
            send_packet.fire = 1;
            ready_fire_ = false;
        }
    }

    send_packet.check_sum = send_packet.yaw + send_packet.pitch + send_packet.delay +
                            float(send_packet.fire) + float(send_packet.distance_mode) +
                            float(send_packet.point1_x) + float(send_packet.point1_y) +
                            float(send_packet.point2_x) + float(send_packet.point2_y) +
                            float(send_packet.point3_x) + float(send_packet.point3_y) +
                            float(send_packet.point4_x) + float(send_packet.point4_y);
    return send_packet;
}

SendPacket OutpostPredictor::Run(Battlefield battlefield, const float &bullet_speed, cv::MatSize frame_size, int time) {
    SendPacket send_packet;
    if (time < 60 * 3)
        send_packet = NewRun(battlefield, bullet_speed, frame_size().width);
    else
        send_packet = OldRun(battlefield);
    return send_packet;
}



