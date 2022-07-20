#include "predictor_outpost.h"

const cv::Size kZoomRatio = {18, 60};

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

void OutpostPredictor::GetROI(const Armor &armor, const double &plane_distance) {
    int length = 600, weight = 800;
    if (plane_distance < 5) {
        length = 600;
        weight = 800;
    } else if (5 < plane_distance && plane_distance < 6) {
        length = 480;
        weight = 640;
    } else if (6 < plane_distance) {
        length = 360;
        weight = 480;
    }
    roi_rect = {int(armor.Center().x - length * 0.5), int(armor.Center().y - weight * 0.3), length, weight};

}

void OutpostPredictor::UpdateROICorners(const Armor &armor) {
    for (int i = 0; i < 4; i++) {
        roi_corners_[i] = armor.Corners()[i];
    }
}

Eigen::Vector3d OutpostPredictor::GetPitchFlyTime(const float &bullet_speed, const Armor &armor) {
    auto f = trajectory_solver::AirResistanceModel();
    f.SetParam(0.26, 994, 32, 0.0425, 0.041);
    auto a = trajectory_solver::BallisticModel();
    a.SetParam(f, 31);
    auto solver = trajectory_solver::PitchAngleSolver();
    solver.SetParam(a, 16, 0.45, 1.3, 4.5);
    solver.UpdateParam(1.3, 4.5);
    auto res = solver.Solve(-CV_PI / 6, CV_PI / 3, 0.01, 16);
    return res;
}

bool ThrowLine(const std::vector<Armor> &armors, double mid_x) {
    for (auto &armor: armors) {
        if ((armor.Corners()[0].x < mid_x && mid_x < armor.Corners()[1].x) ||
            (armor.Corners()[1].x < mid_x && mid_x < armor.Corners()[0].x) ||
            (armor.Corners()[1].x < mid_x && mid_x < armor.Corners()[2].x) ||
            (armor.Corners()[2].x < mid_x && mid_x < armor.Corners()[1].x))
//        if (abs(armor.Center().x - mid_x) < 13)
            return true;
    }
    return false;
}


SendPacket
OutpostPredictor::NewRun(Battlefield battlefield, const float &bullet_speed, int width,
                         const std::array<float, 3> e_yaw_pitch_roll,
                         const std::chrono::steady_clock::time_point &time) {
    outpost_.ClearBottomArmor();
    SendPacket send_packet{0, 0, 0,
                           10, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0, 10};


    /*
     * 收集armors
     */
    auto facilities = battlefield.Facilities();
    auto robots = battlefield.Robots();
    for (auto &robot: robots[enemy_color_]) {
        for (auto &armor: robot.second->Armors()) {
            if (GetOutpostHeight(armor, e_yaw_pitch_roll[1]) > 0.5)
                outpost_.AddBottomArmor(armor);
        }
    }
    for (auto &facility: facilities[enemy_color_]) {
        for (auto &armor: facility.second->BottomArmors()) {
            if (GetOutpostHeight(armor, e_yaw_pitch_roll[1]) > 0.5)
                outpost_.AddBottomArmor(armor);
        }
    }


    /*
     * 15帧未有目标再更新roi
     */
    if (outpost_.BottomArmors().empty()) {
        ++roi_buff_;
        if (roi_buff_ > 15)
            roi_rect = {};
        return send_packet;
    }

    /*
     * 寻找最大目标计算pitch
     * yaw由操作手控制
     */
    int biggest_armor = FindBiggestArmor(outpost_.BottomArmors());
    auto shoot_point = coordinate::transform::WorldToCamera(
            outpost_.BottomArmors()[biggest_armor].TranslationVectorWorld(),
            coordinate::transform::EulerAngleToRotationMatrix(e_yaw_pitch_roll),
            Eigen::Vector3d::Zero(),
            Eigen::Matrix3d::Identity());
    auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(
            shoot_point);
    send_packet.yaw = shoot_point_spherical(0, 0);
    send_packet.yaw = 0;
    send_packet.pitch = shoot_point_spherical(1, 0);


    /*
     * 根据平面距离画roi
     */
    auto distance = distance_filter_(outpost_.BottomArmors()[biggest_armor].Distance());
    GetROI(outpost_.BottomArmors()[biggest_armor], Compensator::Instance().GetPlaneDistance(distance));
    DLOG(INFO) << "distance" << distance;
    auto height = GetOutpostHeight(outpost_.BottomArmors()[biggest_armor], e_yaw_pitch_roll[1]);
    DLOG(INFO) << "outpost height" << height;



// 测补偿
//    send_packet.distance_mode = 0;
//    if (send_packet.yaw != 0 && send_packet.pitch != 0) {
//        send_packet.pitch = send_packet.pitch + OutpostPredictorDebug::Instance().DeltaPitchDown() -
//                            OutpostPredictorDebug::Instance().DeltaPitchUp();
//        send_packet.yaw = send_packet.yaw + OutpostPredictorDebug::Instance().DeltaYawRight() -
//                          OutpostPredictorDebug::Instance().DeltaYawLeft();
//        send_packet.pitch = -Compensator::Instance().PitchOffset(send_packet.pitch,16,7,AimModes::kOutPost);
//    }



//    DLOG(INFO) << "outpost distance： " << outpost_.BottomArmors()[biggest_armor].Distance();
//    DLOG(INFO) << "armor length： " << norm(outpost_.BottomArmors()[biggest_armor].Corners()[0] -
//                                           outpost_.BottomArmors()[biggest_armor].Corners()[1]);

    /*
     * 在目标装甲板穿过线时read_fire_
     * 之后根据射击延迟射击
     */
    int mid_x = width / 2;
    if (!ready_fire_ &&
        ThrowLine(outpost_.BottomArmors(), mid_x)) {
        ready_time_ = std::chrono::high_resolution_clock::now();
        ready_fire_ = true;
    }
    if (ready_fire_) {
        auto current_time = std::chrono::high_resolution_clock::now();
        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time -
                                                                                  ready_time_)).count();
        shoot_delay_time_ = OutpostPredictorDebug::Instance().ShootDelay();
        LOG(INFO) << "shoot_delay_time_" << shoot_delay_time_;
        if (shoot_delay_time_ < time_gap * 1e-3) {
            send_packet.fire = 1;
            ready_fire_ = false;
        }
    }

    if (send_packet.pitch != 0)
        send_packet.pitch -= 0.015;
//    Compensator::Instance().Offset(send_packet.pitch, send_packet.yaw, 16, send_packet.check_sum, distance,
//                                   AimModes::kOutPost);

    fire_ = send_packet.fire;   // fire_只用于图像显示
    send_packet.check_sum = send_packet.yaw + send_packet.pitch + send_packet.delay +
                            float(send_packet.fire) + float(send_packet.distance_mode) +
                            float(send_packet.point1_x) + float(send_packet.point1_y) +
                            float(send_packet.point2_x) + float(send_packet.point2_y) +
                            float(send_packet.point3_x) + float(send_packet.point3_y) +
                            float(send_packet.point4_x) + float(send_packet.point4_y);
    return send_packet;
}

cv::Rect OutpostPredictor::GetROI(const cv::Mat &src_image) {
    cv::Rect roi;
    roi = roi_rect & cv::Rect(0, 0, src_image.cols, src_image.rows);
    return roi;
}

double OutpostPredictor::GetOutpostHeight(const Armor &armor, const float &pitch) {
    auto height = Compensator::Instance().GetPlaneDistance(armor.Distance()) *
                  tan(pitch + coordinate::convert::Rectangular2Spherical(
                          armor.TranslationVectorCam())[0] + 0.104);
    LOG(INFO) << "Outpost Height" << height;
    return height;
}



//SendPacket OutpostPredictor::Run(Battlefield battlefield, const float &bullet_speed, cv::MatSize frame_size,
//                                 const float &real_pitch,
//                                 const std::chrono::steady_clock::time_point &time) {
//    SendPacket send_packet;
//    send_packet = NewRun(battlefield, bullet_speed, frame_size().width, real_pitch, time);
//    return send_packet;
//}



