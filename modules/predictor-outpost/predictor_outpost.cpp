#include "predictor_outpost.h"

bool OutpostPredictor::Initialize() {
#if !NDEBUG
    OutpostPredictorDebug::Instance().addTrackbar();
#endif
    return true;
}


SendPacket OutpostPredictor::StaticOutpostRun(const Battlefield &battlefield, std::array<float, 3> yaw_pitch_roll) {
    /*
     * Initialize
     */
    outpost_.ClearBottomArmor();
    SendPacket send_packet{0, 0, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0};


    /*
     * 收集armors
     */
    auto facilities = battlefield.Facilities();
    auto robots = battlefield.Robots();
    for (auto &robot: robots[enemy_color_]) {
        for (auto &armor: robot.second->Armors()) {
            if (GetOutpostHeight(armor, yaw_pitch_roll[1]) > 0.5)
                outpost_.AddBottomArmor(armor);
        }
    }
    for (auto &facility: facilities[enemy_color_]) {
        for (auto &armor: facility.second->BottomArmors()) {
            if (GetOutpostHeight(armor, yaw_pitch_roll[1]) > 0.5)
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
            coordinate::transform::EulerAngleToRotationMatrix(yaw_pitch_roll),
            Eigen::Vector3d::Zero(),
            Eigen::Matrix3d::Identity());
    auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(
            shoot_point);
    send_packet.yaw = float(shoot_point_spherical(0, 0));
    send_packet.pitch = float(shoot_point_spherical(1, 0));


    /*
     * 根据平面距离画roi
     */
    auto distance = distance_filter_(outpost_.BottomArmors()[biggest_armor].Distance());
    auto plane_distance = Compensator::Instance().GetPlaneDistance(distance);
    GetROI(outpost_.BottomArmors()[biggest_armor], plane_distance);
    DLOG(INFO) << "distance" << distance;
    auto height = GetOutpostHeight(outpost_.BottomArmors()[biggest_armor], yaw_pitch_roll[1]);
    DLOG(INFO) << "outpost height" << height;

    if (send_packet.pitch != 0)
        send_packet.pitch -= 0.015;
//    Compensator::Instance().Offset(send_packet.pitch, send_packet.yaw, 16, send_packet.check_sum, distance,
//                                   AimModes::kOutPost);


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
OutpostPredictor::SpinningOutpostRun(const Battlefield &battlefield, const float &bullet_speed, int width,
                                     const std::array<float, 3> &yaw_pitch_roll,
                                     const std::chrono::steady_clock::time_point &time) {
    outpost_.ClearBottomArmor();
    SendPacket send_packet{0, 0, 0,
                           10, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0};


    /*
     * 收集armors
     */
    auto facilities = battlefield.Facilities();
    auto robots = battlefield.Robots();
    for (auto &robot: robots[enemy_color_]) {
        for (auto &armor: robot.second->Armors()) {
            if (GetOutpostHeight(armor, yaw_pitch_roll[1]) > kHeightThreshold)
                outpost_.AddBottomArmor(armor);
        }
    }
    for (auto &facility: facilities[enemy_color_]) {
        for (auto &armor: facility.second->BottomArmors()) {
            if (GetOutpostHeight(armor, yaw_pitch_roll[1]) > kHeightThreshold)
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
            coordinate::transform::EulerAngleToRotationMatrix(yaw_pitch_roll),
            Eigen::Vector3d::Zero(),
            Eigen::Matrix3d::Identity());
    auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(
            shoot_point);
    send_packet.yaw = float(shoot_point_spherical(0, 0));
    send_packet.yaw = 0;
    send_packet.pitch = float(shoot_point_spherical(1, 0));


    /*
     * 根据平面距离画roi
     */
    auto distance = distance_filter_(outpost_.BottomArmors()[biggest_armor].Distance());
    auto plane_distance = Compensator::Instance().GetPlaneDistance(distance);
    auto height = GetOutpostHeight(outpost_.BottomArmors()[biggest_armor], yaw_pitch_roll[1]);
    DLOG(INFO) << "distance" << distance;
    DLOG(INFO) << "outpost height" << height;
    GetROI(outpost_.BottomArmors()[biggest_armor], plane_distance);


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
        shoot_delay_time_ = GetShootDelay(3);
        LOG(INFO) << "shoot_delay_time_" << shoot_delay_time_;
        if (shoot_delay_time_ < time_gap * 1e-3) {
            send_packet.fire = 1;
            ready_fire_ = false;
        }
    }


    /*
     * 补偿
     */
    Offset(send_packet.pitch, distance);
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
    auto height = armor.Distance() *
                  sin(pitch + coordinate::convert::Rectangular2Spherical(
                          armor.TranslationVectorCam())[1]  + 0.104);
    LOG(INFO) << "Armor Height" << height;
    return height;
}

double OutpostPredictor::GetShootDelay(const double &plane_distance) {
    if (plane_distance <= 4.7)
        return OutpostPredictorDebug::Instance().ShootDelay3M();
    else if (4.7 < plane_distance && plane_distance <= 5.5)
        return OutpostPredictorDebug::Instance().ShootDelay5M();
    else if (plane_distance > 5.5)
        return OutpostPredictorDebug::Instance().ShootDelay6M();
}

void OutpostPredictor::Offset(float &pitch, double distance) {
    if (pitch == 0) return;
    if (distance <= 4.7)
        pitch += 0.01;
    else if (4.7 < distance && distance <= 5.5)
        pitch -= 0.015;
    else if (distance > 5.5)
        pitch -= OutpostPredictorDebug::Instance().DeltaPitchUp();
}







//SendPacket OutpostPredictor::Run(Battlefield battlefield, const float &bullet_speed, cv::MatSize frame_size,
//                                 const float &real_pitch,
//                                 const std::chrono::steady_clock::time_point &time) {
//    SendPacket send_packet;
//    send_packet = NewRun(battlefield, bullet_speed, frame_size().width, real_pitch, time);
//    return send_packet;
//}



