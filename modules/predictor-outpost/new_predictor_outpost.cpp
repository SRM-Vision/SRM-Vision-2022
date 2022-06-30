#include "new_predictor_outpost.h"
void NewOutputData::Update(const coordinate::TranslationVector &shoot_point)
{
    auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(shoot_point);
    yaw = shoot_point_spherical(0,0),pitch = shoot_point_spherical(1,0);

//    DLOG(INFO) << "yaw, pitch: " << float(yaw) << " " << float(pitch);
}

void NewOutpostPredictor::GetFromDetector(SendToOutpostPredictor send_to_outpost_predictor)
{
    clockwise_= send_to_outpost_predictor.is_clockwise;
    outpost_center_ = std::move(send_to_outpost_predictor.outpost_center);
    going_center2d_ = std::move(send_to_outpost_predictor.going_center_point2d);
    going_center3d_ = std::move(send_to_outpost_predictor.going_center_point3d);
    coming_center2d_ = std::move(send_to_outpost_predictor.coming_center_point2d);
    coming_center3d_ = std::move(send_to_outpost_predictor.coming_center_point3d);
    center_distance_ = send_to_outpost_predictor.center_distance;
    bullet_speed_ = send_to_outpost_predictor.bullet_speed;
    shoot_point = send_to_outpost_predictor.shoot_point;
}

SendPacket NewOutpostPredictor::Run()
{
    //TODO
}
