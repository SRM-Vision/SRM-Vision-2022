#include "predictor-outpost.h"

void OutputData::Update(const coordinate::TranslationVector &shoot_point)  //TODO Unsure
{
    auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(shoot_point);
    yaw = shoot_point_spherical(0,0),pitch = shoot_point_spherical(1,0);

    DLOG(INFO) << "yaw, pitch: " << float(yaw) << " " << float(pitch);
}

SendPacket OutpostPredictor::Run()
{
    DLOG(INFO) << "Outpost_center_distance: " << center_distance_ << " | " << "bullet_speed: " << bullet_speed_;

    double time_delay = center_distance_ / bullet_speed_ + kCommunicationTime_;
    double pixel_distance = algorithm::SqrtFloat((coming_center_.x - outpost_center_.x) * (coming_center_.x - outpost_center_.x) +
            (coming_center_.y - outpost_center_.y) * (coming_center_.y - outpost_center_.y));


//    DLOG(INFO) << "shoot_point: " << shoot_point;
    output_data_.Update(shoot_point);
    return {output_data_.yaw, output_data_.pitch, output_data_.delay,
            0,output_data_.fire, float(output_data_.yaw + output_data_.pitch + 0 + output_data_.delay + output_data_.fire)};
}