#include "predictor-outpost.h"
void OutputData::Update(const coordinate::TranslationVector &shoot_point)  //TODO Unsure
{
    auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(shoot_point);
    yaw = shoot_point_spherical(0,0),pitch = shoot_point_spherical(1,0);

//    DLOG(INFO) << "yaw, pitch: " << float(yaw) << " " << float(pitch);
}

void OutpostPredictor::GetFromDetector(SendToOutpostPredictor send_to_outpost_predictor)
{
    clockwise_= send_to_outpost_predictor.is_clockwise;
            outpost_center_ = std::move(send_to_outpost_predictor.outpost_center);
            going_center_ = std::move(send_to_outpost_predictor.going_center_point);
            coming_center_ = std::move(send_to_outpost_predictor.coming_center_point);
            center_distance_ = send_to_outpost_predictor.center_distance;
            bullet_speed_ = send_to_outpost_predictor.bullet_speed;
            shoot_point = send_to_outpost_predictor.shoot_point;
}

SendPacket OutpostPredictor::Run()
{
    DLOG(INFO) << "Outpost_center_distance: " << center_distance_ << " | " << "bullet_speed: " << bullet_speed_;
    output_data_.fire = 0;
//    double time_delay = center_distance_ / bullet_speed_ + kCommunicationTime_;
    if (outpost_center_ != cv::Point2f(0.0, 0.0))
    {
        double pixel_distance = algorithm::SqrtFloat((going_center_.x - outpost_center_.x) * (going_center_.x - outpost_center_.x) +
                                                     (going_center_.y - outpost_center_.y) * (going_center_.y - outpost_center_.y));
        if(pixel_distance >  100 && pixel_distance < 110)
            output_data_.fire = 1;
        DLOG(INFO) << "pixel distance" << pixel_distance;
//    DLOG(INFO) << "shoot_point: " << shoot_point;
        output_data_.Update(shoot_point);
    }
    DLOG(INFO) << "Outpost Send packet: " << output_data_.yaw << " | " << output_data_.pitch - float(delta_pitch_) << " | " << output_data_.fire;

    return {output_data_.yaw, output_data_.pitch - float(delta_pitch_), output_data_.delay,
            0,output_data_.fire, 0,0,0,0,0,0,0,0,
            float(output_data_.yaw + output_data_.pitch + 0 + output_data_.delay + output_data_.fire)};
}