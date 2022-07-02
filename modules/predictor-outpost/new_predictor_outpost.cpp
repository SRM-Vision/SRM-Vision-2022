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
    shoot_point_ = send_to_outpost_predictor.shoot_point;
    center_3d_ = send_to_outpost_predictor.outpost_center_3d;
}

SendPacket NewOutpostPredictor::Run()
{
    output_data_.fire = 0;
    if (coming_center3d_[0] != -1 && coming_center3d_[1] != -1 && coming_center3d_[2] != -1 ){
        double advanced_dis;            // TODO 设一个合理的值

        Eigen::Vector3d xyz ={center_3d_[0],center_3d_[1],center_3d_[2]};
        Eigen::Vector3d ypd;
        ypd = coordinate::convert::Rectangular2Spherical(xyz);
        double pitch = ypd[1];

        // 求根公式求解子弹飞行时间
        double a = 9.8 * 9.8 * 0.25;
        double b = bullet_speed_ * bullet_speed_ + cos(M_PI_2 + pitch) * bullet_speed_ *9.8;
        double c = - center_distance_* center_distance_;
        double b2_4ac = b * b - 4 * a * c;
        if (b2_4ac > 0){
            double t2 = (- b+sqrt(b2_4ac) ) / (2.0 * a);
            if (t2 > 0 ){
                double t = sqrt(t2);
                advanced_dis = 0.2765 * sin( t * 0.8 * M_PI * 0.5 ) * 2;       // 0.2765为半径
            }
        }

        double distance_between = (center_3d_ - coming_center3d_).norm();
        DLOG(INFO) << "distance_between" << distance_between;
        if(distance_between >  advanced_dis && distance_between < advanced_dis+0.01)
            output_data_.fire = 1;

        DLOG(INFO) << "advanced_dis: " << advanced_dis;
        DLOG(INFO) << "xyz" << xyz;
    }

    output_data_.Update(shoot_point_);

//    cv::Mat3d camera_mat;
//    camera_mat <<   859.7363,   0,  0,
//                    -0.7875,    862.3096,0,
//                    950.8627,   567.8418,1;
//    auto show_point = coordinate::transform::CameraToPicture(camera_mat, shoot_point);
//    auto point1_x = short(show_point.x);
//    auto point1_y = short(show_point.y);

    return {output_data_.yaw, output_data_.pitch - float(delta_pitch_),
            output_data_.delay,0,output_data_.fire,
            0,0,
            0,0,
            0,0,
            0,0,
            float(output_data_.yaw + output_data_.pitch + 0 + output_data_.delay + output_data_.fire)};
}
