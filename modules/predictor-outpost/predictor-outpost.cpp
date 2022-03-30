//
// Created by screw on 2022/3/13.
//

#include "predictor-outpost.h"

double SortTarget(const std::vector<Armor>& target)
{
    double temp;
    for(const Armor& armor : target)
    {
        if(armor.TranslationVectorWorld().x() < temp)
            temp = armor.TranslationVectorWorld().x();
    }
    return temp;
}

SendPacket OutpostPredictor::Run(const Battlefield &battlefield) {
    if(battlefield.Facilities().count(color_) == 0)
        std::cout << "Not Found" << std::endl;

    // Filter robot armor and add potential into target.
    std::vector<Armor> potential_target = battlefield.Facilities().at(color_).at(kType_)->BottomArmors();
    target_ = TargetArmorFilter(potential_target);

    int num = 0;
    if(is_clockwise == 0 || num < kCollectNum)
    {
        double minimum_x = SortTarget(target_);   // TODO Current method is not 100% safe.
        clockwise_assist = minimum_x - clockwise_assist; // Origin is 0.
        if (clockwise_assist < 0)
            is_clockwise = -1;
        else
            is_clockwise =  1;
        ++num;
    }
    else
    {
        // Detect 1 armor.
        if(target_.size() == 1)
        {
            coordinate::TranslationVector moving_center_ = coordinate::TranslationVector(target_[0].TranslationVectorWorld().x(), target_[0].TranslationVectorWorld().y(), target_[0].TranslationVectorWorld().z());
            target_moving_center_ = CalculateMovingTargetCenter(moving_center_, 1);
            predict_shoot_center_ = CalculatePredictShootCenter(target_moving_center_, battlefield.BulletSpeed(), battlefield.YawPitchRoll(), target_[0].Distance());

        }
            // Detect 2 armors.
        else if(target_.size() == 2)
        {
            DecideGoingAndComing(target_);
            target_moving_center_ = CalculateMovingTargetCenter(coming_armor_.TranslationVectorWorld(), 2);
            predict_shoot_center_ = CalculatePredictShootCenter(target_moving_center_, battlefield.BulletSpeed(), battlefield.YawPitchRoll(), going_armor_.Distance());
        }
        else if(target_.empty())
        {
            std::cout << "Outpost armor not Found" << std::endl;
        }
        else if(target_.size() > 2)
        {
            std::cout << "Outpost armor over Found" << std::endl;
        }
    }
    return send_packet;
}


std::vector<Armor> OutpostPredictor::TargetArmorFilter(const std::vector<Armor> &potential_target)
{
    std::vector<Armor> target;
    for(auto & it : potential_target)
    {
        if (it.ID() < 0 and it.ID() > 7)  // Get rid of robot armors.
        {
            target.push_back(it);
        }
    }
    return target;
}

coordinate::TranslationVector OutpostPredictor::CalculateMovingTargetCenter(const coordinate::TranslationVector & moving_center, const int detect_num)
{
    coordinate::TranslationVector target_moving_center;
    if (detect_num == 1)
    {
        // Shoot the armor being to appear.
        // x1 = (xcosa - ysina); y1 = (ycosa + xsina);
        target_moving_center.x() = moving_center.x() * std::cos(is_clockwise * kArmor_included_rad_) - moving_center.y() * std::sin(is_clockwise * kArmor_included_rad_);
        target_moving_center.y() = moving_center.y() * std::cos(is_clockwise * kArmor_included_rad_) + moving_center.x() * std::sin(is_clockwise * kArmor_included_rad_);
        target_moving_center.z() = moving_center.z();
    }
    else if(detect_num == 2)
    {
        // Shoot the coming armor.
        target_moving_center.x() = moving_center.x();
        target_moving_center.y() = moving_center.y();
        target_moving_center.z() = moving_center.z();
    }
    return target_moving_center;
}

coordinate::TranslationVector OutpostPredictor::CalculatePredictShootCenter(const coordinate::TranslationVector & moving_center,
                                                                            float bullet_speed,
                                                                            const float yaw_pitch_roll[],
                                                                            float distance)
{
    double tm_cam_to_imu_data[] = {0, -0.026, -0.075};  // TODO
    const static coordinate::TranslationMatrix camera_to_imu_translation_matrix(tm_cam_to_imu_data);
    coordinate::TranslationVector predict_shoot_center;
    shoot_delay_ = distance / bullet_speed;
    float rotate_angle = kControl_delay_ * shoot_delay_ * kRotate_angular_speed_;

    predict_shoot_center.x() = moving_center.x() * std::cos((-1.0) * is_clockwise * rotate_angle) - moving_center.y() * std::sin((-1.0) * is_clockwise * rotate_angle);
    predict_shoot_center.y() = moving_center.y() * std::cos((-1.0) * is_clockwise * rotate_angle) + moving_center.x() * std::sin((-1.0) * is_clockwise * rotate_angle);
    predict_shoot_center.z() = moving_center.z();

    predict_shoot_center = coordinate::transform::WorldToCamera(predict_shoot_center,
                                                                coordinate::transform::EulerAngleToRotationMatrix(
                                                                        yaw_pitch_roll),
                                                              camera_to_imu_translation_matrix,    // TODO
                                                              Eigen::Matrix3d::Identity());
    return predict_shoot_center;
}

void OutpostPredictor::DecideGoingAndComing(const std::vector<Armor> & target)
{
    going_armor_ = target[0];
    if(target[0].Center().x < target[1].Center().x)
    {
        coming_armor_ = target[0];
        going_armor_  = target[1];
    }
    else
    {
        coming_armor_ = target[1];
        going_armor_  = target[0];
    }
}