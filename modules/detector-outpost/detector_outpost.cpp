//
// Created by jonas on 5/2/22.
//
#include "detector_outpost.h"

#include <utility>
void SendToOutpostPredictor::UpdateInfo(cv::Point2f going_center_2d, cv::Point2f center, cv::Point2f coming_center_2d,
                                        coordinate::TranslationVector going_center_3d, coordinate::TranslationVector center_3d,coordinate::TranslationVector coming_center_3d,
                                        int clockwise, double distance, float bulletspeed, coordinate::TranslationVector shootpoint)
{
    going_center_point2d = std::move(going_center_2d);
    coming_center_point2d = std::move(coming_center_2d);
    going_center_point3d = std::move(going_center_3d);
    coming_center_point3d = std::move(coming_center_3d);
    outpost_center = std::move(center);
    outpost_center_3d = std::move(center_3d);
    is_clockwise = clockwise;
    center_distance = distance;
    bullet_speed  = bulletspeed;
    shoot_point = std::move(shootpoint);
}


OutpostDetector::OutpostDetector():
        outpost_center_(320, 240), max_area_(0.0),
        last_armor_x_(0.0),
        going_center_point_2D(0.0, 0.0),
        coming_center_point_2D(0.0, 0.0),
        going_center_point_3D(0,0,0),
        coming_center_point_3D(0,0,0),
        shoot_point_(0,0,0),
        center_distance_(0){}

bool OutpostDetector::Initialize(const std::string &config_path) {
//    cv::FileStorage config;
    // Open config file.
//    config.open(config_path, cv::FileStorage::READ);
//    if (!config.isOpened()) {
//        LOG(ERROR) << "Failed to open outpost config file " << config_path << ".";
//        return false;
//    }
    return true;
}


SendToOutpostPredictor OutpostDetector::Run(const Battlefield& battlefield)
{
    SendToOutpostPredictor send_to_outpost_predictor;
    detected_armors_in_this_frame_.clear();
    auto facilities = battlefield.Facilities();
    auto robots = battlefield.Robots();


//    for(auto &robot : robots[color_]){
//        for( auto &armor:robot.second->Armors())
//            detected_armors_in_this_frame_.emplace_back(armor);
//    }
//    for(auto &facility : facilities[color_]){
//        for( auto &armor: facility.second->BottomArmors())
//            detected_armors_in_this_frame_.emplace_back(armor);
//    }

    //if (facility[color_][Facility::kBase] == nullptr)
    if (robots[color_][Robot::kInfantry4] == nullptr)
    {
        DLOG(INFO) << "No outpost armor founded";

        send_to_outpost_predictor.UpdateInfo(going_center_point_2D, outpost_center_, coming_center_point_2D,
                                             going_center_point_3D, center_3D,coming_center_point_3D,
                                             clockwise_, center_distance_, battlefield.BulletSpeed(), shoot_point_);
        return send_to_outpost_predictor;
    }


    // detected_armors_in_this_frame_ = facility[color_][Facility::kBase]->BottomArmors();
    detected_armors_in_this_frame_ = robots[color_][Robot::kInfantry4]->Armors();

    if(clockwise_<= 7 && clockwise_ >= -7 && !is_checked_clockwise)
        IsClockwise();
    else
        DecideComingGoing();
    FindBiggestArmor();

    send_to_outpost_predictor.UpdateInfo(going_center_point_2D, outpost_center_, coming_center_point_2D,
                                         going_center_point_3D, center_3D,coming_center_point_3D,
                                         clockwise_, center_distance_, battlefield.BulletSpeed(), shoot_point_);
    return send_to_outpost_predictor;
}

void OutpostDetector::IsClockwise()
{
    LOG(INFO) << "nums of armor" << detected_armors_in_this_frame_.size();
    double this_armor_x;
    if(detected_armors_in_this_frame_.size() == 1)
        this_armor_x = detected_armors_in_this_frame_[0].Center().x;
    else
    {
        for(int i = 0; i < detected_armors_in_this_frame_.size() - 1; ++i)
            if(detected_armors_in_this_frame_[i].Center().x > detected_armors_in_this_frame_[i+1].Center().x)
                this_armor_x = detected_armors_in_this_frame_[i+1].Center().x;
    }
    double diff = this_armor_x - last_armor_x_;
    if(diff < 0)
        clockwise_ ++;
    else if(diff > 0)
        clockwise_ --;
    else
        LOG(INFO) << "Outpost clockwise something wrong";
    // 向右转逆，向左转顺
    if (clockwise_ > 7)
    {
        LOG(WARNING)<< "Outpost is Clockwise";
        clockwise_ = 1;
        is_checked_clockwise = true;
    }
    else if(clockwise_ < -7)
    {
        LOG(WARNING)<< "Outpost is anti-Clockwise";
        clockwise_ = -1;
        is_checked_clockwise = true;
    }
}

void OutpostDetector::FindBiggestArmor()
{
    if(outdated_)
    {
        start_time_ =  std::chrono::high_resolution_clock::now();
        outdated_ = false;
        return;
    }

    auto current_time_chrono = std::chrono::high_resolution_clock::now();
    double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time_chrono - start_time_)).count();

    DLOG(INFO) << "time_gap: " << time_gap/1000;

    for(auto & armor : detected_armors_in_this_frame_)
        if(armor.Area() > max_area_)
        {
            max_area_ = armor.Area();
            if(time_gap/1000 < 0.85)
            {
                outpost_center_ = armor.Center();
                center_distance_ = armor.Distance();
                shoot_point_ = armor.TranslationVectorCam();
                center_3D = armor.TranslationVectorWorld();

            }
            else
            {
                if(abs(outpost_center_.y - armor.Center().y) > kVertical_threshold_)
                {
                    DLOG(WARNING) << "outpost center error too much, finding again";
                    outdated_ = true;
                }
                else
                    DLOG(WARNING) << "outpost center founded";
            }
        }
}

void OutpostDetector::DecideComingGoing()
{
//    DLOG(INFO) << "nums of armor" << detected_armors_in_this_frame_.size();
    if(detected_armors_in_this_frame_.size() == 1)
    {
        if(clockwise_ == 1)
        {
            if(detected_armors_in_this_frame_[0].Center().x <= outpost_center_.x)
            {
                going_center_point_2D = detected_armors_in_this_frame_[0].Center();
                going_center_point_3D = detected_armors_in_this_frame_[0].TranslationVectorWorld();
                coming_center_point_2D = cv::Point2f (-1, -1);
                going_center_point_3D = Eigen::Vector3d (-1,-1,-1);
            }
            if(detected_armors_in_this_frame_[0].Center().x > outpost_center_.x){
                coming_center_point_2D = detected_armors_in_this_frame_[0].Center();
                coming_center_point_3D = detected_armors_in_this_frame_[0].TranslationVectorWorld();
            }
        } else if(clockwise_ == -1)
        {
            if(detected_armors_in_this_frame_[0].Center().x >= outpost_center_.x)
            {
                going_center_point_2D = detected_armors_in_this_frame_[0].Center();
                going_center_point_3D = detected_armors_in_this_frame_[0].TranslationVectorWorld();
                coming_center_point_2D = cv::Point2f (-1, -1);
                coming_center_point_3D = Eigen::Vector3d (-1 , -1 , -1);
            }
            if(detected_armors_in_this_frame_[0].Center().x < outpost_center_.x){
                coming_center_point_2D = detected_armors_in_this_frame_[0].Center();
                coming_center_point_3D = detected_armors_in_this_frame_[0].TranslationVectorWorld();
            }
        }
    }
    else
    {
        if(clockwise_ == 1)
        {
            if(detected_armors_in_this_frame_[0].Center().x > detected_armors_in_this_frame_[1].Center().x)
            {
                coming_center_point_2D = detected_armors_in_this_frame_[0].Center();
                coming_center_point_3D = detected_armors_in_this_frame_[0].TranslationVectorWorld();
                going_center_point_2D = detected_armors_in_this_frame_[1].Center();
                going_center_point_3D = detected_armors_in_this_frame_[1].TranslationVectorWorld();
            } else
            {
                coming_center_point_2D = detected_armors_in_this_frame_[1].Center();
                coming_center_point_3D = detected_armors_in_this_frame_[1].TranslationVectorWorld();
                going_center_point_2D = detected_armors_in_this_frame_[0].Center();
                going_center_point_3D = detected_armors_in_this_frame_[0].TranslationVectorWorld();
            }
        } else if(clockwise_ == -1)
        {
            if(detected_armors_in_this_frame_[0].Center().x > detected_armors_in_this_frame_[1].Center().x)
            {
                coming_center_point_2D = detected_armors_in_this_frame_[1].Center();
                coming_center_point_3D = detected_armors_in_this_frame_[1].TranslationVectorWorld();
                going_center_point_2D = detected_armors_in_this_frame_[0].Center();
                going_center_point_3D = detected_armors_in_this_frame_[0].TranslationVectorWorld();
            } else
            {
                coming_center_point_2D = detected_armors_in_this_frame_[0].Center();
                coming_center_point_3D = detected_armors_in_this_frame_[0].TranslationVectorWorld();
                going_center_point_2D = detected_armors_in_this_frame_[1].Center();
                going_center_point_3D = detected_armors_in_this_frame_[1].TranslationVectorWorld();
            }
        }
    }
}
