//
// Created by jonas on 5/2/22.
//
#include "detector_outpost.h"

#include <utility>
void SendToOutpostPredictor::UpdateInfo(cv::Point2f going_center, cv::Point2f center, cv::Point2f coming_center,
                                        int clockwise, double distance, float bulletspeed, coordinate::TranslationVector shootpoint)
{
    going_center_point = std::move(going_center);
    coming_center_point = std::move(coming_center);
    outpost_center = std::move(center);
    is_clockwise = clockwise;
    center_distance = distance;
    bullet_speed  = bulletspeed;
    shoot_point = std::move(shootpoint);
}


OutpostDetector::OutpostDetector():outpost_center_(320, 240), max_area_(0.0), last_armor_x_(0.0){}

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
    auto a = battlefield.Robots();
//    auto facility = battlefield.Facilities();



    if (a[color_][Robot::kInfantry4] == nullptr)
    {
        DLOG(INFO) << "No outpost armor founded";

        send_to_outpost_predictor.UpdateInfo(going_center_point_, outpost_center_, coming_center_point_,
                                             clockwise_, center_distance_, battlefield.BulletSpeed(), shoot_point_);
        return send_to_outpost_predictor;
    }
//    if (a[color_][Facility::kOutpost] == nullptr)
//    {
//        DLOG(INFO) << "No outpost armor founded";
//        return;
//    }

    detected_armors_in_this_frame_ = a[color_][Robot::kInfantry4]->Armors();
//    detected_armors_in_this_frame_ = a[color_][Facility::kOutpost]->BottomArmors();



    if(clockwise_<= 7 && clockwise_ >= -7 && !is_checked_clockwise)
        IsClockwise();
    else
        DecideComingGoing();
    FindBiggestArmor();

    send_to_outpost_predictor.UpdateInfo(going_center_point_, outpost_center_, coming_center_point_,
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
    double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(
            current_time_chrono - start_time_)).count();

    DLOG(INFO) << "time_gap: " << time_gap/1000;

    for(auto & armor : detected_armors_in_this_frame_)
        if(armor.Area() > max_area_)
        {
            max_area_ = armor.Area();
            if(time_gap/1000 < 0.85)
            {
                outpost_center_ = armor.Center();
                center_distance_ = armor.Distance();
                shoot_point_ = armor.TranslationVectorWorld();
            }
            else
            {
                DLOG(WARNING) << "outpost center founded";
                if(abs(outpost_center_.y - armor.Center().y) > kVertical_threshold_)
                    outdated_ = true;
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
                going_center_point_ = detected_armors_in_this_frame_[0].Center();
                coming_center_point_ = cv::Point2f (-1,-1);
            }
            if(detected_armors_in_this_frame_[0].Center().x > outpost_center_.x)
                coming_center_point_ = detected_armors_in_this_frame_[0].Center();
        } else if(clockwise_ == -1)
        {
            if(detected_armors_in_this_frame_[0].Center().x >= outpost_center_.x)
            {
                going_center_point_ = detected_armors_in_this_frame_[0].Center();
                coming_center_point_ = cv::Point2f (-1,-1);
            }
            if(detected_armors_in_this_frame_[0].Center().x < outpost_center_.x)
                coming_center_point_ = detected_armors_in_this_frame_[0].Center();
        }
    }
    else
    {
        if(clockwise_ == 1)
        {
            if(detected_armors_in_this_frame_[0].Center().x > detected_armors_in_this_frame_[1].Center().x)
            {
                coming_center_point_ = detected_armors_in_this_frame_[0].Center();
                going_center_point_ = detected_armors_in_this_frame_[1].Center();
            } else
            {
                coming_center_point_ = detected_armors_in_this_frame_[1].Center();
                going_center_point_ = detected_armors_in_this_frame_[0].Center();
            }
        } else if(clockwise_ == -1)
        {
            if(detected_armors_in_this_frame_[0].Center().x > detected_armors_in_this_frame_[1].Center().x)
            {
                coming_center_point_ = detected_armors_in_this_frame_[1].Center();
                going_center_point_ = detected_armors_in_this_frame_[0].Center();
            } else
            {
                coming_center_point_ = detected_armors_in_this_frame_[0].Center();
                going_center_point_ = detected_armors_in_this_frame_[1].Center();
            }
        }
    }
}