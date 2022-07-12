//
// Created by jonas on 5/2/22.
//
#include "detector_outpost.h"

#include <utility>
OutpostDataDetector::OutpostDataDetector():
        outpost_center_(320, 240), max_area_(0.0),
        last_armor_x_(0.0),
        shoot_point_(0,0,0),
        center_distance_(0){}

bool OutpostDataDetector::Initialize(const std::string &config_path) {
//    cv::FileStorage config;
    // Open config file.
//    config.open(config_path, cv::FileStorage::READ);
//    if (!config.isOpened()) {
//        LOG(ERROR) << "Failed to open outpost config file " << config_path << ".";
//        return false;
//    }
    return true;
}


DetectedData OutpostDataDetector::Run(const Battlefield& battlefield)
{
    detected_armors_in_this_frame_.clear();
    auto facilities = battlefield.Facilities();
    auto robots = battlefield.Robots();

    // TODO 高度判断
    for(auto &robot : robots[color_]){
        for( auto &armor:robot.second->Armors())
            detected_armors_in_this_frame_.emplace_back(armor);
    }
    for(auto &facility : facilities[color_]){
        for( auto &armor: facility.second->BottomArmors())
            detected_armors_in_this_frame_.emplace_back(armor);
    }

    // 显示所有目标
    for (const auto &armor: detected_armors_in_this_frame_) {
        debug::Painter::Instance()->DrawRotatedRectangle(armor.Corners()[0],
                                                         armor.Corners()[1],
                                                         armor.Corners()[2],
                                                         armor.Corners()[3],
                                                         cv::Scalar(0, 255, 0), 2);
        debug::Painter::Instance()->DrawText(std::to_string(armor.ID()), {200,200}, 255, 2);

    }


    if (detected_armors_in_this_frame_.empty())
    {
        DLOG(INFO) << "No outpost armor founded";
        ++disappear_buff_;
        if(disappear_buff_ > 30)
            Clear();
        return {detected_armors_in_this_frame_,
                outpost_center_, center_3D, shoot_point_, center_distance_,
                outpost_corner_[0], outpost_corner_[1],
                outpost_corner_[2], outpost_corner_[3],
                spinning_, false , going_armor_, coming_armor_, clockwise_};
    }

    disappear_buff_ = 0;
    if(detected_armors_in_this_frame_.size() == 1)
        spin_detector_.Update(detected_armors_in_this_frame_[0], battlefield.TimeStamp());
    else{
        double biggest_area = 0;
        int biggest_id = 0;
        for(int i = 0;i<detected_armors_in_this_frame_.size();i++)
            if(detected_armors_in_this_frame_[i].Area() > biggest_area){
                biggest_area = detected_armors_in_this_frame_[i].Area();
                biggest_id = i;
            }
        spin_detector_.Update(detected_armors_in_this_frame_[biggest_id], battlefield.TimeStamp());
    }

    if(spin_detector_.IsSpin()){
        if(clockwise_<= 7 && clockwise_ >= -7 && !is_checked_clockwise)
            IsClockwise();
        else
            DecideComingGoing();
        FindBiggestArmor();
    }

    return  {detected_armors_in_this_frame_, outpost_center_, center_3D, shoot_point_, center_distance_,
             outpost_corner_[0], outpost_corner_[1], outpost_corner_[2], outpost_corner_[3],
             spinning_, prepared_, coming_armor_, going_armor_, clockwise_};

}

void OutpostDataDetector::IsClockwise()
{
    DLOG(INFO) << "nums of armor" << detected_armors_in_this_frame_.size();
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
        DLOG(WARNING)<< "Outpost is anti-Clockwise";
        clockwise_ = -1;
        is_checked_clockwise = true;
    }
}

void OutpostDataDetector::FindBiggestArmor() {
    int biggest_id = -1;
    double max_area = 0;
    int i =0;
    for (i = 0; i < detected_armors_in_this_frame_.size(); i++)
        if (detected_armors_in_this_frame_[i].Area() > max_area) {
            max_area = detected_armors_in_this_frame_[i].Area();
            biggest_id = i;
            if(max_area>max_area_){
                max_area_ = detected_armors_in_this_frame_[biggest_id].Area();
                center_distance_ = detected_armors_in_this_frame_[biggest_id].Distance();
                outpost_center_ = detected_armors_in_this_frame_[biggest_id].Center();
                center_3D = detected_armors_in_this_frame_[biggest_id].TranslationVectorWorld();
                shoot_point_ = detected_armors_in_this_frame_[biggest_id].TranslationVectorCam();
                for(int j = 0;j<4;j++)
                    outpost_corner_[j] = detected_armors_in_this_frame_[biggest_id].Corners()[j];
                prepared_ = false;
                max_area_buff = 0;
            }
            else{
                ++max_area_buff;
                if(max_area_buff > 3)
                    prepared_ = true;
            }
        }
    if(norm(outpost_center_ - detected_armors_in_this_frame_[biggest_id].Center()) >
        norm(outpost_corner_[0]-outpost_corner_[1])*4.7)
    {
        max_area_ = 0;
        prepared_ = false;
    }
}
void OutpostDataDetector::DecideComingGoing()
{
//    DLOG(INFO) << "nums of armor" << detected_armors_in_this_frame_.size();
    going_armor_ = - 1;
    coming_armor_ = -1;
    if(detected_armors_in_this_frame_.size() == 1)
    {
        if(clockwise_ == 1)
        {
            if(detected_armors_in_this_frame_[0].Center().x <= outpost_center_.x)
            {
                going_armor_ = 0;
            }
            if(detected_armors_in_this_frame_[0].Center().x > outpost_center_.x){
                coming_armor_ = 0;
            }
        } else if(clockwise_ == -1)
        {
            if(detected_armors_in_this_frame_[0].Center().x >= outpost_center_.x)
            {
                going_armor_ = 0;
            }
            if(detected_armors_in_this_frame_[0].Center().x < outpost_center_.x){
                coming_armor_ = 0;
            }
        }
    }
    else
    {
        if(clockwise_ == 1)
        {
            if(detected_armors_in_this_frame_[0].Center().x > detected_armors_in_this_frame_[1].Center().x)
            {
                coming_armor_ = 0;
                going_armor_ = 1;
            } else
            {
                coming_armor_ = 1;
                going_armor_ = 0;
            }
        } else if(clockwise_ == -1)
        {
            if(detected_armors_in_this_frame_[0].Center().x > detected_armors_in_this_frame_[1].Center().x)
            {
                coming_armor_ = 1;
                going_armor_ = 0;
            } else
            {
                coming_armor_ = 0;
                going_armor_ = 1;
            }
        }
    }
}

void OutpostDataDetector::Clear() {
    outpost_center_ = {320,240};
    max_area_  = 0.0;
    last_armor_x_ = 0.0;
    coming_armor_ = -1;
    going_armor_ = -1;
    shoot_point_ = {0,0,0};
    for(int i = 0;i<4;i++)
        outpost_corner_[i] = {0,0};
    center_distance_ = 0;
    clockwise_ = 0;
    is_checked_clockwise = false;
    spinning_ = false;
    prepared_ = false;

}

