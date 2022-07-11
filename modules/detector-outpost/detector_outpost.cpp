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
        disappear_buff_++;
        if(disappear_buff_ > 30)
            Clear();

        return {detected_armors_in_this_frame_, outpost_center_, center_3D, shoot_point_,
                {0,0},{0,0},
                {0,0},{0,0},
                spining_, prepared_ , coming_armor_, going_armor_, clockwise_, center_distance_};
    }

    disappear_buff_ = 0;

    if(detected_armors_in_this_frame_.size() == 1)
        IsSpining(detected_armors_in_this_frame_[0], battlefield.TimeStamp());
    else{
        double biggest_area = 0;
        int biggest_id = 0;
        for(int i = 0;i<detected_armors_in_this_frame_.size();i++)
            if(detected_armors_in_this_frame_[i].Area() > biggest_area){
                biggest_area = detected_armors_in_this_frame_[i].Area();
                biggest_id = i;
            }
        IsSpining(detected_armors_in_this_frame_[biggest_id], battlefield.TimeStamp());
    }



    if(spining_){
        if(clockwise_<= 7 && clockwise_ >= -7 && !is_checked_clockwise)
            IsClockwise();
        else
            DecideComingGoing();
        FindBiggestArmor();
    }


    return  {detected_armors_in_this_frame_, outpost_center_, center_3D, shoot_point_,
             outpost_corner_[0],outpost_corner_[1],outpost_corner_[2],outpost_corner_[3],
             spining_,prepared_,coming_armor_, going_armor_, clockwise_, center_distance_};

}

void OutpostDataDetector::IsClockwise()
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

void OutpostDataDetector::FindBiggestArmor() {
    double biggest_id;
    for (int i = 0; i < detected_armors_in_this_frame_.size(); i++)
        if (detected_armors_in_this_frame_[i].Area() > max_area_) {
            biggest_id = i;
            max_area_ = detected_armors_in_this_frame_[i].Area();
            center_distance_ = detected_armors_in_this_frame_[i].Distance();
            outpost_center_ = detected_armors_in_this_frame_[i].Center();
            center_3D = detected_armors_in_this_frame_[i].TranslationVectorWorld();
            shoot_point_ = detected_armors_in_this_frame_[i].TranslationVectorCam();
            for(int j = 0;j<4;j++)
             outpost_corner_[j] = detected_armors_in_this_frame_[i].Corners()[j];
            prepared_ = true;
        }
    if((center_3D - detected_armors_in_this_frame_[biggest_id].TranslationVectorWorld()).norm()>0.4){
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
    spining_ = false;
    need_init_ = true;
    prepared_ = false;
    times_.clear();

}

void OutpostDataDetector::IsSpining(Armor armor, const uint64_t &current_time) {
    double time_after_jump{algorithm::Duration(last_jump_time_,current_time)};

    DLOG(INFO) << "TIME AFTER JUMP: " << time_after_jump;

    // If it exceeds the maximum period and has not turned, it is not top.
    if(!spining_&&time_after_jump > max_jump_period_1 ){
        spining_ = false;
        jump_count_ = 0;
    }

    if(spining_&&time_after_jump > max_jump_period_2 ){
        spining_ = false;
        jump_count_ = 0;
    }

    DLOG(INFO) << "JUMP PERIOD: " << jump_period_;

    // When tracking the same armor plate
    double current_yaw{std::atan2(armor.TranslationVectorWorld()(0,0),
                                  armor.TranslationVectorWorld()(2,0))};
    double yaw_delta{algorithm::shortest_angular_distance(last_yaw_, current_yaw)};
    DLOG(INFO) << "YAW DELTA: " << yaw_delta;
    // When jump enough
    if(std::abs(yaw_delta) > max_jump_yaw_){
        ++jump_count_;
        if(jump_count_ > 1 && std::signbit(yaw_delta) == std::signbit(last_yaw_jump_delta_)){
            spining_ = true;
            jump_period_ = time_after_jump;
        }

        last_jump_time_ = current_time;
        last_yaw_jump_delta_ = yaw_delta;
        last_jump_position_ = armor.TranslationVectorWorld();
    }

    last_yaw_ = current_yaw;

}

