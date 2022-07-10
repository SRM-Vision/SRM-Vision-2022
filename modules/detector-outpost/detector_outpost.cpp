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
    }

    if (detected_armors_in_this_frame_.empty())
    {
        DLOG(INFO) << "No outpost armor founded";
        disappear_buff_++;
        if(disappear_buff_ > 50)
            Clear();

        return {detected_armors_in_this_frame_, outpost_center_, center_3D, shoot_point_,
                {0,0},{0,0},
                {0,0},{0,0},
                spining_, need_init_ , coming_armor_, going_armor_, clockwise_, center_distance_};
    }

    disappear_buff_ = 0;

    IsSpining(detected_armors_in_this_frame_.size(), battlefield.TimeStamp());

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

void OutpostDataDetector::FindBiggestArmor()
{
    if(need_init_)
    {
        start_time_ =  std::chrono::high_resolution_clock::now();
        need_init_ = false;
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
                outpost_corner_[0] = armor.Corners()[0];
                outpost_corner_[1] = armor.Corners()[1];
                outpost_corner_[2] = armor.Corners()[2];
                outpost_corner_[3] = armor.Corners()[3];

            }
            else
            {
                if(abs(outpost_center_.y - armor.Center().y) > kVertical_threshold_)
                {
                    DLOG(WARNING) << "outpost center error too much, finding again";
                    need_init_ = true;
                    prepared_ = false;
                }
                else{
                    DLOG(WARNING) << "outpost center founded";
                    prepared_ = true;
                }

            }
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
    center_distance_ = 0;
    clockwise_ = 0;
    is_checked_clockwise = false;
    spining_ = false;
    need_init_ = true;
    times_.clear();
    prepared_ = false;
}

void OutpostDataDetector::IsSpining(const int &new_armor_num, const uint64_t &now_timestamp) {
    auto new_spining_period{double(now_timestamp - timestamp_) * 1e-9 };
    double exit_spining_time;
    if(spining_)    exit_spining_time = 25;
    else            exit_spining_time = 15;     // 如果在旋转的话，更难退出旋转状态
    if(new_armor_num != armor_num_ && new_spining_period > 0.3) {  //  >1.5为了滤除一部分误识别
        spining_period_ = new_spining_period;
        timestamp_ = now_timestamp;
        armor_num_ = new_armor_num;
        if(times_.size()<5)
            times_.emplace_back(spining_period_);
        else{
            times_.erase(times_.begin());
            times_.emplace_back(spining_period_);
        }
    }
    if(new_spining_period > exit_spining_time ){
        times_.clear();
    }
    double sum=0;
    for(int i = 0;i<times_.size();i++)
        sum += times_[i];

    if(sum < 15 && times_.size() == 5 )
        spining_ = true;
    else
        spining_ = false;

    DLOG(INFO)<<"sum"<<sum<<"size"<<times_.size();

}

