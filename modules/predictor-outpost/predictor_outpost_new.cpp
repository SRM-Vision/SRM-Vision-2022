//
// Created by lzy on 2022/7/13.
//
#include "predictor_outpost_new.h"
#include "math-tools/algorithms.h"
const int kFindTime = 5;
const cv::Size kZoomRatio = {18,22};
SendPacket OutpostPredictorNew::Run(Battlefield battlefield, float bullet_speed) {
    outpost_.ClearBottomArmor();
    SendPacket send_packet{0,0,0,
                           0,0,
                           0,0,
                           0,0,
                           0,0,
                           0,0,0};
    auto facilities = battlefield.Facilities();
    auto robots = battlefield.Robots();
    cv::Point3f shoot_point_;
    // TODO 高度判断
    for(auto &robot : robots[enemy_color_]){
        for( auto &armor:robot.second->Armors())
            outpost_.AddBottomArmor(armor);
    }
    for(auto &facility : facilities[enemy_color_]){
        for( auto &armor: facility.second->BottomArmors())
            outpost_.AddBottomArmor(armor);
    }
    debug::Painter::Instance()->DrawPoint(outpost_.center_point_,cv::Scalar(0,255,0),5,2);

    for (const auto &armor: outpost_.BottomArmors()) {
        debug::Painter::Instance()->DrawRotatedRectangle(armor.Corners()[0],
                                                         armor.Corners()[1],
                                                         armor.Corners()[2],
                                                         armor.Corners()[3],
                                                         cv::Scalar(0, 255, 0), 2);
        debug::Painter::Instance()->DrawText(std::to_string(armor.ID()), {200,200}, 255, 2);

    }

    //

    if(outpost_.BottomArmors().empty() && prepared){
        return {0,0,0,
                10,0,
                0,0,0,0,
                0,0,0,0,0};
    }
    if(outpost_.BottomArmors().empty()){
        return {0,0,0,
                0,0,
                0,0,0,0,
                0,0,0,0,0};
    }
    //
    // 记录开始时间
    if(need_init_)
    {
        start_time_ =  std::chrono::high_resolution_clock::now();
        need_init_ = false;
    }
    //

    // 判断旋转，找到来去装甲板
    if(clockwise_<= 7 && clockwise_ >= -7 && !checked_clockwise_)
        IsClockwise();
    else
        DecideComingGoing();
    //

    auto current_time_chrono = std::chrono::high_resolution_clock::now();
    double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time_chrono - start_time_)).count();

    int biggist_armor = FindBiggestArmor(outpost_.BottomArmors());

    if(time_gap*1e-3 < 4 && !prepared){
        if(outpost_.BottomArmors()[biggist_armor].Area() < biggest_area_) biggest_area_ = outpost_.BottomArmors()[biggist_armor].Area();
        auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(outpost_.BottomArmors()[biggist_armor].TranslationVectorCam());
        send_packet.yaw = shoot_point_spherical(0,0),send_packet.pitch = shoot_point_spherical(1,0);
        debug::Painter::Instance()->DrawPoint(outpost_.BottomArmors()[biggist_armor].Center(),cv::Scalar(0, 255, 0), 5,2);
        UpdateROICorners(outpost_.BottomArmors()[biggist_armor]);
    }
    else if(!prepared){
        if(outpost_.BottomArmors()[biggist_armor].Area() < biggest_area_) biggest_area_ = outpost_.BottomArmors()[biggist_armor].Area();
        auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(outpost_.BottomArmors()[biggist_armor].TranslationVectorCam());
        send_packet.yaw = shoot_point_spherical(0,0),send_packet.pitch = shoot_point_spherical(1,0);
        debug::Painter::Instance()->DrawPoint(outpost_.BottomArmors()[biggist_armor].Center(),cv::Scalar(0, 255, 0), 5,2);
        UpdateROICorners(outpost_.BottomArmors()[biggist_armor]);
        if(0.99 * biggest_area_ < outpost_.BottomArmors()[biggist_armor].Area()){
            buff +=2;
        }else if(0.97 * biggest_area_ < outpost_.BottomArmors()[biggist_armor].Area()){
            ++buff;
        }else buff = 0;
        if(buff > 50){
            outpost_.center_point_ = outpost_.BottomArmors()[biggist_armor].Center();
            prepared = true;
            start_time_ =  std::chrono::high_resolution_clock::now();
        }
    }


    if(prepared){
        double time_gap2 = (static_cast<std::chrono::duration<double, std::milli>>(current_time_chrono - start_time_)).count();
        debug::Painter::Instance()->DrawPoint(outpost_.center_point_,cv::Scalar(0, 255, 0), 5,2);
        if( time_gap2 * 1e-3 > 0.1)
            send_packet.distance_mode = 10;
        double pixel_distance = abs(outpost_.center_point_.x - outpost_.coming_center_.x);
        if(pixel_distance <10 && !ready_fire_){
            ready_time_ = std::chrono::high_resolution_clock::now();
            ready_fire_ = true;
        }
        if(ready_fire_){
            auto current_time = std::chrono::high_resolution_clock::now();
            double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time - ready_time_)).count();
            if (delay_time_< time_gap *1e-3 ){
                send_packet.fire = 1;
                ready_fire_ = false;
            }
        }
    }


    if(!prepared){
        send_packet.pitch += 0.11;
        send_packet.yaw += 0.04;
    }
    return send_packet;

}

int OutpostPredictorNew::FindBiggestArmor(const std::vector<Armor> &armors) {
    if(armors.size() == 1)
        return  0;
    int biggest_id = 0;
    double biggest_area = 0;
    for(int i = 0;i<armors.size();i++){
        if(armors[i].Area() > biggest_area )
            biggest_id = i;
            biggest_area = armors[i].Area();
    }
    return biggest_id;
}

void OutpostPredictorNew::GetROI(cv::Rect &roi_rect, const cv::Mat &src_image) {
    if(outpost_.BottomArmors().empty())   {
        ++roi_buff_;
        if(roi_buff_>30){
            roi_rect = {};
            return;
        }
    }
    else roi_buff_ =0;
    int width =  abs(roi_corners_[0].x - roi_corners_[1].x) < abs(roi_corners_[1].x - roi_corners_[2].x) ?
                 int(abs(roi_corners_[1].x - roi_corners_[2].x)) : int(abs(roi_corners_[0].x - roi_corners_[1].x));
    int height = abs(roi_corners_[0].y - roi_corners_[1].y) < abs(roi_corners_[1].y - roi_corners_[2].y) ?
                 int(abs(roi_corners_[1].y - roi_corners_[2].y)) : int(abs(roi_corners_[0].y - roi_corners_[1].y));
    int x = int(cv::min(cv::min(roi_corners_[0].x,roi_corners_[1].x),cv::min(roi_corners_[2].x,roi_corners_[3].x)));
    int y = int(cv::min(cv::min(roi_corners_[0].y,roi_corners_[1].y),cv::min(roi_corners_[2].y,roi_corners_[3].y)));
//        cv::RotatedRect target(corners_[0],corners_[1],corners_[2]);
//        auto target_right = target.boundingRect();
    cv::Rect target_right{x,y,width,height};
    auto zoom_size = cv::Size(target_right.width * kZoomRatio.width,
                              target_right.height * kZoomRatio.height);
    roi_rect = target_right + zoom_size;
    roi_rect -= cv::Point((zoom_size.width >> 1 ),(zoom_size.height >> 1));
    roi_rect = roi_rect & cv::Rect(0, 0, src_image.cols,src_image.rows);
}

void OutpostPredictorNew::DecideComingGoing() {
    if(outpost_.BottomArmors().size() == 1)
    {
        if(clockwise_ == 1)
        {
            if(outpost_.BottomArmors()[0].Center().x <= outpost_.center_point_.x)
            {
                outpost_.going_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.coming_center_ = cv::Point2f (0x3f3f3f3f, 0x3f3f3f3f);
            }
            if(outpost_.BottomArmors()[0].Center().x > outpost_.center_point_.x){
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_ = cv::Point2f (0x3f3f3f3f, 0x3f3f3f3f);
            }
        } else if(clockwise_ == -1)
        {
            if(outpost_.BottomArmors()[0].Center().x >= outpost_.center_point_.x)
            {
                outpost_.going_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.coming_center_ = cv::Point2f (0x3f3f3f3f, 0x3f3f3f3f);
            }
            if(outpost_.BottomArmors()[0].Center().x < outpost_.center_point_.x){
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_ = cv::Point2f (0x3f3f3f3f, 0x3f3f3f3f);
            }
        }
    }
    else
    {
        if(clockwise_ == 1)
        {
            if(outpost_.BottomArmors()[0].Center().x > outpost_.BottomArmors()[1].Center().x)
            {
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_ = outpost_.BottomArmors()[1].Center();
            } else
            {
                outpost_.coming_center_ = outpost_.BottomArmors()[1].Center();
                outpost_.going_center_ = outpost_.BottomArmors()[0].Center();
            }
        } else if(clockwise_ == -1)
        {
            if(outpost_.BottomArmors()[0].Center().x > outpost_.BottomArmors()[1].Center().x)
            {
                outpost_.coming_center_ = outpost_.BottomArmors()[1].Center();
                outpost_.going_center_ = outpost_.BottomArmors()[0].Center();
            } else
            {
                outpost_.coming_center_ = outpost_.BottomArmors()[0].Center();
                outpost_.going_center_= outpost_.BottomArmors()[1].Center();
            }
        }
    }
}

void OutpostPredictorNew::IsClockwise() {
    LOG(INFO) << "nums of armor" << outpost_.BottomArmors().size();
    double this_armor_x;
    if(outpost_.BottomArmors().size() == 1)
        this_armor_x = outpost_.BottomArmors()[0].Center().x;
    else
    {
        for(int i = 0; i < outpost_.BottomArmors().size() - 1; ++i)
            if(outpost_.BottomArmors()[i].Center().x > outpost_.BottomArmors()[i+1].Center().x)
                this_armor_x = outpost_.BottomArmors()[i+1].Center().x;
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
        checked_clockwise_ = true;
    }
    else if(clockwise_ < -7)
    {
        LOG(WARNING)<< "Outpost is anti-Clockwise";
        clockwise_ = -1;
        checked_clockwise_ = true;
    }
}

void OutpostPredictorNew::UpdateROICorners(const Armor& armor) {
    for(int i = 0;i<4;i++){
        roi_corners_[i] = armor.Corners()[i];
    }
}
