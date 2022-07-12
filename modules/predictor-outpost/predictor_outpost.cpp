#include "predictor_outpost.h"
const cv::Size kZoomRatio = {18,22};
void OutputData::Update(const coordinate::TranslationVector &shoot_point)  //TODO Unsure
{
    auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(shoot_point);
    yaw = shoot_point_spherical(0,0),pitch = shoot_point_spherical(1,0);

//    DLOG(INFO) << "yaw, pitch: " << float(yaw) << " " << float(pitch);
}


SendPacket OutpostPredictor::Run(DetectedData detected_data, float bullet_speed)
{
    DLOG(INFO)  << "Outpost_center_distance: " << detected_data.center_distance << " | "
                << "spining:"<<detected_data.spining<<"|";
    output_data_.fire = 0;
    armor_num = detected_data.out_post_armors.size();
//    double time_delay = center_distance_ / bullet_speed_ + kCommunicationTime_;
    if(detected_data.out_post_armors.empty()){
        return {0,0,0,
                0,0,
                0,0,0,0,
                0,0,0,0,0};
    }

    if(!detected_data.perpared){
        if(detected_data.out_post_armors.size() == 1){
            roi_corners_[0] = detected_data.out_post_armors[0].Corners()[0];
            roi_corners_[1] = detected_data.out_post_armors[0].Corners()[1];
            roi_corners_[2] = detected_data.out_post_armors[0].Corners()[2];
            roi_corners_[3] = detected_data.out_post_armors[0].Corners()[3];
            DLOG(INFO)<<detected_data.out_post_armors[0].Distance();
            DLOG(INFO)<<detected_data.out_post_armors[0].TranslationVectorWorld().x();
            debug::Painter::Instance()->DrawPoint(detected_data.out_post_armors[0].Center(),cv::Scalar(0,255,0),5,2);
            output_data_.Update(detected_data.out_post_armors[0].TranslationVectorCam());

        }
        else{
            double biggest_area = 0;
            int biggest_id = 0;
            for(int i = 0;i<detected_data.out_post_armors.size();i++){
                if(detected_data.out_post_armors[i].Area()>biggest_area){
                     biggest_area = detected_data.out_post_armors[i].Area();
                     biggest_id = i;
                }
            }
            roi_corners_[0] = detected_data.out_post_armors[biggest_id].Corners()[0];
            roi_corners_[1] = detected_data.out_post_armors[biggest_id].Corners()[1];
            roi_corners_[2] = detected_data.out_post_armors[biggest_id].Corners()[2];
            roi_corners_[3] = detected_data.out_post_armors[biggest_id].Corners()[3];
            debug::Painter::Instance()->DrawPoint(detected_data.out_post_armors[0].Center(),cv::Scalar(0,255,0),5,2);
            output_data_.Update(detected_data.out_post_armors[biggest_id].TranslationVectorCam());
        }
    }
    else{
        DLOG(INFO) << "outpost_center_" << detected_data.outpost_center;
        double pixel_distance = 0x3f3f3f3f;
        if(detected_data.coming_armor != -1)
            pixel_distance = algorithm::SqrtFloat((detected_data.out_post_armors[detected_data.coming_armor].Center().x - detected_data.outpost_center.x) *
                                                (detected_data.out_post_armors[detected_data.coming_armor].Center().x - detected_data.outpost_center.x) +
                                                (detected_data.out_post_armors[detected_data.coming_armor].Center().y - detected_data.outpost_center.y) *
                                                (detected_data.out_post_armors[detected_data.coming_armor].Center().y - detected_data.outpost_center.y));
//        if(pixel_distance >  advanced_distance && pixel_distance < advanced_distance+10)
//            output_data_.fire = 1;
        DLOG(INFO)<<"pixel_distance"<<pixel_distance;

        roi_corners_[0] = detected_data.corners0;
        roi_corners_[1] = detected_data.corners1;
        roi_corners_[2] = detected_data.corners2;
        roi_corners_[3] = detected_data.corners3;

        if(pixel_distance < 50 ){
            ready_ = true;
            ready_time_ = std::chrono::high_resolution_clock::now();
        }
        if(ready_)
        {
            auto current_time = std::chrono::high_resolution_clock::now();
            double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time - ready_time_)).count();
            if (time_gap *1e-3 > delay_time_){
                output_data_.fire = 1;
                ready_ = false;
            }
        }

        DLOG(INFO) << "pixel distance" << pixel_distance;
//    DLOG(INFO) << "shoot_point: " << shoot_point;
        output_data_.Update(detected_data.shoot_point);
        debug::Painter::Instance()->DrawPoint(detected_data.outpost_center, cv::Scalar(0, 255, 0), 10, 3);

    }
    DLOG(INFO) << "Outpost Send packet: " << output_data_.yaw << " | " << output_data_.pitch - float(delta_pitch_) << " | " << output_data_.fire;

//    cv::Mat3d camera_mat;
//    camera_mat <<   859.7363,   0,  0,
//                    -0.7875,    862.3096,0,
//                    950.8627,   567.8418,1;
//    auto show_point = coordinate::transform::CameraToPicture(camera_mat, shoot_point);
//    auto point1_x = short(show_point.x);
//    auto point1_y = short(show_point.y);

    return {output_data_.yaw-float(ArmorPredictorDebug::Instance().DeltaYaw()), output_data_.pitch -float(ArmorPredictorDebug::Instance().DeltaPitch()),
            output_data_.delay,0,output_data_.fire,
            0,0,
            0,0,
            0,0,
            0,0,
            float(output_data_.yaw + output_data_.pitch + 0 + output_data_.delay + output_data_.fire)};
}

void OutpostPredictor::GetROI(cv::Rect &roi_rect, const cv::Mat &src_image) {
    if(armor_num ==0)   {
        ++buff;
        if(buff>30){
            roi_rect = {};
            return;
        }
    }
    else buff =0;
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


