//
// Created by xiguang on 2022/7/11.
//

#include "predict_armor.h"

void PredictArmor::Initialize(const std::array<float, 3> &yaw_pitch_roll) {
    Eigen::Matrix<double, 5, 1> x_real; // used to initialize the ekf
    x_real << translation_vector_world_[0],
            0,
            translation_vector_world_[1],
            0,
            translation_vector_world_[2];
    ekf_.Initialize(x_real);

    predict_world_vector_ = translation_vector_world_;

    UpdateShootPointAndPredictCam(yaw_pitch_roll);
    predict_speed_ << 0, 0;
}

void PredictArmor::Predict(const Armor& armor,double delta_t,double bullet_speed,const std::array<float, 3>& yaw_pitch_roll,const double shoot_delay){
    Eigen::Vector2d new_speed;
    if(CmdlineArgParser::Instance().WithEKF()){
        PredictFunction predict;  ///< Predicting function.
        MeasureFunction measure;  ///< Measuring function.
        predict.delta_t = delta_t;

        /// translate measured value to the format of ekf
        Eigen::Matrix<double, 3, 1> y_real;
        coordinate::convert::Rectangular2Spherical(armor.TranslationVectorWorld().data(), y_real.data());

        Eigen::Matrix<double, 5, 1> x_predict = ekf_.Predict(predict);
        Eigen::Matrix<double, 5, 1> x_estimate = ekf_.Update(measure, y_real);

        /// add ballistic delay
        auto delta_t_predict = armor.TranslationVectorWorld().norm() / bullet_speed + shoot_delay;
        predict.delta_t = delta_t_predict;
        predict(x_estimate.data(), x_predict.data());
        predict_world_vector_ << x_predict(0, 0), x_predict(2, 0), x_predict(4, 0);
        DLOG(INFO) << "speed:        " << x_predict(1,0) << "   " << x_predict(3,0);
        // try to not predict pitch.
        auto predict_world_spherical_vector = coordinate::convert::Rectangular2Spherical(predict_world_vector_);
        predict_world_spherical_vector[1] = y_real[1];  // not predict pitch
        predict_world_vector_ = coordinate::convert::Spherical2Rectangular(predict_world_spherical_vector);
        new_speed << x_predict(1,0), x_predict(3,0);
    }else {
        predict_world_vector_ = armor.TranslationVectorWorld();
        new_speed << 0, 0;
    }
    UpdateShootPointAndPredictCam(yaw_pitch_roll);
    Update(armor);
    // if acceleration is higher than threshold, fire.
    if((new_speed - predict_speed_).norm() / delta_t < kFireAccelerationThreshold)
        fire_ = 1;
    else
        fire_ = 0;
    predict_speed_ = new_speed;
}

void PredictArmor::GetROI(cv::Rect &roi_rect, const cv::Mat &src_image) {
    int width =  abs(corners_[0].x - corners_[1].x) < abs(corners_[1].x - corners_[2].x) ?
                 int(abs(corners_[1].x - corners_[2].x)) : int(abs(corners_[0].x - corners_[1].x));
    int height = abs(corners_[0].y - corners_[1].y) < abs(corners_[1].y - corners_[2].y) ?
                 int(abs(corners_[1].y - corners_[2].y)) : int(abs(corners_[0].y - corners_[1].y));
    int x = int(cv::min(cv::min(corners_[0].x,corners_[1].x),cv::min(corners_[2].x,corners_[3].x)));
    int y = int(cv::min(cv::min(corners_[0].y,corners_[1].y),cv::min(corners_[2].y,corners_[3].y)));
//        cv::RotatedRect target(corners_[0],corners_[1],corners_[2]);
//        auto target_right = target.boundingRect();
    cv::Rect target_right{x,y,width,height};
    auto zoom_size = cv::Size(int(target_right.width * kZoomRatio.width + 0.5 * src_image.rows),
                              int(target_right.height * kZoomRatio.height + 0.1 * src_image.cols));
    roi_rect = target_right + zoom_size;
    roi_rect -= cv::Point((zoom_size.width >> 1 ),(zoom_size.height >> 1));
    roi_rect -= cv::Point(0, int(roi_rect.height * 0.15));    // move up
    roi_rect = roi_rect & cv::Rect(0, 0, src_image.cols,src_image.rows);
}

void PredictArmor::AntiSpin(double jump_period, const coordinate::TranslationVector &last_jump_position,
                            uint64_t current_time, uint64_t last_jump_time,
                            const std::array<float, 3> &yaw_pitch_roll) {
    if(algorithm::Duration(last_jump_time,current_time) / jump_period < kAllowFollowRange){
        fire_ = true;
    }else{
        predict_world_vector_ << last_jump_position;
        UpdateShootPointAndPredictCam(yaw_pitch_roll);
    }
}

void PredictArmor::Update(const Armor &armor) {
    if(armor.Area() > Area())   // only update id when armor`s area become larger.
        id_ = armor.ID();
    for(int i = 0; i < 4; ++i)
        corners_[i] = armor.Corners()[i];
    center_ = armor.Center();
    rotation_vector_cam_ = armor.RotationVectorCam();
    translation_vector_cam_ = armor.TranslationVectorCam();
    rotation_vector_world_ = armor.RotationVectorWorld();
    translation_vector_world_ = armor.TranslationVectorWorld();

    distance_ = armor.Distance();
    confidence_ = armor.Confidence();

    type_ = armor.Type();
    color_ = armor.Color();
}

void PredictArmor::UpdateShootPointAndPredictCam(const std::array<float, 3> &yaw_pitch_roll) {
    shoot_point_vector_ = coordinate::transform::WorldToCamera(
            predict_world_vector_,
            coordinate::transform::EulerAngleToRotationMatrix(yaw_pitch_roll),
            Eigen::Vector3d::Zero(),
            Eigen::Matrix3d::Identity());
    predict_cam_vector_ = coordinate::transform::WorldToCamera(
            predict_world_vector_,
            coordinate::transform::EulerAngleToRotationMatrix(yaw_pitch_roll),
            coordinate::camera_to_imu_translation_matrix,
            coordinate::camera_to_imu_rotation_matrix);
}
