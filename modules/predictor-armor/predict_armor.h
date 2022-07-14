//
// Created by xiguang on 2022/4/29.
//

#ifndef PREDICT_ARMOR_H_
#define PREDICT_ARMOR_H_

#include <math-tools/coordinate.h>
#include <digital-twin/components/armor.h>
#include "data-structure/communication.h"
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "math-tools/ekf.h"

class PredictArmor : public Armor{
public:
    PredictArmor() = delete;
    explicit PredictArmor(const Armor& armor): Armor(armor), predict_speed_(0,0),fire_(0){}

    void Initialize(const std::array<float, 3>& yaw_pitch_roll);

    void Predict(const Armor& armor,double delta_t,double bullet_speed,const std::array<float, 3>& yaw_pitch_roll,double shoot_delay);

    /**
    * \brief Generate a packet according to data inside.
    * \return Send packet to serial port.
    */
    [[nodiscard]] inline SendPacket GenerateSendPacket() const {
        auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(shoot_point_vector_);
        auto yaw = shoot_point_spherical(0,0),pitch = shoot_point_spherical(1,0);

        auto delay = 0.f;// TODO Add delay and check_sum here.
        int distance_mode = 0;
        if (0 <= distance_ && distance_ < 2) distance_mode = 1;
        if (2 <= distance_ && distance_ < 4) distance_mode = 2;
        if (4 <= distance_ && distance_ < 6) distance_mode = 3;
        DLOG(INFO) << " distance: " << distance_;

        // 图传点
//        cv::Mat3d camera_mat;
//        camera_mat <<   859.7363,   0,  0,
//                        -0.7875,    862.3096,0,
//                        950.8627,   567.8418,1;
//        auto show_point = coordinate::transform::CameraToPicture(camera_mat, shoot_point);
//        auto point1_x = short(show_point.x);
//        auto point1_y = short(show_point.y);

        return {float(yaw +0.03), float(pitch - ArmorPredictorDebug::Instance().DeltaPitch()+ ArmorPredictorDebug::Instance().DeltaYaw()),
                delay, distance_mode, fire_,
                0,0,
                0,0,
                0,0,
                0,0,
                float(yaw + pitch + distance_mode + delay + fire_ - ArmorPredictorDebug::Instance().DeltaPitch()
                      + ArmorPredictorDebug::Instance().DeltaYaw())};
    }

    /// the shoot point that need to show in the picture.
    inline cv::Point2f ShootPointInPic(const cv::Mat& intrinsic_matrix){
        return coordinate::transform::CameraToPicture(intrinsic_matrix,predict_cam_vector_);
    }

    inline void UpdateSpeed(double x_speed,double y_speed){
        ekf_.x_estimate_(1,0) = x_speed;
        ekf_.x_estimate_(3,0) = y_speed;
    }

    void GetROI(cv::Rect &roi_rect, const cv::Mat &src_image);

    void SpeedDecay(double decay_ratio_x,double decay_ratio_y){
        ekf_.x_estimate_(1,0) = ekf_.x_estimate_(1,0) * decay_ratio_x;
        ekf_.x_estimate_(3,0) = ekf_.x_estimate_(3,0) * decay_ratio_y;
    }

    void AntiSpin(double jump_period, const coordinate::TranslationVector &last_jump_position,
                  uint64_t current_time, uint64_t last_jump_time, const std::array<float, 3> &yaw_pitch_roll);

    ATTR_READER_REF(predict_speed_,Speed)

    ATTR_READER_REF(predict_world_vector_,PredictWorldVector)

    ATTR_READER_REF(predict_cam_vector_,PredictCamVector)

    ATTR_READER_REF(shoot_point_vector_,ShootPointVector)

    friend class AntiTopDetectorRenew;

private:
    coordinate::TranslationVector predict_world_vector_, predict_cam_vector_, shoot_point_vector_;
    Eigen::Vector2d predict_speed_;   ///< x_v,y_v ; norm() = (x_v^2 + y_v^2)^(1/2)

    ExtendedKalmanFilter<5,3> ekf_;

    int fire_; ///< to judge whether fire.

    /// Update a new armor
    inline void Update(const Armor& armor);

    /// Update shoot point and predict camera vector.
    inline void UpdateShootPointAndPredictCam(const std::array<float, 3>& yaw_pitch_roll);
};

#endif //PREDICT_ARMOR_H_

