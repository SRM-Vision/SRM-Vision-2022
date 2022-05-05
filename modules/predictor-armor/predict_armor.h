//
// Created by xiguang on 2022/4/29.
//

#ifndef PREDICT_ARMOR_H_
#define PREDICT_ARMOR_H_

#include "data-structure/communication.h"
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "ekf.h"

// TODO Calibrate shoot delay and acceleration threshold.
const double kShootDelay = 0.02;
const double kFireAccelerationThreshold = 3.0;

/// Predicting function template structure.
struct PredictFunction {
    PredictFunction() : delta_t(0) {}

    /**
     * \brief Uniform linear motion.
     * \details It's supposed that target is doing uniform linear motion.
     * \tparam T Data type.
     * \param [in] x_0 Input original x.
     * \param [out] x Output predicted x.
     */
    template<typename T>
    void operator()(const T x_0[5], T x[5]) {
        x[0] = x_0[0] + delta_t * x_0[1];  // 0.1
        x[1] = x_0[1];  // 100
        x[2] = x_0[2] + delta_t * x_0[3];  // 0.1
        x[3] = x_0[3];  // 100
        x[4] = x_0[4];  // 0.01
    }

    double delta_t;
};

/// Measuring function template structure.
struct MeasureFunction {
    /**
     * \brief Collect positioning data and convert it to spherical coordinate.
     * \tparam T Data type.
     * \param [in] x Input x data.
     * \param [out] y Output target position in spherical coordinate system.
     */
    template<typename T>
    void operator()(const T x[5], T y[3]) {
        T _x[3] = {x[0], x[2], x[4]};
        coordinate::convert::Rectangular2Spherical<T>(_x, y);
    }
};

class PredictArmor : public Armor{
public:
    PredictArmor() = delete;
    explicit PredictArmor(const Armor& armor): Armor(armor), predict_speed_(0,0),fire_(0){}

    void Initialize(const std::array<float, 3>& yaw_pitch_roll){
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

    void Predict(const Armor& armor,double delta_t,double bullet_speed,const std::array<float, 3>& yaw_pitch_roll){
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
            auto delta_t_predict = armor.TranslationVectorWorld().norm() / bullet_speed + kShootDelay;
            predict.delta_t = delta_t_predict;
            predict(x_estimate.data(), x_predict.data());
            predict_world_vector_ << x_predict(0, 0), x_predict(2, 0), x_predict(4, 0);
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
        SendPacket send_packet = {float(yaw), float(pitch - ArmorPredictorDebug::Instance().DeltaPitch()),
                                  delay, distance_mode, fire_,
                                  float(yaw + pitch + distance_mode + delay + fire_ -
                                        ArmorPredictorDebug::Instance().DeltaPitch())};
        return send_packet;
    }

    /// the shoot point that need to show in the picture.
    inline cv::Point2f ShootPointInPic(const cv::Mat& intrinsic_matrix){
        return coordinate::transform::CameraToPicture(intrinsic_matrix,predict_cam_vector_);
    }

    inline void UpdateSpeed(double x_speed,double y_speed){
        ekf_.x_estimate_(1,0) = x_speed;
        ekf_.x_estimate_(3,0) = y_speed;
    }

    ATTR_READER_REF(predict_speed_,Speed);

private:
    coordinate::TranslationVector predict_world_vector_, predict_cam_vector_, shoot_point_vector_;
    Eigen::Vector2d predict_speed_;   /// x_v,y_v ; norm() = (x_v^2 + y_v^2)^(1/2)

    ExtendedKalmanFilter<5,3> ekf_;

    int fire_; /// to judge whether to fire.

    /// Update a new armor
    inline void Update(const Armor& armor){
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

    /// Update shoot point and predict camera vector.
    inline void UpdateShootPointAndPredictCam(const std::array<float, 3>& yaw_pitch_roll){
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
};

#endif //PREDICT_ARMOR_H_

