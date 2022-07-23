//
// Created by xiguang on 2022/7/14.
//

#include "predictor_armor.h"
#include "math-tools/exponential-filter.h"
#include "compensator/compensator.h"

/// When an armor lasts gray for time below this value, it will be considered as hit.
constexpr unsigned int kMaxGreyCount = 20;

/// Picture Distance threshold to judge whether a target is too far.
constexpr double kPicDistanceThreshold = 100;

/// Switch target in anti-top When new armor bigger than 0.7 * old armor.
constexpr double kSwitchByAreaThreshold = 0.7;

/// used for anti-spin, allow to follow for fire.
constexpr double kAllowFollowRange = 0.3;

/// the threshold to consider a armor is oblique.
constexpr double kObliqueThreshold = 1.5;

/// the threshold to consider a armor is oblique in anti-spin mode.
constexpr double kObliqueThresholdInSpin = 1;

// TODO Calibrate shoot delay and acceleration threshold.

/// shooting delay
constexpr double kShootDelay = 0.15;

/// The maximum acceleration allowed to fire
constexpr double kFireAccelerationThreshold = 0.8;

/// When the same armor is detected for more than 10 frames of time, we consider it`s detecting.
constexpr int kDetectThreshold = 10;

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
    void operator()(const T x_0[7], T x[7]) {
        x[0] = x_0[0] + delta_t * x_0[1];  // 0.1
        x[1] = x_0[1] + delta_t * x_0[2];  // 100
        x[2] = x_0[2];
        x[3] = x_0[3] + delta_t * x_0[4];  // 0.1
        x[4] = x_0[4] + delta_t * x_0[5];  // 100
        x[5] = x_0[5];
        x[6] = x_0[6];
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
    void operator()(const T x[7], T y[3]) {
        T _x[3] = {x[0], x[3], x[6]};
        coordinate::convert::Rectangular2Spherical<T>(_x, y);
    }
};

void ArmorPredictor::Initialize(const std::string &controller_type_name) {
    ArmorPredictorDebug::Instance().Initialize(controller_type_name);
    compensator_traj_.Initialize(controller_type_name);
#if !NDEBUG
    ArmorPredictorDebug::Instance().addTrackbar();
#endif
}

SendPacket ArmorPredictor::Run(const Battlefield &battlefield, const cv::MatSize &size, Entity::Colors color) {
    SetColor(color);
    auto& robots = battlefield.Robots();
    auto& facilities = battlefield.Facilities();

    // whether locked same target
    bool locked_same_target(true);

    std::shared_ptr<Armor> target_current{nullptr};

    cv::Point2f picture_center{float(size().width / 2.0), float(size().height / 2.0)};

    std::vector<Armor> armors;

    double delta_t = algorithm::NanoSecondsToSeconds(last_time_, battlefield.TimeStamp());
    last_time_ = battlefield.TimeStamp();

    // add armors to vector.
    if(robots.find(enemy_color_) != robots.end())
        for(auto&& [_,robot]:robots.at(enemy_color_))
            armors.insert(armors.end(),robot->Armors().begin(),robot->Armors().end());
    if(robots.find(Entity::kGrey) != robots.end() && grey_buffer_ < kMaxGreyCount)
        for(auto&& [_,robot]:robots.at(Entity::kGrey))
            armors.insert(armors.end(),robot->Armors().begin(),robot->Armors().end());

    if(facilities.find(enemy_color_) != facilities.end())
        for(auto&& [_,facility]:facilities.at(enemy_color_))
            armors.insert(armors.end(),facility->BottomArmors().begin(),facility->BottomArmors().end());
    if(facilities.find(Entity::kGrey) != facilities.end() && grey_buffer_ < kMaxGreyCount)
        for(auto&& [_,facility]:facilities.at(Entity::kGrey))
            armors.insert(armors.end(),facility->BottomArmors().begin(),facility->BottomArmors().end());

    DLOG(INFO) << "find " << armors.size() << " armors.";

    // find armor which is the same as the last target or nearest to the picture center.
    if(last_target_){
        DLOG(INFO) << "exist historical armor.";
        locked_same_target = FindMatchArmor(target_current,
                                            last_target_->Center(),
                                            armors,
                                            kPicDistanceThreshold,
                                            (spin_predictor_.IsSpin() ? kObliqueThresholdInSpin : kObliqueThreshold));
    }

    //not find current armor
    if(!target_current) {
        DLOG(INFO) << "not found last armor, try to switch new one.";
        locked_same_target = false;
        // oblique threshold is higher than oblique threshold in anti-spin mode.
        FindMatchArmor(target_current,
                       picture_center,
                       armors,
                       DBL_MAX,
                       (spin_predictor_.IsSpin() ? kObliqueThresholdInSpin : kObliqueThreshold));
    }

    // Do nothing if nothing is found.
    if(!target_current){
        Clear();
        DLOG(INFO) << "NO TARGET!";
        return {0, 0, 0, 0, 0,
                0, 0,
                0, 0,
                0, 0,
                0, 0};
    }

    // update grey buffer
    if(target_current->Color() == Entity::kGrey)
        ++grey_buffer_;
    else
        grey_buffer_ = 0;

    // update spin detector
    spin_predictor_.Update(*target_current, battlefield.TimeStamp());

    Eigen::Matrix<double,3,1> compensator_result;

    if(locked_same_target){
        DLOG(INFO) << "locked the same armor.";
        ++detect_count_;

        compensator_result = compensator_traj_.AnyTargetOffset(battlefield.BulletSpeed(),
                                                               *target_current,
                                                               battlefield.YawPitchRoll()[1]);

        // EKF Predict
        Predict(*target_current, delta_t, compensator_result.y(),
                battlefield.YawPitchRoll(), ArmorPredictorDebug::Instance().ShootDelay());

        // anti spinning
        if (spin_predictor_.IsSpin()){
            DLOG(INFO) << "It`s spin.";
            // find another armor in the robot
            Armor* another_armor{nullptr};
            for(auto &&armor:armors){
                if(armor.ID() == last_target_->ID() && armor.Center() != target_current->Center())
                    another_armor = &armor;
            }
            if(another_armor
            && spin_predictor_.Clockwise() != -1
            && another_armor->Area() > last_target_->Area() * kSwitchByAreaThreshold
            && (spin_predictor_.Clockwise() ^ (last_target_->Center().x > another_armor->Center().x))){
                // Update speed in ekf, to speed up fitting.
                UpdateLastArmor(*another_armor);
                InitializeEKF(battlefield.YawPitchRoll(),another_armor->TranslationVectorWorld());

                ekf_.x_estimate_(1,0) = predict_speed_(0,0);
                ekf_.x_estimate_(4,0) = -predict_speed_(1,0);
                ekf_.x_estimate_(2,0) = predict_acc_(0,0);
                ekf_.x_estimate_(5,0) = -predict_acc_(1,0);

                compensator_result = compensator_traj_.AnyTargetOffset(battlefield.BulletSpeed(),
                                                                       *target_current,
                                                                       battlefield.YawPitchRoll()[1]);

                Predict(*another_armor,delta_t,compensator_result.y(),
                        battlefield.YawPitchRoll(),ArmorPredictorDebug::Instance().ShootDelay());
                DLOG(INFO) << "anti-spin mode, switch to another spinning armor.";
            }else if(algorithm::NanoSecondsToSeconds(spin_predictor_.LastJumpTime(), battlefield.TimeStamp()) /
               spin_predictor_.JumpPeriod() < kAllowFollowRange){
                DLOG(INFO) << "anti-spin mode, allow to fire.";
                fire_ = 1;
            }else{
                // if spinning, Swing head in advance.
                DLOG(INFO) << "anti-spin mode, head shake.";
                fire_ = 0;
                predict_world_vector_ << spin_predictor_.LastJumpPosition();
                UpdateShootPointAndPredictCam(battlefield.YawPitchRoll());
            }
        }

        if(detect_count_ < kDetectThreshold && !spin_predictor_.IsSpin()){
            predict_world_vector_ << target_current->TranslationVectorWorld();
            UpdateShootPointAndPredictCam(battlefield.YawPitchRoll());
        }

    }else {
        DLOG(INFO) << "locked a new armor, initialize ekf.";
        detect_count_ = 0;
        // locked a new armor
        last_target_ = target_current;
        InitializeEKF(battlefield.YawPitchRoll(), target_current->TranslationVectorWorld());
        predict_world_vector_ = target_current->TranslationVectorWorld();
        UpdateShootPointAndPredictCam(battlefield.YawPitchRoll());
        predict_speed_ << 0, 0;
        predict_acc_ << 0, 0;
    }
    double vx = battlefield.SelfSpeed().x() * cos(battlefield.YawPitchRoll()[0]);
    double dx = compensator_result.y() * vx;
    return GenerateSendPacket(battlefield, (float)compensator_result.x(), dx);
}

auto ArmorPredictor::SameArmorByPixelDistance(const cv::Point2f &target_center,
                                              std::vector<Armor> &armors,
                                              double threshold) -> decltype(armors.cend()) {
    double distance_min{DBL_MAX};
    auto same_armor{armors.cend()};
    for(auto begin{armors.cbegin()};begin < armors.cend();++begin){
        double distance{cv::norm(begin->Center() - target_center)};
        if(distance < threshold && distance < distance_min) {
            distance_min = distance;
            same_armor = begin;
        }
    }
    DLOG(INFO) << "min distance to found armor: " << distance_min;
    return same_armor;
}

bool ArmorPredictor::FindMatchArmor(std::shared_ptr<Armor> &target, const cv::Point2f &target_center,
                                    std::vector<Armor> &armors, double distance_threshold,
                                    double oblique_threshold) {
    bool is_same_armor{true};
    while(true){
        auto same_armor = SameArmorByPixelDistance(target_center, armors, distance_threshold);
        if(same_armor != armors.cend()){
            DLOG(INFO) << "find a same armor by distance in picture.";
            double armor_height_pixel = std::max(
                    abs(same_armor->Corners()[0].y - same_armor->Corners()[1].y),
                    abs(same_armor->Corners()[1].y - same_armor->Corners()[2].y)
            ), armor_width_pixel = std::max(
                    abs(same_armor->Corners()[0].x - same_armor->Corners()[1].x),
                    abs(same_armor->Corners()[1].x - same_armor->Corners()[2].x)
            );

            // when armor is too oblique, change target.
            if(armor_width_pixel / armor_height_pixel < oblique_threshold){
                DLOG(INFO) << "armor is too oblique, try to change target.";
                armors.erase(same_armor);
                is_same_armor = false;
                continue;
            }else{
                DLOG(INFO) << "confirm armor.";
                target = std::make_shared<Armor>(*same_armor);
                return is_same_armor;
            }
        }else
            break;
    }
    DLOG(INFO) << "no matched armor.";
    target = nullptr;
    return false;
}

void ArmorPredictor::InitializeEKF(const std::array<float, 3> &yaw_pitch_roll,
                                   const coordinate::TranslationVector &translation_vector_world) {
    Eigen::Matrix<double, 7, 1> x_real; // used to initialize the ekf
    Eigen::Matrix<double, 7, 7> predict_cov;
    Eigen::Matrix<double, 3, 3> measure_cov;
    x_real << translation_vector_world[0],
            0,0,
            translation_vector_world[1],
            0,0,
            translation_vector_world[2];

    predict_cov << ArmorPredictorDebug::Instance().PredictedXZNoise(), 0, 0, 0, 0, 0, 0,
                   0, ArmorPredictorDebug::Instance().PredictedXSpeedNoise(), 0, 0, 0, 0, 0,
                   0, 0, ArmorPredictorDebug::Instance().PredictedXAccelerationNoise(), 0, 0, 0, 0,
                   0, 0, 0, ArmorPredictorDebug::Instance().PredictedYNoise(), 0, 0, 0,
                   0, 0, 0, 0, ArmorPredictorDebug::Instance().PredictedYSpeedNoise(), 0, 0,
                   0, 0, 0, 0, 0, ArmorPredictorDebug::Instance().PredictedYAccelerationNoise(), 0,
                   0, 0, 0, 0, 0, 0, ArmorPredictorDebug::Instance().PredictedXZNoise();

    measure_cov << ArmorPredictorDebug::Instance().MeasureXNoise(), 0, 0,
                   0, ArmorPredictorDebug::Instance().MeasureYNoise(), 0,
                   0, 0, ArmorPredictorDebug::Instance().MeasureZNoise();

    ekf_.Initialize(x_real,predict_cov, measure_cov);
}

void ArmorPredictor::UpdateShootPointAndPredictCam(const std::array<float, 3> &yaw_pitch_roll) {
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

void ArmorPredictor::Predict(const Armor &armor, double delta_t, double flight_duration,
                             const std::array<float, 3> &yaw_pitch_roll, double shoot_delay) {
    Eigen::Vector2d new_speed;
    Eigen::Vector2d new_acc;
    if(CmdlineArgParser::Instance().WithEKF()){
        PredictFunction predict;  ///< Predicting function.
        MeasureFunction measure;  ///< Measuring function.
        predict.delta_t = delta_t;
        DLOG(INFO) << "delta_t: " << delta_t;
        /// translate measured value to the format of ekf
        Eigen::Matrix<double, 3, 1> y_real;
        coordinate::convert::Rectangular2Spherical(armor.TranslationVectorWorld().data(), y_real.data());
        DLOG(INFO) << "translation world vector of target: " << armor.TranslationVectorWorld() << '\n';

# if !NDEBUG
        // update trackbar param
        ArmorPredictorDebug::Instance().AlterPredictCovMeasureCov(ekf_);
# endif

        ekf_.Predict(predict);
        auto x_estimate = ekf_.Update(measure, y_real);
        Eigen::Matrix<double,7,1> x_predict;
        /// add ballistic delay
        auto delta_t_predict = flight_duration + shoot_delay;
        predict.delta_t = delta_t_predict;
        DLOG(INFO) << "delta_t_predict: " << delta_t_predict;
        predict(x_estimate.data(), x_predict.data());
        predict_world_vector_ << x_predict(0, 0), x_predict(3, 0), x_predict(6, 0);
        DLOG(INFO) << "speed:        " << x_predict(1,0) << "   " << x_predict(4,0);
        // try to not predict pitch.
        auto predict_world_spherical_vector = coordinate::convert::Rectangular2Spherical(predict_world_vector_);
        predict_world_spherical_vector[1] = y_real[1];  // not predict pitch
        predict_world_vector_ = coordinate::convert::Spherical2Rectangular(predict_world_spherical_vector);
        new_speed << x_predict(1,0), x_predict(4,0);
        new_acc << x_predict(2,0), x_predict(5,0);
    }else {
        predict_world_vector_ = armor.TranslationVectorWorld();
        new_speed << 0, 0;
        new_acc << 0, 0;
    }
    UpdateShootPointAndPredictCam(yaw_pitch_roll);
    UpdateLastArmor(armor);

    DLOG(INFO) << "acceleration: " << (new_speed - predict_speed_).norm() / delta_t;
    DLOG(INFO) << "predicted acceleration: " << new_acc.norm();

    // if acceleration is higher than threshold, fire.
    if(new_acc.norm() < kFireAccelerationThreshold)
        fire_ = 1;
    else
        fire_ = 0;
    predict_speed_ = new_speed;
    predict_acc_ = new_acc;
}

void ArmorPredictor::UpdateLastArmor(const Armor &armor) {
    unsigned int id(last_target_->ID());
    if(armor.Area() > last_target_->Area())   // only update id when armor`s area become larger.
        id = armor.ID();
    last_target_ = std::make_shared<Armor>(armor);
    last_target_->SetID(id);
}

SendPacket ArmorPredictor::GenerateSendPacket(const Battlefield &battlefield, float current_pitch, double current_dx) {
    auto shoot_point_spherical = coordinate::convert::Rectangular2Spherical(shoot_point_vector_);
    auto delta_yaw = shoot_point_spherical(0, 0),delta_pitch = shoot_point_spherical(1, 0);
    auto delay = 0.f;
    int distance_mode = 0;
    if (0 <= last_target_->Distance() && last_target_->Distance() < 4) distance_mode = 1;
    if (4 <= last_target_->Distance() && last_target_->Distance() < 6) distance_mode = 2;
    if (6 <= last_target_->Distance() && last_target_->Distance() < 8) distance_mode = 3;
    DLOG(INFO) << "pnp distance: " << last_target_->Distance();
    DLOG(INFO) << "Filtered distance is : " << distance_filter_.Filter(last_target_->Distance());

    delta_yaw -= atan2(current_dx, last_target_->Distance());
    DLOG(INFO) << "Self speed: " << battlefield.SelfSpeed() << ", delta yaw: " << delta_yaw;

    // 图传点
//        cv::Mat3d camera_mat;
//        camera_mat <<   859.7363,   0,  0,
//                        -0.7875,    862.3096,0,
//                        950.8627,   567.8418,1;
//        auto show_point = coordinate::transform::CameraToPicture(camera_mat, shoot_point);
//        auto point1_x = short(show_point.x);
//        auto point1_y = short(show_point.y);

    return {float(- delta_yaw + battlefield.YawPitchRoll()[0] + ArmorPredictorDebug::Instance().DeltaYaw()),
            float(current_pitch - ArmorPredictorDebug::Instance().DeltaPitch()),
            delay, distance_mode, fire_,
            0, 0,
            0, 0,
            0, 0,
            0, 0};
}

void ArmorPredictor::Clear() {
    last_target_ = nullptr;
    spin_predictor_.Reset();
}

cv::Point2f ArmorPredictor::TargetCenter() {
    if(last_target_)
        return last_target_->Center();
    return {-1,-1};
}

cv::Point2f ArmorPredictor::ShootPointInPic(const cv::Mat &intrinsic_matrix, cv::MatSize size) {
    DLOG(INFO) << "shooting point in picture: " << (last_target_ ?
            coordinate::transform::CameraToPicture(intrinsic_matrix,predict_cam_vector_)
            : cv::Point2f{float(size().width / 2.0), float(size().height / 2.0)});
    if(last_target_)
        return coordinate::transform::CameraToPicture(intrinsic_matrix,predict_cam_vector_);
    return {float(size().width / 2.0), float(size().height / 2.0)};
}

double ArmorPredictor::GetTargetDistance() {
    if(last_target_)
        return last_target_->Distance();
    return 0;
}
