#include "predictor_rune.h"
#include "math-tools/algorithms.h"

[[maybe_unused]] RunePredictor::RunePredictor(bool debug) :
        debug_(debug),
        is_okay_to_fit_(false),
        is_need_to_fit_(false),
        b_(0),
        phase_(0),
        delta_u_(0),
        delta_v_(0),
        amplitude_(0),
        palstance_(0),
        rotated_angle_(0),
        rotated_radian_(0),
        current_time_(0.0),
        predicted_angle_(0),
        current_fan_angle_(0),
        current_fan_radian_(0),
        last_RTG_vec_(0, 0),
        last_fan_current_angle_(0),
        current_fan_palstance_(0.0),
        final_target_point_send_to_control_(0, 0) {}

bool RunePredictor::Initialize(const std::string &config_path, bool debug) {
    cv::FileStorage config;

    // Open config file.
    config.open(config_path, cv::FileStorage::READ);
    if (!config.isOpened()) {
        LOG(ERROR) << "Failed to open rune detector config file " << config_path << ".";
        return false;
    }

    config["AMPLITUDE"] >> amplitude_;
    config["PALSTANCE"] >> palstance_;
    config["PHASE"] >> phase_;

    b_ = 2.090 - amplitude_;
    is_okay_to_fit_ = false;
    is_need_to_fit_ = false;
    debug_ = debug;

    return true;
}

SendPacket RunePredictor::Run(const PowerRune &power_rune, AimModes mode) {
    rune_ = power_rune;

    if (mode == kBigRune) {
        LOG(INFO) << "Now Run Mode : Big Rune!";
        current_fan_palstance_ = CalAngularVelocity();
        CalCurrentFanAngle();                  // Using armor_rotated_point
        CalPredictAngle(
                mode);                 // Calculate function parameters first, preparing 1000 frames;Then integral.
        CalPredictPoint();                     // Calculate the predict_target_point
        CalYawPitchDelay();
        FanChanged();
    } else {
        LOG(INFO) << "Now Run Mode : Small Rune!";
        palstance_ = 1.04717;
        CalPredictAngle(mode);
        CalPredictPoint();
        CalYawPitchDelay();
        FanChanged();
    }

    return {send_yaw_pitch_delay_.x,
            send_yaw_pitch_delay_.y,
            send_yaw_pitch_delay_.z,
            static_cast<int>(send_yaw_pitch_delay_.x + send_yaw_pitch_delay_.y + send_yaw_pitch_delay_.z)};
}

bool RunePredictor::FanChanged() {
    if (std::abs(last_fan_current_angle_ - 0.0) < 0.001) {
        last_fan_current_angle_ = current_fan_angle_;
        return false;
    }
    double delta_angle = std::abs(current_fan_angle_ - last_fan_current_angle_);
    last_fan_current_angle_ = current_fan_angle_;
    if (delta_angle > 60 && delta_angle < 350) {
        LOG(INFO) << "Fan changed now! ";
        return true;
    } else
        return false;
}

/**
 * @Brief: 计算能量机关角速度
 */
double RunePredictor::CalAngularVelocity() {
    /// 丢失的数据不记
    if (rune_.CenterR() == cv::Point2f(0, 0)) {
        last_RTG_vec_ = cv::Point2f(0, 0);
        return -1;
    }

    float cal_angle;
    current_fan_palstance_ = -1;
    auto current_time = std::chrono::high_resolution_clock::now();                 // Used to calculate time gap in order to cal angular velocity.
    current_time_ = double(
            std::chrono::duration_cast<std::chrono::milliseconds>(  // Used to provide time_data in order to fit the curve.
                    std::chrono::system_clock::now().time_since_epoch()).count());
    current_time_ /= 1000;                                                         // ms to s
    /// 滤掉无效帧
    if (last_RTG_vec_ != cv::Point2f(0.0, 0.0)) {
        cal_angle = algorithm::VectorAngle(last_RTG_vec_, rune_.RtgVec()); // 计算矢量夹角
        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time - last_time_)).count();
        current_fan_palstance_ = cal_angle / time_gap * 1000.0;
        circle_fan_palstance_queue.Push(std::make_pair(current_time, current_fan_palstance_));
    }

    /// 时间和位置矢量更新
    last_time_ = current_time;
    last_RTG_vec_ = rune_.RtgVec();

    speed_data_.push_back(current_fan_palstance_);
    time_data_.push_back(current_time_);

    return current_fan_palstance_;
}

/**
 * @Brief: calculate current fan angle
 */
void RunePredictor::CalCurrentFanAngle() {
    current_fan_radian_ = std::atan2(rune_.RtpVec().y * (-1.0), rune_.RtpVec().x);

    /** Adapt std::atan2 rad. Maybe in III or IV */
    if (current_fan_radian_ < 0 && rune_.RtpVec().x < 0
        || current_fan_radian_ < 0 && rune_.RtpVec().x > 0) {
        current_fan_radian_ += CV_2PI;
    }
    current_fan_angle_ = current_fan_radian_ * 180 / CV_PI;

    /** Adapt overflow value to normal region. */
    while (current_fan_angle_ < 0) {
        current_fan_angle_ += 360;
    }
    while (current_fan_angle_ > 360) {
        current_fan_angle_ -= 360;
    }
}

/**
* @Brief: calculate rad integral from speed
*/
double RunePredictor::CalRadIntegralFromSpeed(const double &integral_time) {
    const double c = 0;
    return (-1.0) * rune_.Clockwise() *
           (-amplitude_ / palstance_ * cos(palstance_ * integral_time + phase_) + b_ * integral_time + c);
}

/**
* @Brief: calculate parameters
*/
void RunePredictor::CalFunctionParameters() {
    if (is_need_to_fit_ == 1) {
        // Prepare stage
        if (speed_data_.size() < kPrepareNum && is_okay_to_fit_ == 0) {
            return;
        }
        if (speed_data_.size() == kPrepareNum && is_okay_to_fit_ == 0) {
            is_okay_to_fit_ = true;
            speed_data_.clear();
            return;
        }

        // Finish preparing and collect data
        if (speed_data_.size() < 335 && is_okay_to_fit_ == 1) {
            return;
        }

        // Finishing collecting
        if (speed_data_.size() == 335 && is_okay_to_fit_ == 1) {
            for (double &speed: speed_data_) {
                speed = speed / 180 * CV_PI;
            }

            ceres::Problem problem;
            for (int i = 0; i < kNumObservation; ++i) {
                problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<TrigonometricResidual, 1, 1, 1, 1>(
                                new TrigonometricResidual(time_data_[i], speed_data_[i], rune_.Clockwise())),
                        new ceres::CauchyLoss(0.5),
                        &amplitude_,
                        &palstance_,
                        &phase_
                );
            }
            ceres::Solver::Options options;
            options.max_num_iterations = 300;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;

            if (debug_) {
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                std::cout << summary.BriefReport() << "\n";
                std::cout << "Inference A = " << amplitude_ << " Omega = " << palstance_ << " phase = " << phase_
                          << std::endl;
            }
            is_okay_to_fit_ = false;
            is_need_to_fit_ = false;
        }
    }
}

/**
* @Brief: calculate rotated angle
*/
void RunePredictor::CalPredictAngle(AimModes mode) {
    if (mode == kBigRune) {
        if (is_need_to_fit_) {
            CalFunctionParameters();
            rotated_angle_ = 0;
            CalCurrentFanAngle();
            predicted_angle_ = current_fan_angle_;
        } else {
            double delay_time = 7.0 / 28.5;
            rotated_radian_ = CalRadIntegralFromSpeed(current_time_ + delay_time)
                              - CalRadIntegralFromSpeed(current_time_);
            rotated_angle_ = rotated_radian_ * 180 / CV_PI;

            CalCurrentFanAngle();
            predicted_angle_ = (-1) * rune_.Clockwise() * (current_fan_angle_ + rotated_angle_);
        }
    } else {
        double delay_time = 7.0 / 28.5;  // The offset of bullet
        current_time_ = double(std::chrono::duration_cast<std::chrono::milliseconds>
                                       (std::chrono::system_clock::now().time_since_epoch()).count());
        current_time_ /= 1000;   // ms -> s
        rotated_radian_ = (-1.0) * rune_.Clockwise() *
                          (palstance_ * (current_time_ + delay_time) - palstance_ * (current_time_));
        rotated_angle_ = rotated_radian_ * 180 / CV_PI;

        CalCurrentFanAngle();
        predicted_angle_ = (-1) * rune_.Clockwise() * (current_fan_angle_ + rotated_angle_);
    }
}

/**
* @Brief: calculate radius
*/
double RunePredictor::CalRadius() const {
    return sqrt((rune_.RtpVec().x - rune_.CenterR().x) *
                (rune_.RtpVec().x - rune_.CenterR().x)
                + (rune_.RtpVec().y - rune_.CenterR().y) *
                  (rune_.RtpVec().y - rune_.CenterR().y));
}

/**
* @Brief: calculate predict point
*/
void RunePredictor::CalPredictPoint() {
    double radius = CalRadius();
    double predicted_rad = predicted_angle_ * 180 / CV_PI;
    predicted_rad = CV_2PI - predicted_rad;
    predicted_target_point_ = rune_.CenterR() +
                              cv::Point2f(float(std::cos(predicted_rad) * radius),
                                          float(std::sin(predicted_rad) * radius));
}

cv::Point3f RunePredictor::CalYawPitchDelay() {
    if (rune_.ArmorCenterP() == cv::Point2f(0, 0))  // 如果没有识别到装甲板，则不计算预测
    {
        DLOG(INFO) << "Rune Predictor receive zeros detection result.";
        return {0, 0, 0};
    }


    float delta_u = delta_u_, delta_v = delta_v_;
    DLOG(INFO) << "predicted_angle_:" << predicted_angle_;

    double angle = predicted_angle_;

    while (angle < 0) angle += 360;
    while (angle > 360) angle -= 360;

    if (angle > 150 && angle < 180) {
        delta_u -= 20;
        delta_v += 20;
    }
    if (angle >= 90 && angle < 180) {
        delta_u += 10;
        delta_v += 10;
    }
    if (angle < 90 && angle >= 0) {
        delta_v += 20;
    }
    if (angle >= 180 && angle < 225) {
        delta_u -= 10;
        delta_v += 20;
    }
    if (angle >= 225 && angle < 270) {
        delta_u -= 10;
        delta_v += 20;
    }
    if (angle >= 315 && angle < 360) {
        delta_u -= 10;
        delta_v += 20;
    }
    if (angle >= 270 && angle < 315) {
        delta_u -= 5;
        delta_v += 25;
    }

    auto final_predicted_target_point_ = predicted_target_point_ + cv::Point2f(delta_u, -delta_v - 15);//0,-80

    auto yaw_data_pixel = float(std::atan2(final_predicted_target_point_.x - rune_.ImageCenter().x, 1350));
    auto pitch_data_pixel = float(std::atan2(-rune_.ImageCenter().y + final_predicted_target_point_.y, 1350));

    DLOG(INFO) << "Yaw " << yaw_data_pixel << ", Pitch " << pitch_data_pixel << ".";
    send_yaw_pitch_delay_ = cv::Point3f(yaw_data_pixel, pitch_data_pixel, 1.0f);

    return send_yaw_pitch_delay_;
}


