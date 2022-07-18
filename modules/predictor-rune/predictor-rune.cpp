#include "predictor-rune.h"
#include <ceres/ceres.h>
#include "detector-rune/detector_rune_debug.h"

predictor::rune::RunePredictor::RunePredictor() :
        rune_(), state_(), fitting_data_(),
        output_data_(), bullet_speed_(0),
        predicted_angle_(), predicted_point_() {}

bool predictor::rune::RunePredictor::Initialize(const std::string &config_path) {
    cv::FileStorage config;

    // Open config file.
    config.open(config_path, cv::FileStorage::READ);
    if (!config.isOpened()) {
        DLOG(WARNING) << "Failed to open rune predictor config file " << config_path << ".";
        return false;
    }

    config["AMPLITUDE"] >> rotational_speed_.a;
    config["PALSTANCE"] >> rotational_speed_.w;
    config["PHASE"] >> rotational_speed_.p;
    rotational_speed_.b = 2.090 - rotational_speed_.a;

    return true;
}

SendPacket predictor::rune::RunePredictor::Run(const PowerRune &power_rune, AimModes aim_mode, float bullet_speed) {
    rune_ = power_rune;
//    if (bullet_speed != bullet_speed_)
//        InitModel(bullet_speed);
    bullet_speed_ = bullet_speed;

    // Aim mode is big rune.
    if (aim_mode == kBigRune) {
        DLOG(INFO) << "Rune predictor mode: big.";
        state_.UpdatePalstance(rune_, fitting_data_);
        state_.UpdateAngle(rune_.RtpVec());
        /// Fit data if not ready.
        PredictAngle(aim_mode);
        PredictPoint();
        output_data_.Update(rune_, predicted_angle_, predicted_point_, fixed_point_);
        state_.CheckMode();
    }

        // Aim mode is small rune.
    else if (aim_mode == kSmallRune) {
        DLOG(INFO) << "Rune predictor mode: small.";
        rotational_speed_.w = rune_.Clockwise() * 1.04717;
        PredictAngle(aim_mode);
        PredictPoint();
        output_data_.Update(rune_, predicted_angle_, predicted_point_, fixed_point_);
        state_.CheckMode();
    }

        // Compensation test
    else {
        DLOG(INFO) << "Compensation mode!";
        PredictAngle(aim_mode);
        predicted_point_ = rune_.ArmorCenterP();
        output_data_.Update(rune_, predicted_angle_, predicted_point_, fixed_point_);
    }

    DLOG(INFO) << "Predicted angle: " << predicted_angle_ << "   Predicted point: " << predicted_point_;
    DLOG(INFO) << "Fixed point: " << fixed_point_;
    return {output_data_.yaw, output_data_.pitch, output_data_.delay, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            static_cast<float>(output_data_.yaw + output_data_.pitch + output_data_.delay)};
}

/// Update current palstance and collect palstance and time data.
bool predictor::rune::State::UpdatePalstance(const PowerRune &rune, FittingData &fitting_data) {
    /*
     * Ignore discarded data. Here use rtg, may need change to rtp.
     */
    if (rune.CenterR() == cv::Point2f(0, 0)) {
        last_rtg_vec = cv::Point2f(0, 0);
        return false;
    }

    float angle;  ///< Temp angle var in DEGREE.
    auto current_time_chrono = std::chrono::high_resolution_clock::now();
    current_time = double(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    current_time *= 1e-3;  ///< Convert millisecond to second.

    /*
     * Filter invalid data.
     */
    if (last_rtg_vec != cv::Point2f(0, 0)) {
        angle = algorithm::VectorAngle(last_rtg_vec, rune.RtpVec());  // Calculate angle in DEGREE.
        if (angle == 0) {
            last_rtg_vec = rune.RtpVec();
            last_time = current_time_chrono;
            return true;
        }
        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(
                current_time_chrono - last_time)).count();

        current_palstance = angle / time_gap * 1e3;
        /// Restrict palstance between min_value to max_value
        current_palstance = std::min(160.0, std::max(0.2, current_palstance));

        /*
         * Collect palstance and time data.
         */
        fitting_data.palstance.push_back(current_palstance);
        fitting_data.time.push_back(current_time);
        last_time = current_time_chrono;
    }

    last_rtg_vec = rune.RtpVec();
    return true;
}

/// Update current angle.
void predictor::rune::State::UpdateAngle(const cv::Point2f &rtg_vec) {
    /*
     * Calculate RADIAN angle.
     */
    auto rad = algorithm::Atan2Float(rtg_vec.y, rtg_vec.x);

    /*
     * Make positive radian for quadrant III or IV.
     */
    if (rad < 0)
        rad += CV_2PI;
    current_angle = rad * 180 / CV_PI;

    /*
     * Fix current angle to principal value interval (0~360).
     */
    while (current_angle < 0) current_angle += 360;
    while (current_angle > 360) current_angle -= 360;
    current_angle = 360 - current_angle;
}

/// Get predict angle.
void predictor::rune::RunePredictor::PredictAngle(AimModes aim_mode) {
    double rotated_angle;  ///< Angle to rotate calculated by palstance.

    /*
     * Aim mode is big rune. Need to fit data.
     */
    if (aim_mode == kBigRune) {
        if (!fitting_data_.ready) {
            /// Collecting specific frames.
            if (fitting_data_.palstance.size() >= kPreparePalstanceDataNum) {
                fitting_data_.ready = true;  // Ready to fit.
                /// Clear.
                fitting_data_.palstance.clear();
                fitting_data_.time.clear();
            }
            state_.UpdateAngle(rune_.RtpVec());
            predicted_angle_ = state_.current_angle;   ///< No Predicting When Fitting.
        } else {

            /*
             * Fit only if data num meet the condition.
             */
            fitting_data_.Fit(rotational_speed_);
            double bullet_delay_time = 7.0 / bullet_speed_;   ///< Ballistic Time Compensation
            auto rotated_radian =
                    rotational_speed_.AngularIntegral(state_.current_time + bullet_delay_time + rune::kCompensateTime)
                    - rotational_speed_.AngularIntegral(state_.current_time);
            rotated_angle = rotated_radian * 180 / CV_PI;
            state_.UpdateAngle(rune_.RtpVec());
            DLOG(INFO) << "clock_wise: " << rune_.Clockwise();

            predicted_angle_ = state_.current_angle - rune_.Clockwise() * rotated_angle;
        }
    }

        // Aim mode is small rune.
    else if (aim_mode == kSmallRune) {
        double bullet_delay_time = 7.0 / bullet_speed_;  // Ballistic Time Compensation.
        auto rotated_radian = rune_.Clockwise() * rotational_speed_.w * (kCompensateTime + bullet_delay_time);
        rotated_angle = rotated_radian * 180 / CV_PI;

        state_.UpdateAngle(rune_.RtpVec());

        predicted_angle_ = state_.current_angle - rune_.Clockwise() * rotated_angle;
    }

    // Compensation test
    else{
        state_.UpdateAngle(rune_.RtpVec());
        predicted_angle_ = state_.current_angle;
    }
}

/// Get point from predict angle.
void predictor::rune::RunePredictor::PredictPoint() {
    auto radius = algorithm::SqrtFloat(rune_.RtpVec().x * rune_.RtpVec().x
                                       + rune_.RtpVec().y * rune_.RtpVec().y);
    auto predicted_rad = static_cast<float>(CV_2PI - predicted_angle_ * CV_PI / 180);
    predicted_point_ = rune_.CenterR() + cv::Point2f(algorithm::CosFloat(predicted_rad) * radius,
                                                     algorithm::SinFloat(predicted_rad) * radius);
}

/// Update data to be sent.
void predictor::rune::OutputData::Update(const PowerRune &rune,
                                         double predicted_angle,
                                         const cv::Point2f &predicted_point,
                                         cv::Point2f &fixed_point) {
    if (rune.ArmorCenterP() == cv::Point2f(0, 0)) {
        DLOG(INFO) << "Rune Predictor receive zeros detection result.";
        yaw = pitch = delay = 0;
    }

    int delta_u = RuneDetectorDebug::Instance().DeltaU() - 2000;  ///< Horizontal ballistic compensation.
    int delta_v = RuneDetectorDebug::Instance().DeltaV() - 2000;  ///< Vertical ballistic compensation.

    while (predicted_angle < 0) predicted_angle += 360;
    while (predicted_angle > 360) predicted_angle -= 360;

    fixed_point = predicted_point + cv::Point2f(static_cast<float>(delta_u), static_cast<float>(delta_v));

    // Use SIMD atan2 for 4x floats.
    float x[4] = {kP_yaw, kP_pitch, 1, 1},
            y[4] = {fixed_point.x - rune.ImageCenter().x, fixed_point.y - rune.ImageCenter().y, 1, 1},
            z[4] = {0};
    algorithm::Atan2FloatX4(y, x, z);

    yaw = abs(z[0]) < .15f ? z[0] : (z[0] / abs(z[0]) * .005f);
    pitch = abs(z[1]) < .15f ? z[1] : (z[1] / abs(z[1]) * .005f);  // Avoid excessive offset
//    double target_h = 5 + (rune.ArmorCenterP().x * rune.ArmorCenterP().y) /
//                          std::abs((rune.ArmorCenterP().x * rune.ArmorCenterP().y)) *
//                          algorithm::CosFloat(float(predicted_angle));
//    pitch_solver_.UpdateParam(target_h, 8.32);
//    auto res = pitch_solver_.Solve(-CV_PI / 6, CV_PI / 3, 0.01, 16);
//    pitch = float(res.x()) - pitch > .15f ? .05f : float(res.x()) - pitch;

    DLOG(INFO) << "Output yaw: " << yaw << ", pitch: " << pitch << ".";
    delay = 1.f;
}

/// Check if fan changed.
void predictor::rune::State::CheckMode() {
    if (std::abs(last_angle - 0.0) < 0.001) {
        last_angle = current_angle;
    } else {
        double delta_angle = std::abs(current_angle - last_angle);
        last_angle = current_angle;
        if (delta_angle > 60 && delta_angle < 350)
            DLOG(INFO) << "Rune fan changes now.";
    }
}

double predictor::rune::RotationalSpeed::AngularIntegral(double integral_time) const {
    constexpr double c = 0;
    // Angular displacement integral
    return -a / w * cos(w * integral_time + p) + b * integral_time + c;
}

void predictor::rune::FittingData::Fit(RotationalSpeed &rotational_speed) {
    /*
     * Keep and update observation data num in the vector.
     */
    if (!first_fit && palstance.size() == kObservationDataNum + kBufferDataNum)
    {
        auto buffer_begin = palstance.begin();
        auto buffer_end   = palstance.begin() + kBufferDataNum;
        palstance.erase(buffer_begin, buffer_end);
    }
    /*
     * Collect data and fit.
     */
    if ( palstance.size() == kFirstFitPalstanceDataNum ||
         palstance.size() == kObservationDataNum)
    {
        ++ fit_num;
        DLOG(INFO) << "Fit num: " << fit_num << "Data amount: " << palstance.size();


        for (double &speed: palstance)
            speed = speed / 180 * CV_PI;


        /*
         * Solve
         */
        ceres::Problem problem;
        int fit_data_num = first_fit ? kFirstFitPalstanceDataNum : kObservationDataNum;
        for (int i = 0; i < fit_data_num; ++i)
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<TrigonometricResidual, 1, 1, 1, 1>(
                            new TrigonometricResidual(time[i], palstance[i])),
                    new ceres::CauchyLoss(0.5), &rotational_speed.a,
                    &rotational_speed.w, &rotational_speed.p);
        ceres::Solver::Options options;
        options.max_num_iterations           = 300;
        options.linear_solver_type           = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;


        /*
         * Result
         */
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        DLOG(INFO) << summary.BriefReport();
        DLOG(INFO) << "Fitting: a = " << rotational_speed.a
                   << ", w = "        << rotational_speed.w
                   << ", phi = "      << rotational_speed.p;
        first_fit = false;
    }
}

//void predictor::rune::RunePredictor::InitModel(double bullet_speed) {
//    auto f = trajectory_solver::AirResistanceModel();
//    f.SetParam(0.48, 994, 30, 0.017, 0.0032);
//    auto a = trajectory_solver::BallisticModel();
//    a.SetParam(f, 31);
//    pitch_solver_ = trajectory_solver::PitchAngleSolver();
//    pitch_solver_.SetParam(a, bullet_speed, 1.35, 2.41, 8.32);
//}
