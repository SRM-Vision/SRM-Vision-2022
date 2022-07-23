#include "predictor-rune.h"
#include <ceres/ceres.h>
#include "predictor_rune_debug.h"

predictor::rune::RunePredictor::RunePredictor() :
        rune_(), state_(), fitting_data_(),
        output_data_(), bullet_speed_(0),
        predicted_angle_(), predicted_point_() {}

bool predictor::rune::RunePredictor::Initialize() {

#if !NDEBUG
    RunePredictorDebug::Instance().addTrackbar();
#endif

    return true;
}

SendPacket predictor::rune::RunePredictor::Run(const PowerRune &power_rune, AimModes aim_mode, float bullet_speed) {
    rune_ = power_rune;
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
    }

        // Aim mode is small rune.
    else if (aim_mode == kSmallRune) {
        DLOG(INFO) << "Rune predictor mode: small.";
        rotational_speed_.w = rune_.Clockwise() * 1.04717;
        PredictAngle(aim_mode);
        PredictPoint();
        output_data_.Update(rune_, predicted_angle_, predicted_point_, fixed_point_);
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
    return {output_data_.yaw, output_data_.pitch, output_data_.delay, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
}

/// Update current palstance and collect palstance and time data.
bool predictor::rune::State::UpdatePalstance(const PowerRune &rune, FittingData &fitting_data) {
    if (rune.CenterR() == cv::Point2f(0, 0) || FanChanged()) {
        last_rtp_vec = cv::Point2f(0, 0);
        return false;
    }

    if (last_rtp_vec != cv::Point2f(0, 0)) {
        float angle = CalcVectorsAngle(last_rtp_vec, rune.RtpVec());  // Calculate angle in DEGREE.
        if (angle == 0) {
            last_rtp_vec = rune.RtpVec();
            return true;
        }
        current_palstance = angle / rune.TimeGap();
        // Restrict palstance between min_value to max_value
//        current_palstance = std::min(160.0, std::max(0.2, current_palstance));

//        current_palstance = angle / time_gap * 1e3;
        current_palstance = angle / rune.TimeGap() * 1e3;
        /// Restrict palstance between min_value to max_value
        current_palstance = std::min(160.0, std::max(0.2, current_palstance));

        DLOG(INFO) << "Current time: " << rune.CurrentTime() << " time gap: " << rune.TimeGap();
        DLOG(INFO) << "Current palstance: " << current_palstance;
        fitting_data.palstance.push_back(current_palstance);
        fitting_data.time.push_back(rune.CurrentTime());
    }

    last_rtp_vec = rune.RtpVec();
    return true;
}

/// Update current angle.
void predictor::rune::State::UpdateAngle(const cv::Point2f &rtp_vec) {
    /*
     * Calculate RADIAN angle.
     */
    current_angle = algorithm::Atan2Float(rtp_vec.y, rtp_vec.x) * 180 / CV_PI;

    while (current_angle < 0) current_angle += 360;
    while (current_angle > 360) current_angle -= 360;
    current_angle = 360 - current_angle;
}

// Get predicted angle.
void predictor::rune::RunePredictor::PredictAngle(AimModes aim_mode) {
    double rotated_angle;  ///< Angle to rotate calculated by palstance.
    float compensateTime = float(RunePredictorDebug::Instance().CompensateTime() - 500) / 1000;

    /*
     * Aim mode is big rune. Need to fit data.
     */
    if (aim_mode == kBigRune) {
        if (!fitting_data_.ready) {
            // Collecting specific frames.
            if (fitting_data_.palstance.size() >= kPreparePalstanceDataNum) {
                fitting_data_.ready = true;  // Ready to fit.
                // Clear.
                fitting_data_.palstance.clear();
                fitting_data_.time.clear();
            }
            state_.UpdateAngle(rune_.RtpVec());
            predicted_angle_ = state_.current_angle;   // No Predicting When Fitting.
        } else {
            /*
             * Fit only if data num meet the condition.
             */
            fitting_data_.Fit(rotational_speed_);
            if (fitting_data_.first_fit) {
                state_.UpdateAngle(rune_.RtpVec());
                predicted_angle_ = state_.current_angle;   //< No Predicting When Fitting.
            } else {
                double bullet_delay_time = 7.0 / bullet_speed_;   //< Ballistic Time Compensation
                auto rotated_radian = rotational_speed_.AngularIntegral(
                        rune_.CurrentTime() + bullet_delay_time + compensateTime)
                                      - rotational_speed_.AngularIntegral(rune_.CurrentTime());
                rotated_angle = rotated_radian * 180 / CV_PI;
                state_.UpdateAngle(rune_.RtpVec());
                DLOG(INFO) << "clock_wise: " << rune_.Clockwise();

                predicted_angle_ = state_.current_angle - rune_.Clockwise() * rotated_angle;
                DLOG(INFO) << "Predicted: " << predicted_angle_ << "\tCurrent: "
                           << state_.current_angle << "\tRotated_angle: " << rotated_angle;
//                cv::waitKey(0);
            }
        }
    }

        // Aim mode is small rune.
    else if (aim_mode == kSmallRune) {
        double bullet_delay_time = 7.0 / bullet_speed_;  // Ballistic Time Compensation.
        auto rotated_radian = rune_.Clockwise() * rotational_speed_.w * (compensateTime + bullet_delay_time);
        rotated_angle = rotated_radian * 180 / CV_PI;

        state_.UpdateAngle(rune_.RtpVec());

        predicted_angle_ = state_.current_angle - rune_.Clockwise() * rotated_angle;
    }

        // Compensation test
    else {
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

    int delta_u = RunePredictorDebug::Instance().DeltaU() - 2000;  ///< Horizontal ballistic compensation.
    int delta_v = RunePredictorDebug::Instance().DeltaV() - 2000;  ///< Vertical ballistic compensation.

    while (predicted_angle < 0) predicted_angle += 360;
    while (predicted_angle > 360) predicted_angle -= 360;

    fixed_point = predicted_point + cv::Point2f(static_cast<float>(delta_u), static_cast<float>(delta_v));

    // Use SIMD atan2 for 4x floats.
    float x[4] = {kP_yaw, kP_pitch, 1, 1},
            y[4] = {fixed_point.x - rune.ImageCenter().x, fixed_point.y - rune.ImageCenter().y, 1, 1},
            z[4] = {0};
    algorithm::Atan2FloatX4(y, x, z);

    yaw = abs(z[0]) < .1f ? z[0] : .005f, pitch = abs(z[1]) < .1f ? z[1] : .005f;  // Avoid excessive offset

    DLOG(INFO) << "Output yaw: " << yaw << ", pitch: " << pitch << ".";
    delay = 1.f;
}

/// Check if fan changed.
bool predictor::rune::State::FanChanged() {
    if (std::abs(last_angle - 0.0) < 0.001) {
        last_angle = current_angle;
        return false;
    } else {
        double delta_angle = std::abs(current_angle - last_angle);
        last_angle = current_angle;
        if (delta_angle > 45 && delta_angle < 315) {
            DLOG(INFO) << "Rune fan changes now.";
            fan_changed_time_chrono = std::chrono::high_resolution_clock::now();
            return true;
        }
    }
    return false;
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
    // Fixme just for the case that 'prepare' and 'observe' are different
    if (!first_fit && palstance.size() == kObservationDataNum + kBufferDataNum) {
        auto buffer_begin_palstance = palstance.begin();
        auto buffer_end_palstance = palstance.begin() + kBufferDataNum;
        auto buffer_begin_time = time.begin();
        auto buffer_end_time = time.begin() + kBufferDataNum;
        palstance.erase(buffer_begin_palstance, buffer_end_palstance);
        time.erase(buffer_begin_time, buffer_end_time);
    }

    /*
     * Collect data and fit.
     */
    if (palstance.size() == kFirstFitPalstanceDataNum ||
        palstance.size() == kObservationDataNum) {
        ++fit_num;
        DLOG(INFO) << "Fit num: " << fit_num << "Data amount: " << palstance.size();

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
        options.max_num_iterations = kMaxNumIterations;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        rotational_speed.b = 2.090 - rotational_speed.a;

        fit_complete_time = std::chrono::high_resolution_clock::now();

        /*
         * Result
         */
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        DLOG(INFO) << summary.BriefReport();
        DLOG(INFO) << "Fitting: a = " << rotational_speed.a
                   << ", w = " << rotational_speed.w
                   << ", phi = " << rotational_speed.p;
        first_fit = false;
    }
}

float predictor::rune::State::CalcVectorsAngle(const cv::Point2f &first_vector, const cv::Point2f &second_vector) {
    float angle = std::min(1.f, std::max(-1.f, first_vector.dot(second_vector) /
                                               (std::hypot(first_vector.x, first_vector.y) *
                                                std::hypot(second_vector.x, second_vector.y))));
    return float(acos(angle));
}

void predictor::rune::RunePredictor::AutoFire() {
    if (rune::kAutoFireFlag) {
        /// auto fire.
        auto_fire_signal_ = state_.FanChanged();
        auto current_time_chrono = std::chrono::high_resolution_clock::now();
        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time_chrono -
                                                                                  state_.fan_changed_time_chrono)).count();
        if (auto_fire_signal_ && time_gap == kAutoFireTimeGap) {
            output_data_.fire = 1;
            DLOG(INFO) << "Fire now !!!!!!!!";
            auto_fire_signal_ = false;
        }
    }
}