#include "predictor-rune.h"
#include <ceres/ceres.h>
#include "predictor_rune_debug.h"

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
    fitting_data_.palstance.clear();
    fitting_data_.time.clear();
    fitting_data_.first_fit = true;
    fitting_data_.fit_num = 0;
    fitting_data_.ready = false;

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
        output_data_.Update(rune_, predicted_angle_, predicted_point_, fixed_point_, bullet_speed);
    }

        // Aim mode is small rune.
    else if (aim_mode == kSmallRune) {
        DLOG(INFO) << "Rune predictor mode: small.";
        rotational_speed_.w = rune_.Clockwise() * 1.04717;
        PredictAngle(aim_mode);
        PredictPoint();
        output_data_.Update(rune_, predicted_angle_, predicted_point_, fixed_point_, bullet_speed);
    }

        // Compensation test
    else {
        DLOG(INFO) << "Compensation mode!";
        PredictAngle(aim_mode);
        predicted_point_ = rune_.ArmorCenterP();
        output_data_.Update(rune_, predicted_angle_, predicted_point_, fixed_point_, bullet_speed);
    }

    DLOG(INFO) << "Predicted angle: " << predicted_angle_ ;
    DLOG(INFO) << "Predicted point: " << predicted_point_ << "Fixed point: " << fixed_point_;
    return {output_data_.yaw, output_data_.pitch, output_data_.delay, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
}

/// Update current palstance and collect palstance and time data.
bool predictor::rune::State::UpdatePalstance(const PowerRune &rune, FittingData &fitting_data) {
    if (rune.CenterR() == cv::Point2f(0, 0) || FanChanged()) {
        last_rtp_vec = cv::Point2f(0, 0);
        return false;
    }

    float angle;
    if (last_rtp_vec != cv::Point2f(0, 0)) {
        angle = CalcVectorsAngle(last_rtp_vec, rune.RtpVec());  // Calculate angle in DEGREE.
        if (angle == 0) {
            last_rtp_vec = rune.RtpVec();
            return true;
        }


        // Restrict palstance between min_value to max_value

        current_palstance = angle / rune.TimeGap() * 1e3;
        /// Restrict palstance between min_value to max_value
        current_palstance = std::min(160.0, std::max(0.2, current_palstance));



//        DLOG(INFO) << "angle: " << angle;
//        DLOG(INFO) << "Current time: " << rune.CurrentTime() << " time gap: " << rune.TimeGap();
        DLOG(INFO) << "Current palstance: " << current_palstance;
        fitting_data.palstance.push_back(current_palstance);
        fitting_data.time.push_back(rune.CurrentTime());
    }

    last_rtp_vec = rune.RtpVec();
    return true;
}

/// [checked] Update current angle.
void predictor::rune::State::UpdateAngle(const cv::Point2f &rtp_vec) {
    /*
     * Calculate RADIAN angle.
     */
    auto rad = algorithm::Atan2Float(rtp_vec.y, rtp_vec.x);

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

/// [checked] Get predicted angle.
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
            if (fitting_data_.first_fit)
            {
                state_.UpdateAngle(rune_.RtpVec());
                predicted_angle_ = state_.current_angle;   ///< No Predicting When Fitting.
            }else {
                double bullet_delay_time = 7.0 / (bullet_speed_-1);   ///< Ballistic Time Compensation
                double compensation_time = 0;

                /*
                 * compensation_time_tuning
                 */
                if (bullet_speed_ == 30)
                    compensation_time = kCompensateTime30;
                else if(bullet_speed_ == 18)
                    compensation_time = kCompensateTime18;
                else if(bullet_speed_ == 15)
                    compensation_time = kCompensateTime15;


                auto rotated_radian =
                        rotational_speed_.AngularIntegral(
                                rune_.CurrentTime() + bullet_delay_time + compensation_time)
                        - rotational_speed_.AngularIntegral(rune_.CurrentTime());
                rotated_angle = rotated_radian * 180 / CV_PI;
                state_.UpdateAngle(rune_.RtpVec());
                DLOG(INFO) << "clock_wise: " << rune_.Clockwise();

                predicted_angle_ = state_.current_angle - rune_.Clockwise() * rotated_angle;
            }
        }
    }

        // Aim mode is small rune.
    else if (aim_mode == kSmallRune) {

        double bullet_delay_time = 7.0 / (bullet_speed_-1);  // Ballistic Time Compensation.
        double compensation_time = 0;

        /*
         * compensation_time_tuning
         */
        if (bullet_speed_ == 30)
            compensation_time = kCompensateTime30;
        else if(bullet_speed_ == 18)
            compensation_time = kCompensateTime18;
        else if(bullet_speed_ == 15)
            compensation_time = kCompensateTime15;

        auto rotated_radian = rune_.Clockwise() * rotational_speed_.w * (compensation_time + bullet_delay_time);
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

/// [checked] Get point from predict angle.
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
                                         cv::Point2f &fixed_point,
                                         float bullet_speed) {
    if (rune.ArmorCenterP() == cv::Point2f(0, 0)) {
        DLOG(INFO) << "Rune Predictor receive zeros detection result.";
        yaw = pitch = delay = 0;
    }

    int delta_u = RunePredictorDebug::Instance().DeltaU() - 2000;  ///< Horizontal ballistic compensation.
    int delta_v = RunePredictorDebug::Instance().DeltaV() - 2000;  ///< Vertical ballistic compensation.




    while (predicted_angle < 0) predicted_angle += 360;
    while (predicted_angle > 360) predicted_angle -= 360;

    /*
     *  Force offset
     */
    if(bullet_speed == 18)
    {
        if(predicted_angle >= 180 && predicted_angle <= 360)
        {
            delta_u -= 10;
            delta_v += 5;
        }
        if(predicted_angle >90 && predicted_angle < 180)
        {
            delta_u -= 5;
        }
    }


    fixed_point = predicted_point + cv::Point2f(static_cast<float>(delta_u), static_cast<float>(delta_v));

//    DLOG(WARNING) << "delta_u: " << delta_u << " delta_v" << delta_v;
//    DLOG(WARNING) << "predicted_point.x: " << predicted_point.x << " predicted point.y: " << predicted_point.y;
//    DLOG(WARNING) << "fixed_point.x: " << fixed_point.x << " fixed point.y: " << fixed_point.y;
//    DLOG(WARNING) << "rune.ImageCenter.x: " << rune.ImageCenter().x << " rune.ImageCenter.y: " << rune.ImageCenter().y;


    // Use SIMD atan2 for 4x floats.
    float x[4] = {kP_yaw, kP_pitch, 1, 1},
            y[4] = {fixed_point.x - rune.ImageCenter().x, fixed_point.y - rune.ImageCenter().y, 1, 1},
            z[4] = {0};
    algorithm::Atan2FloatX4(y, x, z);

    yaw = abs(z[0]) < .1f ? z[0] : .00005f, pitch = abs(z[1]) < .1f ?  z[1] : .00005f;  // Avoid excessive offset

//    DLOG(INFO) << "Output yaw: " << yaw << ", pitch: " << pitch << ".";
    delay = 1.f;
}

/// [checked] Check if fan changed.
bool predictor::rune::State::FanChanged() {
    if (std::abs(last_angle - 0.0) < 0.001) {
        last_angle = current_angle;
        return false;
    } else {
        double delta_angle = std::abs(current_angle - last_angle);
        last_angle = current_angle;
        if (delta_angle > 60 && delta_angle < 350)
        {
            DLOG(INFO) << "Rune fan changes now.";
            fan_changed_time_chrono = std::chrono::high_resolution_clock::now();
            return true;
        }

    }
    return false;
}

/// [checked]
double predictor::rune::RotationalSpeed::AngularIntegral(double integral_time) const {
    constexpr double c = 0;
    // Angular displacement integral
    return -a / w * cos(w * integral_time + p) + b * integral_time + c;
}

/// [checked]
void predictor::rune::FittingData::Fit(RotationalSpeed &rotational_speed) {
    /*
     * Keep and update observation data num in the vector.
     */
    // Fixme just for the case that 'prepare' and 'observe' are different
    if (!first_fit && palstance.size() == kObservationDataNum + kBufferDataNum)
    {
        auto buffer_begin_palstance = palstance.begin();
        auto buffer_end_palstance   = palstance.begin() + kBufferDataNum;
        auto buffer_begin_time      = time.begin();
        auto buffer_end_time                  = time.begin() + kBufferDataNum;
        palstance.erase(buffer_begin_palstance, buffer_end_palstance);
        time.erase(buffer_begin_time, buffer_end_time);
    }


    // Fixme This is made for fit specific num of data in one period.
//    if (!first_fit)
//    {
//        palstance.clear();
//        time.clear();
//        first_fit  = true;
//    }

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
        options.max_num_iterations           = kMaxNumIterations;
        options.linear_solver_type           = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        fit_complete_time = std::chrono::high_resolution_clock::now();

        /*
         * Result
         */
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        DLOG(INFO) << summary.BriefReport();
        DLOG(INFO) << "Fitting: a = " << rotational_speed.a
                   << ", w = "        << rotational_speed.w
                   << ", phi = "      << rotational_speed.p;

        /*
         * Optimize the parameter
         */
//        if(!first_fit)
//        {
//            //Fixme adjust a, w, p. Recommend to add reserved parameter.
//        }

        first_fit = false;
    }
}

/*
 * @Breif: 计算两点距离函数
 */

float predictor::rune::State::CalcPointsDistance(const cv::Point2f &point1, const cv::Point2f &point2) {
    return std::sqrt(std::pow((point1.x - point2.x), 2) + std::pow((point1.y - point2.y), 2));
}

/*
 * @Breif: 计算矢量夹角函数
 */

float predictor::rune::State::CalcVectorsAngle(const cv::Point2f &first_vector, const cv::Point2f &second_vector) {
    //余弦定理求角度
    float angle = first_vector.dot(second_vector) /
                  (CalcPointsDistance(first_vector, cv::Point2f(0.0, 0.0)) *
                   CalcPointsDistance(second_vector, cv::Point2f(0.0, 0.0)));
    return acos(angle) * (180.0 / CV_PI);
}


void predictor::rune::RunePredictor::AutoFire()
{
    output_data_.fire = 0;
    if(rune::kAutoFireFlag)
    {
        /// auto fire.
        fan_changed = state_.FanChanged();
        if (fan_changed)
            auto_fire_signal_ = true;

        auto current_time_chrono = std::chrono::high_resolution_clock::now();
        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(current_time_chrono - state_.fan_changed_time_chrono)).count();

        if (auto_fire_signal_ && time_gap >= kAutoFireTimeGap && time_gap <= kAutoFireTimeGap + 200 )
        {
            output_data_.fire = 1;
            DLOG(INFO) << "Fire now !!!!!!!!";
        }
        if(time_gap>=600)
        {
            auto_fire_signal_ = false;
        }
    }
}