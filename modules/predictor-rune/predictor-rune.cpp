#include "predictor-rune.h"
#include <ceres/ceres.h>
#include <detector-rune-debug/detector_rune_debug.h>

predictor::rune::RunePredictor::RunePredictor() : debug_(true),
                                                  rune_(), state_(), rotational_speed_(), fitting_data_(),
                                                  output_data_(), bullet_speed_(15),
                                                  predicted_angle_(), predicted_point_() {}

SendPacket predictor::rune::RunePredictor::Run(const PowerRune &power_rune, AimModes aim_mode, float bullet_speed) {
    rune_ = power_rune;
    bullet_speed_ = bullet_speed;
    bullet_speed_ = 30;
    if (aim_mode == kBigRune) {
        LOG(INFO) << "Rune predictor mode: big.";
        state_.UpdatePalstance(rune_, fitting_data_);
        state_.UpdateAngle(rune_.RtgVec());
        PredictAngle(aim_mode);
        PredictPoint();
        output_data_.Update(debug_, rune_, predicted_angle_, predicted_point_, fixed_point_);
        state_.CheckMode();
    } else {
        LOG(INFO) << "Rune predictor mode: small.";
        rotational_speed_.w = 1.04717;
        PredictAngle(aim_mode);
        PredictPoint();
        output_data_.Update(debug_, rune_, predicted_angle_, predicted_point_, fixed_point_);
        state_.CheckMode();
    }

    return {output_data_.yaw, output_data_.pitch, output_data_.delay,
            static_cast<int>(output_data_.yaw + output_data_.pitch + output_data_.delay)};
}

double predictor::rune::RotationalSpeed::Integral(double integral_time) const {
    constexpr double c = 0;
    return -rotational_direction * (a / w * cos(w * integral_time + p) - b * integral_time - c);
}

void predictor::rune::OutputData::Update(bool debug,
                                         const PowerRune &rune,
                                         double predicted_angle,
                                         const cv::Point2f &predicted_point,
                                         cv::Point2f &fixed_point) {
    if (rune.ArmorCenterP() == cv::Point2f(0, 0)) {
        if (debug)
            DLOG(INFO) << "Rune Predictor receive zero detection result.";
        yaw = pitch = delay = 0;
    }
    if (debug)
        DLOG(INFO) << "Predicted angle: " << predicted_angle << "-------";
    int delta_u = RuneDetectorDebug::Instance().DeltaU() - 50,
            delta_v = RuneDetectorDebug::Instance().DeltaV() - 50;  ///< Horizontal and vertical ballistic compensation.

    while (predicted_angle < 0) predicted_angle += 360;
    while (predicted_angle > 360) predicted_angle -= 360;

//    ///< Ballistic Compensation for every 45 degrees
//    if (predicted_angle >= 0 && predicted_angle < 45) {
//        delta_v += 20;
//        delta_u += 0;
//    } else if (predicted_angle < 90) {
//        delta_u += 0;
//        delta_v += 10;
//    } else if (predicted_angle < 135) {
//        delta_u += 10;
//        delta_v += 10;
//    } else if (predicted_angle < 180) {
//        delta_u -= 20;
//        delta_v += 20;
//    } else if (predicted_angle < 225) {
//        delta_u -= 10;
//        delta_v += 15;
//    } else if (predicted_angle < 270) {
//        delta_u -= 10;
//        delta_v += 20;
//    } else if (predicted_angle < 315) {
//        delta_u -= 5;
//        delta_v += 15;
//    } else {
//        delta_u -= 10;
//        delta_v += 10;
//    }

    fixed_point = predicted_point + cv::Point2f(static_cast<float>(delta_u), static_cast<float>(delta_v));

    float x[4] = {1350, 1350, 1, 1},
            y[4] = {fixed_point.x - rune.ImageCenter().x, fixed_point.y - rune.ImageCenter().y, 1, 1},
            z[4] = {0};
    algorithm::Atan2FloatX4(y, x, z);
    yaw = z[0], pitch = z[1];

    if (debug)
        DLOG(INFO) << "Output yaw: " << yaw << ", pitch: " << pitch << ".";
    delay = 1.f;    ///< Output Data's Time Delay
}

void predictor::rune::FittingData::Fit(bool debug, RotationalSpeed &rotational_speed) {
//    if (!outdated)  // Fitting is error.
//        return;
    if (!ready) {
        // Preparing, wait for some time.
        if (palstance.size() >= kPreparePalstanceDataNum) {
            ready = true;          // Ready to fit.
            palstance.clear();
        }
    } else {
        // Collect data and fit.
        if (palstance.size() >= kCollectPalstanceDataNum) {
            for (double &speed: palstance)
                speed = speed / 180 * CV_PI;

            ceres::Problem problem;
            for (int i = 0; i < kResidualBlockNum; ++i)
                problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<TrigonometricResidual, 1, 1, 1, 1>(
                                new TrigonometricResidual(time[i], palstance[i],
                                                          rotational_speed.rotational_direction)),
                        new ceres::CauchyLoss(0.5), &rotational_speed.a,
                        &rotational_speed.w, &rotational_speed.p);

            ceres::Solver::Options options;
            options.max_num_iterations = 300;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            if (debug) {
                DLOG(INFO) << summary.BriefReport();
                DLOG(INFO) << "Fitting: a = " << rotational_speed.a
                           << ", w = " << rotational_speed.w
                           << ", phi = " << rotational_speed.p;
            }
            ready = false;
            outdated = false;      // Now fitting may be right.
        }
    }
}

void predictor::rune::State::UpdateAngle(const cv::Point2f &rtg_vec) {
    // Calculate Radian angle.
    auto rad = algorithm::Atan2Float(rtg_vec.y, rtg_vec.x);

    // Make positive radian for quadrant III or IV.
    if (rad < 0)
        rad += CV_2PI;
    current_angle = rad * 180 / CV_PI;

    // Fix current angle to principal value interval (0~360).
    while (current_angle < 0) current_angle += 360;
    while (current_angle > 360) current_angle -= 360;
}

bool predictor::rune::State::UpdatePalstance(const PowerRune &rune, FittingData &fitting_data) {
    // Ignore discarded data. Here use rtg, may need change to rtp.
    if (rune.CenterR() == cv::Point2f(0, 0)) {
        last_rtg_vec = cv::Point2f(0, 0);
        return false;
    }

    float angle;  ///< Temp angle var in DEGREE.
    auto current_time_chrono = std::chrono::high_resolution_clock::now();
    current_time = double(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    current_time *= 1e-3;  // Convert millisecond to second.

    // Filter invalid data.
    if (last_rtg_vec != cv::Point2f(0, 0)) {
        angle = algorithm::VectorAngle(last_rtg_vec, rune.RtgVec());  // Calculate angle in DEGREE.
        angle = std::min(160.f, std::max(.2f, angle));

        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(
                current_time_chrono - last_time)).count();
        current_palstance = angle / time_gap * 1e3;
        DLOG(INFO) << "palstance:" << current_palstance;

        // Update palstance data.
        fitting_data.palstance.push_back(current_palstance);
        fitting_data.time.push_back(current_time);
        last_time = current_time_chrono;
    }

    last_rtg_vec = rune.RtgVec();
    return true;
}

void predictor::rune::State::CheckMode() {
    if (std::abs(last_angle - 0.0) < 0.001) {
        last_angle = current_angle;
    } else {
        double delta_angle = std::abs(current_angle - last_angle);
        last_angle = current_angle;
        if (delta_angle > 60 && delta_angle < 350)
            LOG(INFO) << "Rune fan changes now.";
    }
}

bool predictor::rune::RunePredictor::Initialize(const std::string &config_path, bool debug) {
    cv::FileStorage config;

    // Open config file.
    config.open(config_path, cv::FileStorage::READ);
    if (!config.isOpened()) {
        LOG(ERROR) << "Failed to open rune detector config file " << config_path << ".";
        return false;
    }

    debug_ = debug;
    config["AMPLITUDE"] >> rotational_speed_.a;
    config["PALSTANCE"] >> rotational_speed_.w;
    config["PHASE"] >> rotational_speed_.p;
    rotational_speed_.b = 2.090 - rotational_speed_.a;

    return true;
}

void predictor::rune::RunePredictor::PredictAngle(AimModes aim_mode) {
    double rotated_angle;
    if (aim_mode == kBigRune) {
        if (fitting_data_.outdated) {
            fitting_data_.Fit(debug_, rotational_speed_);
            state_.UpdateAngle(rune_.RtgVec());
            predicted_angle_ = state_.current_angle;   // No Predicting When Fitting.
        } else {
            double delay_time = 7.0 / bullet_speed_;   // Ballistic Time Compensation
            double compensate_time = 0.05;
            auto rotated_radian = rotational_speed_.Integral(state_.current_time + delay_time + compensate_time)
                                  - rotational_speed_.Integral(state_.current_time);
            rotated_angle = rotated_radian * 180 / CV_PI;

            state_.UpdateAngle(rune_.RtgVec());
            predicted_angle_ = rune_.Clockwise() * (state_.current_angle + rotated_angle);
        }
    } else {
        double delay_time = 7.0 / bullet_speed_;  // Ballistic Time Compensation.
        double compensate_time = 0.05;   // Other compensation

        auto rotated_radian = rune_.Clockwise() * rotational_speed_.w * (compensate_time + delay_time);
        rotated_angle = rotated_radian * 180 / CV_PI;

        state_.UpdateAngle(rune_.RtpVec());
        predicted_angle_ = rune_.Clockwise() * (state_.current_angle + rotated_angle);
    }
}

void predictor::rune::RunePredictor::PredictPoint() {
    auto radius = algorithm::SqrtFloat(rune_.RtpVec().x * rune_.RtpVec().x
                                       + rune_.RtpVec().y * rune_.RtpVec().y);
    auto predicted_rad = static_cast<float>(predicted_angle_ * CV_PI / 180);
    predicted_point_ = rune_.CenterR() + cv::Point2f(algorithm::CosFloat(predicted_rad) * radius,
                                                     algorithm::SinFloat(predicted_rad) * radius);
}
