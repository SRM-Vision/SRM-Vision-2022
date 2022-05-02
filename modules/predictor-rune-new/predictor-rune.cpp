#include "predictor-rune.h"

double predictor::rune::RotationalSpeed::Integral(double integral_time) {
    constexpr double c = 0;
    // -1 * rotational_direction * (-a / w * cos(w * integral_time + p) + b * integral_time + c)
    return rotational_direction * (a / w * cos(w * integral_time + p) - b * integral_time - c);
}

void predictor::rune::FittingData::Fit(bool debug, RotationalSpeed &rotational_speed) {
    if (!outdated)
        return;
    if (!ready) {
        // Preparing, wait for some time.
        if (palstance.size() >= kPreparePalstanceDataNum) {
            ready = true;
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
                                new TrigonometricResidual(
                                        time[i], palstance[i],
                                        rotational_speed.rotational_direction)),
                        new ceres::CauchyLoss(0.5),
                        &rotational_speed.a,
                        &rotational_speed.w,
                        &rotational_speed.p);
            ceres::Solver::Options options;
            options.max_num_iterations = 300;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;

            if (debug) {
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                DLOG(INFO) << summary.BriefReport();
                DLOG(INFO) << "Fitting: a = " << rotational_speed.a
                           << ", w = " << rotational_speed.w
                           << ", phi = " << rotational_speed.p;
            }
            ready = false;
            outdated = false;
        }
    }
}

void predictor::rune::State::UpdateAngle(const cv::Point2f &rtp_vec) {
    // Calculate RADIAN angle.
    auto rad = algorithm::Atan2Float(-rtp_vec.y, rtp_vec.x);

    // Make positive radian for quadrant III or IV.
    if (rad < 0)
        rad += CV_2PI;
    current_angle = rad * 180 / CV_PI;

    // Fix current angle to principal value interval (0~360).
    while (current_angle < 0) current_angle += 360;
    while (current_angle > 360) current_angle -= 360;
}

bool predictor::rune::State::UpdatePalstance(const PowerRune &rune,
                                             FittingData &fitting_data) {
    // Ignore discarded data.
    if (rune.CenterR() == cv::Point2f(0, 0)) {
        last_rtg_vec = cv::Point2f(0, 0);
        return false;
    }

    float angle;  ///< Temp angle var in DEGREE.
    current_palstance = -1;
    auto current_time_chrono = std::chrono::high_resolution_clock::now();
    current_time = double(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    current_time *= 0.001;  // Convert millisecond to second.
    // Filter invalid data.
    if (last_rtg_vec != cv::Point2f(0, 0)) {
        angle = algorithm::VectorAngle(last_rtg_vec, rune.RtgVec());  // Calculate angle in DEGREE.
        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(
                current_time_chrono - last_time)).count();
        current_palstance = angle / time_gap * 1000.0;
    }

    // Update time and rtg vector.
    last_time = current_time_chrono;
    last_rtg_vec = rune.RtgVec();

    fitting_data.palstance.push_back(current_palstance);
    fitting_data.time.push_back(current_time);
    return true;
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

SendPacket predictor::rune::RunePredictor::Run(const PowerRune &power_rune, AimModes aim_mode) {
    rune_ = power_rune;
    return {};
}

