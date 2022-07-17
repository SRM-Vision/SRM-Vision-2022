#include <iostream>
#include <chrono>
#include <glog/logging.h>
#include <opencv2/core/cvdef.h>
#include "trajectory-solver/trajectory-solver.h"

using namespace trajectory_solver;
using namespace std;

int main([[maybe_unused]] int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_log_dir = "../../log";
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_max_log_size = 16;

    constexpr double deg2rad = CV_PI / 180;
    auto f = AirResistanceModel();
    f.SetParam(0.26, 994, 30, 0.0425, 0.041);
    auto a = BallisticModel();
    a.SetParam(f, 31);
    auto solver = PitchAngleSolver();
    solver.SetParam(a, 16, 0.35, 1.1, 1);
    LOG(INFO) << "Now testing lob shoot.";
    for (double target_x = 1; target_x <= 10; target_x += 0.5) {
        auto start_time = std::chrono::high_resolution_clock::now();
        solver.UpdateParam(1.1, target_x);
        auto res = solver.Solve(-CV_PI / 2, CV_PI / 2, 0.01, 16);
        auto theta = res.x(), t = res.y(), error = res.z();
        auto end_time = std::chrono::high_resolution_clock::now();
        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(
                end_time - start_time)).count();
        LOG(INFO) << "target: " << target_x << " m, angle: " << theta / deg2rad
                  << " deg, time: " << t << "s, error: " << error * 100 << "%, cost: " << time_gap << " ms.";
    }
    f = AirResistanceModel();
    f.SetParam(0.48, 994, 30, 0.0032, 0.017);
    a = BallisticModel();
    a.SetParam(f, 31);
    solver = PitchAngleSolver();
    solver.SetParam(a, 30, 1.2, 0.18, 1);
    LOG(INFO) << "Now testing flat shoot.";
    for (double target_x = 1; target_x <= 10; target_x += 0.5) {
        auto start_time = std::chrono::high_resolution_clock::now();
        solver.UpdateParam(0.18, target_x);
        auto res = solver.Solve(-CV_PI / 2, CV_PI / 2, 0.01, 16);
        auto theta = res.x(), t = res.y(), error = res.z();
        auto end_time = std::chrono::high_resolution_clock::now();
        double time_gap = (static_cast<std::chrono::duration<double, std::milli>>(
                end_time - start_time)).count();
        LOG(INFO) << "target: " << target_x << " m, angle: " << theta / deg2rad
                  << " deg, time: " << t << "s, error: " << error * 100 << "%, cost: " << time_gap << " ms.";
    }
}
