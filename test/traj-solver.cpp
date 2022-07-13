#include <iostream>
#include <glog/logging.h>
#include <opencv2/core/cvdef.h>
#include "bullet-trajectory-solver/pitch-angle-solver.h"

using namespace bullet_trajectory_solver;
using namespace std;

int main([[maybe_unused]] int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_log_dir = "../../log";
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_max_log_size = 16;

    auto f = AirResistanceModel();
    f.SetParam(0.48, 995, 32, 0.017, 0.0032);
    auto a = BallisticModel();
    a.SetParam(f, 31);
    auto solver = PitchAngleSolver();
    solver.SetParam(a, 14, 0.37, 0.18, 1);
    constexpr double deg2rad = CV_PI / 180;
    for (double target_x = 2; target_x < 5; target_x += 0.05) {
        timespec t_1{}, t_2{};
        clock_gettime(CLOCK_REALTIME, &t_1);
        solver.UpdateParam(0.18, target_x);
        double theta = solver.Solve(-12 * deg2rad, 0.02, 16);
        clock_gettime(CLOCK_REALTIME, &t_2);
        LOG(INFO) << "target: " << target_x << " m, angle: " << theta / deg2rad
                  << " deg, cost: " << (t_2.tv_nsec - t_1.tv_nsec) << " ns.";
        auto traj_solver = TrajectorySolver();
        traj_solver.SetParam(a, 0, 4, 14, 0.37, theta, 2048);
        double t;
        Eigen::Vector2d v, x;
        if (traj_solver.Solve(0.18, t, v, x))
            LOG(INFO) << "t: " << t << ", v: " << v.norm() << ", x: " << x.x();
    }
}
