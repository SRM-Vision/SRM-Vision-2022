#include <iostream>
#include <glog/logging.h>
#include <opencv2/core/cvdef.h>
#include "bullet-trajectory-solver/pitch-angle-solver.h"

using namespace bullet_trajectory_solver;
using namespace std;

int main() {
    auto f = AirResistanceModel();
    f.SetParam(0.48, 995, 32, 0.017, 0.0032);
    auto a = BallisticModel();
    a.SetParam(f, 31);
    auto solver = PitchAngleSolver();
    solver.SetParam(a, 14, 0.37, 0.18, 1, 0.11);
    constexpr double deg2rad = CV_PI / 180;
    for (double target_x = 2; target_x < 5; target_x += 0.05) {
        solver.UpdateParam(0.18, target_x);
        double theta = solver.Solve(-15 * deg2rad, 0.01, 16);
        LOG(INFO) << "target: " << target_x << " m, angle: " << theta / deg2rad << " deg.";
        auto traj_solver = TrajectorySolver();
        traj_solver.SetParam(a, 0, 4, 14, 0.37, 0.11, theta, 1024);
        double t;
        Eigen::Vector2d v, x;
        if (traj_solver.Solve(0.18, t, v, x))
            LOG(INFO) << "t: " << t << ", v: " << v.norm() << ", x: " << x.x();
    }
}
