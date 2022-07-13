#include <opencv2/core/cvdef.h>
#include <glog/logging.h>
#include "pitch-angle-solver.h"

using namespace bullet_trajectory_solver;

[[maybe_unused]] void PitchAngleSolver::SetParam(const BallisticModel &model,
                                                 double _start_v, double _start_h,
                                                 double _target_h, double _target_x) {
    ballistic_model = model;
    start_v = _start_v;
    start_h = _start_h;
    target_h = _target_h;
    target_x = _target_x;
}

[[maybe_unused]] void PitchAngleSolver::UpdateParam(double _target_h, double _target_x) {
    target_h = _target_h;
    target_x = _target_x;
}

[[maybe_unused]] double PitchAngleSolver::Solve(double min_theta, double max_theta, double max_error, unsigned int max_iter) {
    TrajectorySolver solver;
    unsigned int n = 1;
    double mid_theta, error, t;
#if NDEBUG
    solver.SetParam(ballistic_model, 0, 8, start_v, start_h, mid_theta, 8192);
#else
    solver.SetParam(ballistic_model, 0, 8, start_v, start_h, mid_theta, 2048);
#endif
    do {
        Eigen::Vector2d x, v;
        mid_theta = (max_theta + min_theta) * 0.5;
        solver.UpdateParam(mid_theta);
        if (solver.Solve(target_h, t, v, x)) {
            error = target_x - x.x();
            DLOG(INFO) << "iter: " << n << ", theta: " << mid_theta << ", x: " << x.x() << ", error: " << error << ".";
        } else
            return mid_theta;
        if (error * v.y() > 0)
            max_theta = mid_theta;
        else
            min_theta = mid_theta;
    } while ((error > max_error || error < -max_error) && n++ < max_iter);
    return mid_theta;
}
