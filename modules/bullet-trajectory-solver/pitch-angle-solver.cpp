#include <opencv2/core/cvdef.h>
#include <glog/logging.h>
#include "pitch-angle-solver.h"

using namespace bullet_trajectory_solver;

void PitchAngleSolver::SetParam(const BallisticModel &model,
                                double _start_v, double _start_h,
                                double _target_h, double _target_x,
                                double _l) {
    ballistic_model = model;
    start_v = _start_v;
    start_h = _start_h;
    target_h = _target_h;
    target_x = _target_x;
    l = _l;
}

void PitchAngleSolver::UpdateParam(double _target_h, double _target_x) {
    target_h = _target_h;
    target_x = _target_x;
}

double PitchAngleSolver::Solve(double theta, double min_error) {
    TrajectorySolver solver;
    double theta_higher = CV_PI * 0.5, theta_lower = theta, theta_d, error, t;
    do {
        Eigen::Vector2d x, v;
        theta_d = (theta_higher + theta_lower) * 0.5;
        solver.SetParam(ballistic_model, 0, 4, start_v, start_h, l, theta_d, 1024);
        if (solver.Solve(target_h, t, v, x)) {
            error = target_x - x.x();
            DLOG(INFO) << "theta: " << theta_d << " x: " << x.x() << " error: " << error << ".";
        } else
            return theta;
        if (error < 0)
            theta_higher = theta_d;
        else
            theta_lower = theta_d;
    } while (error > min_error || error < -min_error);
    return theta_d;
}
