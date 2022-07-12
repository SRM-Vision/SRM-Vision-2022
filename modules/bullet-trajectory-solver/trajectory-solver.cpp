#include "math-tools/algorithms.h"
#include "trajectory-solver.h"

using namespace bullet_trajectory_solver;
using namespace algorithm;

[[maybe_unused]] void TrajectoryDifferentialEquation::SetParam(const BallisticModel &model) {
    ballistic_model = model;
}

[[maybe_unused]] Eigen::Vector2d TrajectoryDifferentialEquation::operator()(double t, Eigen::Vector2d v) const {
    return ballistic_model(v);
}

[[maybe_unused]] void TrajectorySolver::SetParam(
        const BallisticModel &model,
        double _start_t, double _end_t,
        double _start_v, double _start_h, double _l, double _theta,
        unsigned int _iter) {
    solver.f = TrajectoryDifferentialEquation();
    solver.f.SetParam(model);
    solver.h = (_end_t - _start_t) / _iter;
    solver.t = _start_t;
    solver.y = {_start_v * CosFloat(static_cast<float>(_theta)),
                _start_v * SinFloat(static_cast<float>(_theta))};
    l = _l;
    theta = _theta;
    start_h = _start_h;
    start_t = _start_t;
    start_v = _start_v;
    iter = _iter;
}

[[maybe_unused]] void TrajectorySolver::UpdateParam(double _theta) {
    solver.t = start_t;
    solver.y = {start_v * CosFloat(static_cast<float>(_theta)),
                start_v * SinFloat(static_cast<float>(_theta))};
    theta = _theta;
}

[[maybe_unused]] bool TrajectorySolver::Solve(
        double target_h, double &t, Eigen::Vector2d &v, Eigen::Vector2d &x) {
    double last_h, last_t, current_h = start_h, current_t = solver.t;
    Eigen::Vector2d current_x = solver.y * solver.h;
    current_x[0] += l * CosFloat(static_cast<float>(theta));
    current_x[1] += start_h + l * SinFloat(static_cast<float>(theta));
    for (unsigned int i = 0; i < iter; ++i) {
        solver.Iterate();
        last_h = current_h;
        last_t = current_t;
        current_h += solver.y.y() * solver.h;
        current_t = solver.t;
        current_x = current_x + solver.y * solver.h;
        if ((target_h - last_h) * (target_h - current_h) < 0) {
            t = (last_t + current_t) * 0.5;
            v = solver.y;
            x = current_x;
            return true;
        }
    }
    return false;
}
