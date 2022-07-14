#include <glog/logging.h>
#include "math-tools/algorithms.h"
#include "trajectory-solver.h"

using namespace trajectory_solver;
using namespace algorithm;

[[maybe_unused]] void AirResistanceModel::SetParam(double c, double p, double t, double d, double m) {
    double rho = 1.293 * (p / 1013.25) * (273.15 / (273.15 + t));  ///< Air density, kg/m^3.
    double S = 0.25 * d * d;  ///< Frontal area, m^2.
    coefficient = 0.5 * c * rho * S / m;  // Visit https://en.wikipedia.org/wiki/Drag_equation for details.
}

[[maybe_unused]] Eigen::Vector2d AirResistanceModel::operator()(const Eigen::Vector2d &v) const {
    return -coefficient * v.norm() * v;
}

[[maybe_unused]] void BallisticModel::SetParam(const AirResistanceModel &model, double p) {
    air_resistance_model = model;
    constexpr float deg2rad = CV_PI / 180;
    float p_deg = static_cast<float>(p) * deg2rad;
    float s[4] = {p_deg, 2 * p_deg};
    algorithm::SinFloatX4(s);
    g = 9.78 * (1 + 0.0052884 * s[0] * s[0] - 0.0000059 * s[1] * s[1]);
}

[[maybe_unused]] Eigen::Vector2d BallisticModel::operator()(const Eigen::Vector2d &v) const {
    auto acceleration = air_resistance_model(v);
    acceleration[1] -= g;
    return acceleration;
}

[[maybe_unused]] void TrajectoryDifferentialEquation::SetParam(const BallisticModel &model) {
    ballistic_model = model;
}

[[maybe_unused]] Eigen::Vector2d TrajectoryDifferentialEquation::operator()(double t, Eigen::Vector2d v) const {
    return ballistic_model(v);
}

[[maybe_unused]] void TrajectorySolver::SetParam(
        const BallisticModel &model,
        double _start_t, double _end_t,
        double _start_v, double _start_h, double _theta,
        unsigned int _iter) {
    solver.f = TrajectoryDifferentialEquation();
    solver.f.SetParam(model);
    solver.h = (_end_t - _start_t) / _iter;
    solver.t = _start_t;
    solver.y = {_start_v * CosFloat(static_cast<float>(_theta)),
                _start_v * SinFloat(static_cast<float>(_theta))};
    start_h = _start_h;
    start_t = _start_t;
    start_v = _start_v;
    iter = _iter;
}

[[maybe_unused]] void TrajectorySolver::UpdateParam(double _theta) {
    solver.t = start_t;
    solver.y = {start_v * CosFloat(static_cast<float>(_theta)),
                start_v * SinFloat(static_cast<float>(_theta))};
}

[[maybe_unused]] bool TrajectorySolver::Solve(
        double target_h, double &t, Eigen::Vector2d &v, Eigen::Vector2d &x) {
    double last_h, current_h = start_h;
    Eigen::Vector2d current_x = {0, start_h};
    current_x += solver.y * solver.h;
    for (unsigned int i = 0; i < iter; ++i) {
        solver.Iterate();
        last_h = current_h;
        current_h += solver.y.y() * solver.h;
        current_x = current_x + solver.y * solver.h;
        if ((target_h - last_h) * (target_h - current_h) < 0) {
            t = solver.t;
            v = solver.y;
            x = current_x;
            return true;
        }
    }
    return false;
}

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
