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
    // a = -f*v^2/m
    // (a->) = -c*|v|*(v->)
    return -coefficient * v.norm() * v;
}

[[maybe_unused]] void BallisticModel::SetParam(const AirResistanceModel &model, double p) {
    air_resistance_model = model;
    constexpr double deg2rad = CV_PI / 180;
    double p_rad = p * deg2rad, s_1 = sin(p_rad), s_2 = sin(2 * p_rad);
    // Suppose that ALTITUDE = 0.
    // Visit https://en.wikipedia.org/wiki/Gravity_of_Earth for more information.
    g = 9.78 * (1 + 0.0052884 * s_1 * s_1 - 0.0000059 * s_2 * s_2);
}

[[maybe_unused]] Eigen::Vector2d BallisticModel::operator()(const Eigen::Vector2d &v) const {
    auto acceleration = air_resistance_model(v);
    acceleration[1] -= g;

    // total a = a_f - g.
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
    // Set up the solver where y is velocity, t is time and h is the step size.
    solver.f = TrajectoryDifferentialEquation();
    solver.f.SetParam(model);
    solver.h = (_end_t - _start_t) / _iter;
    solver.t = _start_t;
    solver.y = {_start_v * cos(_theta),
                _start_v * sin(_theta)};
    // Set up the initial values.
    start_h = _start_h;
    start_t = _start_t;
    start_v = _start_v;
    iter = _iter;

}

[[maybe_unused]] void TrajectorySolver::UpdateParam(double _theta) {
    solver.t = start_t;
    solver.y = {start_v * cos(_theta),
                start_v * sin(_theta)};
}

[[maybe_unused]] bool TrajectorySolver::Solve(double target_h, std::vector<TrajectoryResult> &solutions) {
    solutions.clear();
    double last_h, current_h = start_h;
    Eigen::Vector2d current_x = {0, start_h};
    current_x += solver.y * solver.h;
    unsigned int i = 0;
    // There must be at most 2 solutions and the height must be positive.
    for (; i < iter && current_x.y() > 0 && solutions.size() < 2; ++i) {
        solver.Iterate();
        last_h = current_h;
        current_h += solver.y.y() * solver.h;
        current_x = current_x + solver.y * solver.h;
        // The intermediate value theorem.
        if ((target_h - last_h) * (target_h - current_h) < 0)
            solutions.push_back({solver.t - solver.h / 2, solver.y, current_x});
    }
    if (i == iter)
        LOG(WARNING) << "Time limit exceeded when solving the trajectory. The result maybe wrong or incomplete.";
    if (solutions.empty()) {
        solutions.push_back({solver.t, solver.y, current_x});
        return false;
    }
    return true;
}

[[maybe_unused]] void PitchAngleSolver::SetParam(const BallisticModel &model,
                                                 double _start_v, double _start_h,
                                                 double _target_h, double _target_x) {
    if (_target_h < 0)
        LOG(WARNING) << "Detected negative _target_h, the result may be wrong.";
    ballistic_model = model;
    start_v = _start_v;
    start_h = _start_h;
    target_h = _target_h;
    target_x = _target_x;
}

[[maybe_unused]] void PitchAngleSolver::UpdateParam(double _target_h, double _target_x) {
    if (_target_h < 0)
        LOG(WARNING) << "Detected negative _target_h, the result may be wrong.";
    target_h = _target_h;
    target_x = _target_x;
}

[[maybe_unused]] Eigen::Vector2d
PitchAngleSolver::Solve(double min_theta, double max_theta, double max_error, unsigned int max_iter) {
    TrajectorySolver solver;
    TrajectoryResult min_error_result;
    unsigned int n = 1;
    double mid_theta, error = target_x, min_error = target_x, min_error_theta;
#if NDEBUG
    solver.SetParam(ballistic_model, 0, 4, start_v, start_h, mid_theta, 8192);
#else
    solver.SetParam(ballistic_model, 0, 4, start_v, start_h, mid_theta, 2048);
#endif
    do {
        mid_theta = (max_theta + min_theta) * 0.5;
        solver.UpdateParam(mid_theta);
        std::vector<TrajectoryResult> solutions;
        // If reached the target's height.
        if (solver.Solve(target_h, solutions)) {
            TrajectoryResult *result;
            // The target is higher, there must be 2 solutions.
            if (solutions.size() == 2) {
                // The 2nd solution still not reach the target,
                //   use the farther solution and increase theta.
                if (solutions[1].x.x() < target_x) {
                    result = &solutions[1];
                    min_theta = mid_theta;
                    error = target_x - result->x.x();
                    DLOG(INFO) << "iter: " << n << ", type: LOB A, theta: " << mid_theta
                               << ", x: " << result->x.x() << ", error: " << error << ".";
                }
                    // The 1st solution still farther than the target,
                    //   use the nearer solution and also increase theta.
                else if (solutions[0].x.x() > target_x) {
                    result = &solutions[0];
                    min_theta = mid_theta;
                    error = target_x - result->x.x();
                    DLOG(INFO) << "iter: " << n << ", type: LOB C, theta: " << mid_theta
                               << ", x: " << result->x.x() << ", error: " << error << ".";
                }
                    // The target is between the 2 solutions,
                    //   use the nearer solution (to reduce the error) and decrease theta.
                else {
                    result = &solutions[0];
                    max_theta = mid_theta;
                    error = target_x - result->x.x();
                    DLOG(INFO) << "iter: " << n << ", type: LOB B, theta: " << mid_theta
                               << ", x: " << result->x.x() << ", error: " << error << ".";
                }
            }
                // The target is lower, only get one solution.
            else {
                // The current iteration only considers the case with smaller theta.
                // If theta is big enough, the update method of theta will be the reverse.
                // FIXME Check the cases with bigger theta.
                result = &solutions[0];
                error = target_x - result->x.x();
                if (error > 0)
                    min_theta = mid_theta;
                else
                    max_theta = mid_theta;
                DLOG(INFO) << "iter: " << n << ", type: FLAT, theta: " << mid_theta
                           << ", x: " << result->x.x() << ", error: " << error << ".";
            }
            if (abs(error) < abs(min_error)) {
                min_error = error;
                min_error_theta = mid_theta;
                min_error_result = *result;
            }
        }
            // The target is higher and max height is still lower than the target, so get no solution.
        else {
            min_theta = mid_theta;
            DLOG(WARNING) << "iter: " << n << ", type: LOB D, theta: " << mid_theta
                          << ", x: " << solutions[0].x.x() << ", error: " << error << ".";
        }
    } while ((error > max_error || error < -max_error) && n++ < max_iter);
    DLOG(INFO) << "Selected theta: " << min_error_theta << ", error: " << min_error << ".";
    return {min_error_theta, min_error_result.t};
}
