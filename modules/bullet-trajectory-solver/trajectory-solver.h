#ifndef TRAJECTORY_SOLVER_H_
#define TRAJECTORY_SOLVER_H_

#include "ballistic-model.h"
#include "math-tools/rk4-solver.h"

namespace bullet_trajectory_solver {
    class TrajectoryDifferentialEquation {
    public:
        void SetParam(const BallisticModel &model);

        Eigen::Vector2d operator()(double t, Eigen::Vector2d v) const;

    private:
        BallisticModel ballistic_model;
    };

    class TrajectorySolver {
    public:
        [[maybe_unused]] void SetParam(const BallisticModel &model,
                                       double _start_t, double _end_t,
                                       double _start_v, double _start_h,
                                       double _l, double _theta,
                                       unsigned int _iter);

        [[maybe_unused]] bool Solve(double target_h, double &t,
                                    Eigen::Vector2d &v, Eigen::Vector2d &x);

    private:
        unsigned int iter{};
        double start_h{}, l{}, theta{};
        RK4Solver<double, Eigen::Vector2d, TrajectoryDifferentialEquation> solver;
    };
}

#endif  // TRAJECTORY_SOLVER_H_
