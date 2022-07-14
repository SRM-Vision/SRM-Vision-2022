#ifndef TRAJECTORY_SOLVER_H_
#define TRAJECTORY_SOLVER_H_

#include "math-tools/rk4-solver.h"
#include "ballistic-model.h"

namespace bullet_trajectory_solver {
    /**
     * @brief Trajectory differential equation for RK4Solver to solve.
     * @details a = dv/dt = f(t, v).
     */
    class TrajectoryDifferentialEquation {
    public:
        /**
         * @brief Set necessary variables for the differential equation.
         * @param model Ballistic model.
         */
        [[maybe_unused]] void SetParam(const BallisticModel &model);

        /**
         * @brief Calculate current acceleration.
         * @param t Current time, s, not used.
         * @param v Current velocity, m/s.
         * @return Total acceleration, m/s^2.
         */
        [[maybe_unused]] Eigen::Vector2d operator()(double t, Eigen::Vector2d v) const;

    private:
        BallisticModel ballistic_model;  ///< Ballistic model.
    };

    /**
     * @brief Trajectory solver.
     * @details Given initial shooting args, this solver can calculate the whole 2d trajectory of the bullet.
     */
    class TrajectorySolver {
    public:
        /**
         * @brief Set necessary parameters for trajectory solver.
         * @param model Ballistic model.
         * @param _start_t Start time, s.
         * @param _end_t End time, s, should be big enough to solve the whole trajectory.
         * @param _start_v Initial velocity, 1D, m/s.
         * @param _start_h Initial height, m.
         * @param _theta Elevation angle, RADIUS.
         * @param _iter Iteration times.
         */
        [[maybe_unused]] void SetParam(const BallisticModel &model,
                                       double _start_t, double _end_t,
                                       double _start_v, double _start_h,
                                       double _theta, unsigned int _iter);

        /**
         * @brief Update parameters for solving trajectory.
         * @param _theta Initial elevation angle, RADIUS.
         */
        [[maybe_unused]] void UpdateParam(double _theta);

        /**
         * @brief Solve the trajectory.
         * @param target_h Height of the target, m.
         * @param [out] t The time point when bullet hits the target.
         * @param [out] v The velocity vector when bullet hits the target.
         * @param [out] x The position vector when bullet hits the target.
         * @return If the solution is valid.
         */
        [[maybe_unused]] bool Solve(double target_h, double &t,
                                    Eigen::Vector2d &v, Eigen::Vector2d &x);

    private:
        unsigned int iter{};
        double start_h{}, start_t{}, start_v{};
        RK4Solver<double, Eigen::Vector2d, TrajectoryDifferentialEquation> solver;
    };
}

#endif  // TRAJECTORY_SOLVER_H_
