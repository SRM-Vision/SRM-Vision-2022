#ifndef TRAJECTORY_SOLVER_H_
#define TRAJECTORY_SOLVER_H_

#include <Eigen/Eigen>
#include "math-tools/rk4-solver.h"

namespace trajectory_solver {
    /**
     * @brief The air resistance function package.
     * @details Use operator () to get the air resistance acceleration.
     */
    class AirResistanceModel {
    public:
        /**
         * @brief Set necessary parameters for calculating air resistance.
         * @param c Const air resistance coefficient, 0.48 for imaginary ball and 0.26 for golf ball.
         * @param p Air pressure, hPa.
         * @param t Air temperature, `C.
         * @param d Diameter of the ball, meter.
         * @param m Mass of the ball, meter.
         */
        [[maybe_unused]] void SetParam(double c, double p, double t, double d, double m);

        /**
         * @brief Calculate the air resistance acceleration.
         * @param v Current 2d velocity, m/s.
         * @return Current 2d air resistance acceleration, m/s^2.
         */
        [[maybe_unused]] Eigen::Vector2d operator()(const Eigen::Vector2d &v) const;

    private:
        double coefficient;  ///< Air resistance coefficient, will be calculated after setting parameters.
    };

    /**
     * @brief Ballistic model function package.
     * @details Use operator () to get the ballistic model acceleration.
     */
    class BallisticModel {
    public:
        /**
         * @brief Set necessary parameters for calculating total acceleration.
         * @param model Air resistance model.
         * @param p Latitude of current position, DEG.
         */
        [[maybe_unused]] void SetParam(const AirResistanceModel &model, double p);

        /**
         * @brief Calculate current total acceleration.
         * @param v Current velocity, m/s.
         * @return Current acceleration, m/s^2.
         */
        [[maybe_unused]] Eigen::Vector2d operator()(const Eigen::Vector2d &v) const;

    private:
        AirResistanceModel air_resistance_model;  ///< Air resistance model.
        double g;  ///< Current gravitational acceleration, m/s^2.
    };

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

    struct TrajectoryResult {
        double t;
        Eigen::Vector2d v, x;
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
         * @param _theta Elevation angle, RAD.
         * @param _iter Iteration times.
         */
        [[maybe_unused]] void SetParam(const BallisticModel &model,
                                       double _start_t, double _end_t,
                                       double _start_v, double _start_h,
                                       double _theta, unsigned int _iter);

        /**
         * @brief Update parameters for solving trajectory.
         * @param _theta Initial elevation angle, RAD.
         */
        [[maybe_unused]] void UpdateParam(double _theta);

        /**
         * @brief Solve the trajectory.
         * @param target_h Height of the target, m.
         * @param [out] solutions Array of all solutions, sorted by x;
         *   or, if no solutions are found, record the last status.
         * @return If there is any solution.
         */
        [[maybe_unused]] bool Solve(double target_h, std::vector<TrajectoryResult> &solutions);

    private:
        unsigned int iter{};
        double start_h{}, start_t{}, start_v{};
        RK4Solver<double, Eigen::Vector2d, TrajectoryDifferentialEquation> solver;
    };

    /**
     * @brief Pitch solver for ballistic trajectory.
     * @datails Use Solve function to get the fixed pitch angle.
     */
    class PitchAngleSolver {
    public:
        /**
         * @brief Set necessary parameters for pitch solver.
         * @param model Ballistic model.
         * @param _start_v Initial 1D velocity, m/s.
         * @param _start_h Initial height, m.
         * @param _target_h Target height, m.
         * @param _target_x Horizontal distance from target, m.
         * @note Due to limits in trajectory solver, _target_h must be positive.
         */
        [[maybe_unused]] void SetParam(const BallisticModel &model,
                                       double _start_v, double _start_h,
                                       double _target_h, double _target_x);

        /**
         * @brief Update target parameters.
         * @param _target_h Target height, m.
         * @param _target_x Horizontal distance from target, m.
         * @note Due to limits in trajectory solver, _target_h must be positive.
         */
        [[maybe_unused]] void UpdateParam(double _target_h, double _target_x);

        /**
         * @brief Solve the pitch angle.
         * @param min_theta Initial pitch angle without fixes, RAD.
         * @param max_theta Maximal pitch angle, RAD.
         * @param max_error Maximal error acceptable, m.
         * @param max_iter Maximal iteration times.
         * @return The fixed pitch angle, RAD; total time, s.
         */
        [[maybe_unused]] Eigen::Vector2d
        Solve(double min_theta, double max_theta, double max_error, unsigned int max_iter);

    private:
        BallisticModel ballistic_model;
        double start_v, start_h, target_h, target_x;
    };
}

#endif  // TRAJECTORY_SOLVER_H_
