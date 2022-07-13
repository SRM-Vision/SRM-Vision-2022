#ifndef PITCH_ANGLE_SOLVER_H_
#define PITCH_ANGLE_SOLVER_H_

#include "trajectory-solver.h"

namespace bullet_trajectory_solver {
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
         */
        [[maybe_unused]] void SetParam(const BallisticModel &model,
                                       double _start_v, double _start_h,
                                       double _target_h, double _target_x);

        /**
         * @brief Update target parameters.
         * @param _target_h Target height, m.
         * @param _target_x Horizontal distance from target, m.
         */
        [[maybe_unused]] void UpdateParam(double _target_h, double _target_x);

        /**
         * @brief Solve the pitch angle.
         * @param theta Initial pitch angle without fixes, RADIUS.
         * @param max_error Maximal error acceptable, m.
         * @param max_iter Maximal iteration times.
         * @return The fixed ELEVATION angle, RADIUS,
         */
        [[maybe_unused]] double Solve(double theta, double max_error, unsigned int max_iter);

    private:
        BallisticModel ballistic_model;
        double start_v, start_h, target_h, target_x;
    };
}

#endif  // PITCH_ANGLE_SOLVER_H_
