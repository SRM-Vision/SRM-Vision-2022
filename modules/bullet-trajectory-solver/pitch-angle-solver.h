#ifndef PITCH_ANGLE_SOLVER_H_
#define PITCH_ANGLE_SOLVER_H_

#include "trajectory-solver.h"

namespace bullet_trajectory_solver {
    class PitchAngleSolver {
    public:
        void SetParam(const BallisticModel &model,
                      double _start_v, double _start_h,
                      double _target_h, double _target_x,
                      double _l);

        void UpdateParam(double _target_h, double _target_x);

        double Solve(double theta, double min_error, unsigned int max_iter);

    private:
        BallisticModel ballistic_model;
        double start_v, start_h,
                target_h, target_x, l;
    };
}

#endif  // PITCH_ANGLE_SOLVER_H_
