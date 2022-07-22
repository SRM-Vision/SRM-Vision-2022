#ifndef TRAJECTORY_COMPENSATOR_H_
#define TRAJECTORY_COMPENSATOR_H_

#include "digital-twin/components/armor.h"
#include "trajectory-solver/trajectory-solver.h"

namespace compensator {
    // TODO [@trantuan-20048607] [Refactor] Rename this class.
    class CompensatorTraj {
    public:
        bool Initialize(const std::string &type);

        [[nodiscard]] Eigen::Vector3d AnyTargetOffset(
                double bullet_speed, const Armor &armor, double current_pitch,
                double min_theta = -CV_PI / 2, double max_theta = CV_PI / 2,
                double max_error = 0.01, unsigned int max_iter = 16) const;

    private:
        trajectory_solver::BallisticModel ballistic_model;
        double small_pnp_map[4], big_pnp_map[4];
    };
}

#endif  // TRAJECTORY_COMPENSATOR_H_
