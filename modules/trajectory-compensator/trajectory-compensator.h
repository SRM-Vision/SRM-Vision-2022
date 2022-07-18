#ifndef TRAJECTORY_COMPENSATOR_H_
#define TRAJECTORY_COMPENSATOR_H_

#include "digital-twin/components/armor.h"
#include "trajectory-solver/trajectory-solver.h"

namespace compensator {
    class CompensatorTraj {
    public:
        bool Initialize(const std::string &type);

        [[nodiscard]] Eigen::Vector3d Solve(
                double bullet_speed, const Armor &armor,
                double min_theta = -CV_PI / 2, double max_theta = CV_PI / 2,
                double max_error = 0.01, unsigned int max_iter = 16) const;

    private:
        trajectory_solver::BallisticModel ballistic_model;
        double pnp_map_coefficients[4];
    };
}

#endif  // TRAJECTORY_COMPENSATOR_H_
