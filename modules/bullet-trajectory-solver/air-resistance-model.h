#ifndef AIR_RESISTANCE_H_
#define AIR_RESISTANCE_H_

#include <Eigen/Eigen>

namespace bullet_trajectory_solver {
    class AirResistanceModel {
    public:
        [[maybe_unused]] void SetParam(double c, double p, double t, double d, double m);

        Eigen::Vector2d operator()(const Eigen::Vector2d &v) const;

    private:
        double coefficient;
    };
}

#endif  // AIR_RESISTANCE_H_
