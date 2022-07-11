#include "air-resistance-model.h"

using namespace bullet_trajectory_solver;

[[maybe_unused]] void AirResistanceModel::SetParam(double c, double p, double t, double d, double m) {
    double rho = 1.293 * (p / 1013.25) * (273.15 / (273.15 + t));
    double S = 0.25 * d * d;
    coefficient = 0.5 * c * rho * S / m;
}

Eigen::Vector2d AirResistanceModel::operator()(const Eigen::Vector2d &v) const {
    return -coefficient * v.norm() * v;
}
