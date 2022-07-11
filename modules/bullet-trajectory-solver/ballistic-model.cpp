#include "math-tools/algorithms.h"
#include "ballistic-model.h"

using namespace bullet_trajectory_solver;

[[maybe_unused]] void BallisticModel::SetParam(const AirResistanceModel &model, double p) {
    air_resistance_model = model;
    constexpr float deg2rad = CV_PI / 180;
    float p_deg = static_cast<float>(p) * deg2rad;
    float s[4] = {p_deg, 2 * p_deg};
    algorithm::SinFloatX4(s);
    g = 9.78 * (1 + 0.0052884 * s[0] * s[0] - 0.0000059 * s[1] * s[1]);
}

Eigen::Vector2d BallisticModel::operator()(const Eigen::Vector2d &v) const {
    auto acceleration = air_resistance_model(v);
    acceleration[1] -= g;
    return acceleration;
}
