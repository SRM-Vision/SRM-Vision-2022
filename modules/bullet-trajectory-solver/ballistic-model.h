#ifndef BALLISTIC_MODEL_H_
#define BALLISTIC_MODEL_H_

#include "air-resistance-model.h"

namespace bullet_trajectory_solver {
    class BallisticModel {
    public:
        [[maybe_unused]] void SetParam(const AirResistanceModel &model, double p);

        Eigen::Vector2d operator()(const Eigen::Vector2d &v) const;

    private:
        AirResistanceModel air_resistance_model;
        double g;
    };
}

#endif  // BALLISTIC_MODEL_H_
