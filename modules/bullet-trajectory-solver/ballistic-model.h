#ifndef BALLISTIC_MODEL_H_
#define BALLISTIC_MODEL_H_

#include "air-resistance-model.h"

namespace bullet_trajectory_solver {
    /**
     * @brief Ballistic model function package.
     * @details Use operator () to get the ballistic model acceleration.
     */
    class BallisticModel {
    public:
        /**
         * @brief Set necessary parameters for calculating total acceleration.
         * @param model Air resistance model.
         * @param p Latitude of current position, DEGREE.
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
}

#endif  // BALLISTIC_MODEL_H_
