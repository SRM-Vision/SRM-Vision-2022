#ifndef AIR_RESISTANCE_H_
#define AIR_RESISTANCE_H_

#include <Eigen/Eigen>

namespace bullet_trajectory_solver {
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
}

#endif  // AIR_RESISTANCE_H_
