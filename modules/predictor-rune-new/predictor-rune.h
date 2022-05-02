#pragma once

#include <ceres/ceres.h>
#include "data-structure/buffer.h"
#include "data-structure/frame.h"
#include "data-structure/communication.h"
#include "digital-twin/facilities/power_rune.h"

namespace predictor::rune {
    constexpr int kCollectPalstanceDataNum = 335;
    constexpr int kPreparePalstanceDataNum = 1000;     ///< Amount of data required before fitting.
    constexpr int kResidualBlockNum = 300;  ///< Amount of data observed in one circle.

    /// @brief Trigonometric residual (cost) function package for Ceres solver.
    struct TrigonometricResidual {
        /**
         * @param _x Independent (input) variable.
         * @param _y Dependent (output) variable.
         * @param _rotational_direction 1 is clockwise, -1 is counterclockwise.
         */
        TrigonometricResidual(double _x, double _y, int _rotational_direction) :
                x(_x), y(_y), rotational_direction(_rotational_direction) {}

        /**
         * @brief Calculate trigonometric (cost) residual.
         * @tparam T Reserved for Ceres solver.
         * @param [in] a Amplitude.
         * @param [in] omega Palstance.
         * @param [in] phi Initial phase.
         * @param [out] residual Calculation result.
         * @return Always true, reserved for Ceres solver.
         */
        template<typename T>
        bool operator()(const T *const a, const T *const omega, const T *const phi, T *residual) const {
            // residual[0] = y - (-1.0) * rotational_direction * (a[0] * sin(omega[0] * x + phi[0]) + 2.090 - a[0])
            residual[0] = y + (a[0] * sin(omega[0] * x + phi[0]) + 2.090 - a[0]) * (double) rotational_direction;
            return true;
        }

        double x;  ///< Independent variable.
        double y;  ///< Dependent variable.
        int rotational_direction;  ///< 1 is clockwise, -1 is counterclockwise.
    };

    /**
     * @brief Parameters to describe and calculate rotational speed.
     * @details Rotational speed = a*sin(w*t + p) + b.
     */
    struct RotationalSpeed {
        /**
         * @brief Integral of rotational speed is rotational angle.
         * @param integral_time Integrate from t=0 to t=integral_time.
         * @return Rotational angle, in RADIAN, COUNTERCLOCKWISE is positive.
         */
        double Integral(double integral_time);

        int rotational_direction;  ///< 1 is clockwise, -1 is counterclockwise.
        double a;  ///< Amplitude, or A.
        double w;  ///< Palstance, or omega.
        double p;  ///< Initial phase, or phi.
        double b;  ///< b = 2.090 - a under official rule.
    };

    /// @brief Data package for output.
    struct OutputData {
        float yaw;
        float pitch;
        float delay;
    };

    struct FittingData {
        FittingData() : outdated(false), ready(false) {}

        /**
         * @brief Fit the rotational speed curve.
         * @param debug Debug mode switch.
         * @param [out] rotational_speed Output fit rotational speed.
         */
        void Fit(bool debug, RotationalSpeed &rotational_speed);

        std::vector<double> palstance;  ///< Speed data for fitting.
        std::vector<double> time;   ///< Time data for fitting.
        bool outdated;
        bool ready;
    };

    struct State {
        /**
         * @brief Update current angle of fan, requires new rtp vector.
         * @param [in] rtp_vec Input rtp vec.
         */
        void UpdateAngle(const cv::Point2f &rtp_vec);

        /**
         * @brief Update current and last palstance.
         * @param [in] rune Input rune params.
         * @param [out] fitting_data Output fitting data vector, saving current time and current palstance.
         * @return This function will return false when rtg vector is invalid.
         */
        bool UpdatePalstance(const PowerRune &rune,
                             FittingData &fitting_data);

        double current_time;  ///< Current time, made double for calculations.
        std::chrono::high_resolution_clock::time_point last_time;
        double current_angle;
        double current_palstance;
        double last_angle;
        cv::Point2f last_rtg_vec;
    };

    class RunePredictor : NO_COPY, NO_MOVE {
    public:
        RunePredictor() :
                debug_(false),
                rotational_speed_(),
                output_data_() {}

        ~RunePredictor() = default;

        bool Initialize(const std::string &config_path, bool debug);

        SendPacket Run(const PowerRune &power_rune, AimModes aim_mode);

    private:
        bool debug_;
        PowerRune rune_;

        RotationalSpeed rotational_speed_;
        OutputData output_data_;
        FittingData fitting_data_;

        State state_;
    };
}
