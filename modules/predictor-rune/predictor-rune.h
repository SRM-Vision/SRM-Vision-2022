/**
 * Rune predictor definition header.
 * @author LIYunzhe1408, trantuan-20048607, LemonadeJJ
 * @date 2022.5.2
 */

#ifndef PREDICTOR_RUNE_H_
#define PREDICTOR_RUNE_H_

#include "lang-feature-extension/disable-constructors.h"
#include "data-structure/communication.h"
#include "digital-twin/facilities/power_rune.h"

namespace predictor::rune {
    constexpr int kCollectPalstanceDataNum = 250;   ///< Amount of data collected for fitting.
    constexpr int kPreparePalstanceDataNum = 100;  ///< Amount of data required before fitting.
    constexpr int kResidualBlockNum = 300;          ///< Amount of data observed in one circle.

    /// @brief Trigonometric residual (cost) function package for Ceres solver.
    struct TrigonometricResidual {
        /**
         * @param _x Independent (input) variable.
         * @param _y Dependent (output) variable.
         * @param _rotational_direction 1 is clockwise, -1 is counterclockwise.
         */
        TrigonometricResidual(double _x, double _y) :
                x(_x), y(_y) {}

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
            residual[0] = y - (a[0] * sin(omega[0] * x + phi[0]) + 2.090 - a[0]);
            return true;
        }

        double x;  ///< Independent variable.
        double y;  ///< Dependent variable.
        [[maybe_unused]] int rotational_direction{};  ///< 1 is clockwise, -1 is counterclockwise.
    };

    /**
     * @brief Parameters to describe and calculate rotational speed.
     * @details Rotational speed = a * sin(w * t + p) + b.
     */
    struct RotationalSpeed {
        /**
         * @brief Integral of rotational speed is rotational angle.
         * @param integral_time Integrate from t=0 to t=integral_time.
         * @return Rotational angle, in RADIAN, COUNTERCLOCKWISE is positive.
         */
        [[nodiscard]] double Integral(double integral_time) const;

        int rotational_direction;  ///< 1 is clockwise, -1 is counterclockwise.
        double a;  ///< Amplitude, or A.
        double w;  ///< Palstance, or omega.
        double p;  ///< Initial phase, or phi.
        double b;  ///< b = 2.090 - a under official rule.
    };

    /// @brief Data package for output.
    struct OutputData {
        /**
         * @brief Update output data.
         * @param debug Debug mode switch.
         * @param [in] rune Input rune data.
         * @param [in] predicted_angle Predicted angle calculated by predictor.
         * @param [in] predicted_point Predicted point calculated by predictor.
         * @param [out] fixed_point Fixed predicted point considering shooting delay.
         */
        void Update(bool debug,
                    const PowerRune &rune,
                    double predicted_angle,
                    const cv::Point2f &predicted_point,
                    cv::Point2f &fixed_point);

        float yaw;
        float pitch;
        float delay;
    };

    /// @brief Data and function package for fitting palstance curve.
    struct FittingData {
        FittingData() : outdated(true), ready(false) {}

        /**
         * @brief Fit the rotational speed curve.
         * @param debug Debug mode switch.
         * @param [out] rotational_speed Output fit rotational speed.
         */
        void Fit(bool debug, RotationalSpeed &rotational_speed);

        std::vector<double> palstance;  ///< Speed data for fitting.
        std::vector<double> time;       ///< Time data for fitting.

        bool outdated;   ///< Whether Fitting Is Error.
        bool ready;
    };

    struct State {
        /**
         * @brief Update current angle of fan, requires new rtp vector.
         * @param [in] rtp_vec Input rtp vec.
         */
        void UpdateAngle(const cv::Point2f &rtg_vec);

        /**
         * @brief Update current and last palstance.
         * @param [in] rune Input rune params.
         * @param [out] fitting_data Output fitting data vector, saving current time and current palstance.
         * @return This function will return false when rtg vector is invalid.
         */
        bool UpdatePalstance(const PowerRune &rune,
                             FittingData &fitting_data);

        /// @brief Check if the rune predicting mode changed.
        void CheckMode();

        double current_time;  ///< Current time, made double for calculations.
        std::chrono::high_resolution_clock::time_point last_time;
        double current_angle;
        double current_palstance;
        double last_angle;
        cv::Point2f last_rtg_vec;
    };

    class RunePredictor : NO_COPY, NO_MOVE {
    public:

        ATTR_READER_REF(predicted_point_, PredictedPoint)

        ATTR_READER_REF(fixed_point_, FixedPoint)

        /**
         * @brief Constructor, initiate some parameters.
         */
        RunePredictor();

        ~RunePredictor() = default;

        /**
         * @brief Initialize rune predictor.
         * @param [in] config_path Rune predictor's parameters' yaml address.
         * @param [in] debug Whether use debug.
         * @return Whether initialize successfully.
         */
        bool Initialize(const std::string &config_path, bool debug);

        /**
         * @brief Use rune predictor to get predicted point.
         * @param [in] power_rune Some parameters conveyed by rune detector.
         * @param [in] aim_mode Rune's mode, kBigRune or kSmallRune.
         * @param [in] bullet_speed Bullet speed from electronic controller.
         * @return Processed data which will be sent to electronic controller.
         */
        [[nodiscard]]SendPacket Run(const PowerRune &power_rune, AimModes aim_mode, float bullet_speed);

    private:
        void PredictAngle(AimModes aim_mode);

        void PredictPoint();

        bool debug_;
        PowerRune rune_;  ///< It is initiated by package conveyed by rune detector.
        State state_;
        RotationalSpeed rotational_speed_;
        FittingData fitting_data_;  ///< Data for fit.
        OutputData output_data_;  ///< Contain yaw, pitch, delay.

        double predicted_angle_;  ///< Predicted angle according to fitted palstance.
        float bullet_speed_;
        cv::Point2f predicted_point_;
        cv::Point2f fixed_point_;  ///< Final point which contains all compensation.
    };
}

using RunePredictor = predictor::rune::RunePredictor;

#endif  // PREDICTOR_RUNE_H_
