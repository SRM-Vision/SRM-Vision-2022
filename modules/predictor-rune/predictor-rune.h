/**
 * Rune predictor definition header.
 * @author LIYunzhe1408, trantuan-20048607, LemonadeJJ
 * @date 2022.5.2
 */

#ifndef PREDICTOR_RUNE_H_
#define PREDICTOR_RUNE_H_

#include "lang-feature-extension/disable-constructors.h"
#include "trajectory-solver/trajectory-solver.h"
#include "digital-twin/facilities/power_rune.h"
#include "data-structure/communication.h"
#include "data-structure/buffer.h"
#include <queue>

namespace predictor::rune {
    constexpr int kFirstFitPalstanceDataNum = 150; ///< Amount of data collected for fitting.
    constexpr int kPreparePalstanceDataNum = 50;   ///< Amount of data required before fitting.
    constexpr int kObservationDataNum = 300;       ///< Amount of data observed in one circle.
    constexpr int kBufferDataNum = 150;            ///< Updated data num in every fit period.
    constexpr int kMaxNumIterations = 300;


    constexpr double kCompensateTime30       = 0.019; ///< Communication, program process and etc delay.
    constexpr double kCompensateTime18       = 0.08; ///< Communication, program process and etc delay.
    constexpr double kCompensateTime15       = 0.099; ///< Communication, program process and etc delay.


    constexpr int kAutoFireTimeGap = 1000;
    constexpr bool kAutoFireFlag = false;

    constexpr int kP_pitch = 3000;
    constexpr int kP_yaw = 3000;

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
         * @brief AngularIntegral of rotational speed is rotational angle.
         * @param integral_time Integrate from t=0 to t=integral_time.
         * @return Rotational angle, in RADIAN, COUNTERCLOCKWISE is positive.
         */
        [[nodiscard]] double AngularIntegral(double integral_time) const;

        [[maybe_unused]] int rotational_direction;  ///< 1 is clockwise, -1 is counterclockwise.
        double a;  ///< Amplitude, or A.
        double w;  ///< Palstance, or omega.
        double p;  ///< Initial phase, or phi.
        double b;  ///< b = 2.090 - a under official rule.
    };

    /// @brief Data package for output.
    struct OutputData {
        /**
         * @brief Final result! ! ! ! ! Update output data.
         * @param debug Debug mode switch.
         * @param [in] rune Input rune data.
         * @param [in] predicted_angle Predicted angle calculated by predictor.
         * @param [in] predicted_point Predicted point calculated by predictor.
         * @param [out] fixed_point Fixed predicted point considering shooting delay.
         */
        void Update(const PowerRune &rune,
                    double predicted_angle,
                    const cv::Point2f &predicted_point,
                    cv::Point2f &fixed_point,
                    float bullet_speed);

        float yaw{};
        float pitch{};
        float delay{};
        int fire = 0;
    };

    /// @brief Data and function package for fitting palstance curve.
    struct FittingData {
        FittingData() : ready(false), first_fit(true) {}

        /**
         * @brief Fit the rotational speed curve only if meet the data num condition.
         * @param debug Debug mode switch.
         * @param [out] rotational_speed Output fit rotational speed.
         */
        void Fit(RotationalSpeed &rotational_speed);

        std::vector<double> palstance;  ///< Speed data for fitting.
        std::vector<double> time;       ///< Time data for fitting.
        bool first_fit;
        int fit_num = 0;
        std::chrono::high_resolution_clock::time_point fit_complete_time;

        bool ready;
    };

    struct State {
        /**
         * @brief Update current angle of fan and convert it into reasonable value, requires new rtp vector.
         * @param [in] rtp_vec Input rtp vec.
         */
        void UpdateAngle(const cv::Point2f &rtp_vec);

        /**
         * @brief Update current and last palstance.
         * @param [in] rune Input rune params.
         * @param [out] fitting_data Output fitting data vector, saving current time and current palstance.
         * @return This function will return false when rtp vector is invalid.
         */
        bool UpdatePalstance(const PowerRune &rune,
                             FittingData &fitting_data);

        static float CalcVectorsAngle(const cv::Point2f &first_vector, const cv::Point2f &second_vector);
        static float CalcPointsDistance(const cv::Point2f &point1, const cv::Point2f &point2);

        /// @brief Check if the rune predicting mode changed.
        bool FanChanged();

        std::chrono::high_resolution_clock::time_point fan_changed_time_chrono;
        double current_angle;
        double current_palstance;
        double last_angle;
        cv::Point2f last_rtp_vec;
    };

    class RunePredictor : NO_COPY, NO_MOVE {
    public:
        ATTR_READER(rune_.ArmorCenterP(), ArmorCenterP)

        ATTR_READER(rune_.CenterR(), EnergyCenterR)

        ATTR_READER(predicted_point_, PredictedPoint)

        ATTR_READER(fixed_point_, FixedPoint)

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
        bool Initialize(const std::string &config_path);

        /**
         * @brief Use rune predictor to get predicted point.
         * @param [in] power_rune Some parameters conveyed by rune detector.
         * @param [in] aim_mode Rune's mode, kBigRune or kSmallRune.
         * @param [in] bullet_speed Bullet speed from electronic controller.
         * @return Processed data which will be sent to electronic controller.
         */
        [[nodiscard]]SendPacket Run(const PowerRune &power_rune,
                                    AimModes aim_mode,
                                    float bullet_speed,
                                    float yaw,
                                    float pitch);

    private:
        void PredictAngle(AimModes aim_mode);

        void PredictPoint();

        void AutoFire();

        PowerRune rune_;  ///< It is initiated by package conveyed by rune detector.
        State state_;
        RotationalSpeed rotational_speed_{};
        FittingData fitting_data_;  ///< Data for fit.
        OutputData output_data_;  ///< Contain yaw, pitch, delay.

        double predicted_angle_;  ///< Predicted angle according to fitted palstance.
        float bullet_speed_;
        bool auto_fire_signal_ = false;
        bool fan_changed = false;
        cv::Point2f predicted_point_;
        cv::Point2f fixed_point_;  ///< Final point which contains all compensation.
    };
}

using RunePredictor = predictor::rune::RunePredictor;

#endif  // PREDICTOR_RUNE_H_