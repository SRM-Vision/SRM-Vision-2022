/**
 * Energy predictor definition header.
 * \author LIYunzhe1408, trantuan-20048607, LemonadeJJ
 * \date 2022.3.31
 */

#ifndef PREDICTOR_RUNE_H_
#define PREDICTOR_RUNE_H_

#include <ceres/ceres.h>
#include "data-structure/buffer.h"
#include "data-structure/frame.h"
#include "data-structure/communication.h"
#include "digital-twin/facilities/power_rune.h"

struct TrigonometricResidual {
    TrigonometricResidual(double _x, double _y, int rotate_direction) :
            x(_x), y(_y),
            rotate_direction(rotate_direction) {}

    template<typename T>
    bool operator()(const T *const a, const T *const omega, const T *const phi, T *residual) const {
        residual[0] = y - (-1.0) * rotate_direction * (a[0] * sin(omega[0] * x + phi[0]) + 2.090 - a[0]);
        return true;
    }

    double x;
    double y;
    int rotate_direction;
};

class RunePredictor : NO_COPY, NO_MOVE {
public:
    ATTR_READER_REF(send_yaw_pitch_delay_, SendYawPitchDelay)

    ATTR_READER_REF(final_target_point_send_to_control_, FinalTargetPoint)

    ATTR_READER_REF(predicted_target_point_, PredictedTargetPoint)

    [[maybe_unused]] explicit RunePredictor(bool debug = false);

    ~RunePredictor() = default;

    /**
     * \brief Initialize predictor.
     * \param [in] config_path Config file path.
     * \param [in] debug Whether open debug
     * \return Whether predictor is ready.
     */
    bool Initialize(const std::string &config_path, bool debug = false);

    SendPacket Run(const PowerRune &power_rune, AimModes mode = kBigRune);

private:
    /// Calculate function parameters
    void CalFunctionParameters();

    /// Whether the fan is changed
    bool FanChanged();

    /// Calculate big rune's palstance
    double CalAngularVelocity();

    /// Calculate current_fan_angle_
    void CalCurrentFanAngle();

    /// Calculate rotated angle
    double CalRadIntegralFromSpeed(const double &integral_time);

    /// Calculate predict point
    void CalPredictPoint();

    /// Calculate predict angle
    void CalPredictAngle(AimModes mode);

    /// Calculate rune's radius
    [[nodiscard]] double CalRadius() const;

    /// Calculate yaw and pitch offset
    cv::Point3f CalYawPitchDelay();

    const int kPrepareNum = 1000;                    ///< Amount of data required before fitting
    const int kNumObservation = 300;                 ///< Amount of data observed in one circle

    CircularBuffer<std::pair<std::chrono::high_resolution_clock::time_point, float>, 16> circle_fan_palstance_queue;

    /// State parameter
    double current_time_;
    double current_fan_angle_;
    double current_fan_radian_;
    double current_fan_palstance_;
    double last_fan_current_angle_;
    cv::Point2f last_RTG_vec_;
    std::chrono::high_resolution_clock::time_point last_time_;         ///< Last valid frame timestamp

    /// Amend parameter
    bool debug_;
    float delta_u_;
    float delta_v_;
    bool is_okay_to_fit_;                              ///< Whether is ok to fit
    bool is_need_to_fit_;                              ///< whether need fitting

    /// Data parameter
    PowerRune rune_;                                   ///< Data from detector
    double rotated_angle_;
    double rotated_radian_;
    double predicted_angle_;
    std::vector<double> speed_data_;                   ///< Speed data for fitting
    std::vector<double> time_data_;                    ///< Time data for fitting
    cv::Point3f send_yaw_pitch_delay_;
    cv::Point2f predicted_target_point_;               ///< Rotated armor center, without offset
    cv::Point2f final_target_point_send_to_control_;   ///< Predicted armor center (get offset)

    /// Formula parameter
    double amplitude_;
    double palstance_ ;
    double phase_;
    double b_;
};

#endif  // PREDICTOR_RUNE_H_
