/**
 * Energy predictor definition header.
 * \author LIYunzhe1408, trantuan-20048607
 * \date 2022.2.4
 */

#ifndef PREDICTOR_RUNE_H_
#define PREDICTOR_RUNE_H_

#include <ceres/ceres.h>
#include "data-structure/buffer.h"
#include "data-structure/frame.h"
#include "data-structure/communication.h"
#include "digital-twin/facilities/power_rune.h"

// Ceres.
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

    RunePredictor()
            : present_time_(0.0),
              current_fan_angular_velocity_(0.0),
              final_target_point_send_to_control_(0, 0),
              last_RTG_vec_(0, 0),
              is_target_changed_(false),
              is_small_model_(false),
              is_big_model_(false) {};

    ~RunePredictor() = default;

    /// 参数初始化
    [[nodiscard]] static bool Initialize(const std::string &config_path);

    SendPacket Predict(const PowerRune &power_rune);

private:
    PowerRune rune_;

    int debug_ = 1;
    double present_time_; // Decide in CalAngularVelocity

    /// 扇叶切换
    bool FanChanged();

    double last_fan_current_angle_{};

    /// 计算角速度
    double CalAngularVelocity();

    double current_fan_angular_velocity_;
    cv::Point2f last_RTG_vec_;                                                  // 上一帧能量机关中心R指向扇叶中心G的矢量
    std::chrono::high_resolution_clock::time_point last_time_;                  // 上一有效帧的时间戳
    CircularBuffer<std::pair<std::chrono::high_resolution_clock::time_point, float>, 16> circle_fan_palstance_queue;

    /// calculate current_fan_angle_
    void CalCurrentFanAngle();

    double current_fan_angle_{};
    double current_fan_rad_{};

    /// calculate function parameters
    void CalFunctionParameters();

    int is_okay_to_fit_ = 0;
    std::vector<double> speed_data_;
    std::vector<double> time_data_;
    double amplitude_ = 0.780;
    double omega_ = 1.884;
    double phi_ = 0;
    double b_ = 2.090 - amplitude_;

    const int kNumObservation = 300;
    const int kPrepareNum = 1000;

    /// calculate rotated angle
    double CalRadIntegralFromSpeed(const double &integral_time);

    void CalPredictAngle();

    double rotated_angle_{};
    double rotated_rad_{};
    double predicted_angle_{};
    int is_need_to_fit_ = 1;

    /// calculate predict point
    void CalPredictPoint();

    double CalRadius() const;

    cv::Point2f predicted_target_point_;  // 旋转后的装甲板中心, without offset

    /// offset
    cv::Point3f CalYawPitchDelay();

    cv::Point2f final_target_point_send_to_control_;  // final预测后的装甲板中心(包括旋转预测和运动补偿）
    float delta_u_{};  // x
    float delta_v_{};  // y
    cv::Point3f send_yaw_pitch_delay_;    // 识别成功(yaw,pitch)

    /// 数据参数
    double last_fan_angle_{};  // 上一有效帧扇叶角度

    /// 修正参数
    bool is_target_changed_;  // 目标是否切换
    bool is_big_model_;       // 是否是大能量机关
    bool is_small_model_;     // 是否是小能量机关
};

#endif  // PREDICTOR_RUNE_H_
