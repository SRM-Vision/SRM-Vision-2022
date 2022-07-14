//
// Created by xiguang on 2022/7/14.
//

#ifndef PREDICTOR_ARMOR_H_
#define PREDICTOR_ARMOR_H_

#include "data-structure/communication.h"
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "digital-twin/battlefield.h"
#include "debug-tools/painter.h"
#include "spin_detector.h"
#include "math-tools/ekf.h"


class ArmorPredictor: NO_COPY, NO_MOVE {
public:
    /// Map structure of robots data.
    typedef std::unordered_map<Entity::Colors, std::unordered_map<Robot::RobotTypes, std::shared_ptr<Robot>>> RobotMap;

    explicit ArmorPredictor(Entity::Colors enemy_color=Entity::kBlue, const std::string& car_name="infantry")
            : enemy_color_(enemy_color){
        Initialize(car_name);
    }

    /// Set enemy`s color.
    void SetColor(const Entity::Colors& enemy_color){enemy_color_ = enemy_color;}

    inline void Clear();

    cv::Point2f ShootPointInPic(const cv::Mat& intrinsic_matrix,cv::MatSize size);

    /// used to offset
    double GetTargetDistance();

    cv::Point2f TargetCenter();

    static void Initialize(const std::string& car_name);

    SendPacket Run(const Battlefield &battlefield, const cv::MatSize &size, double bullet_speed = 15);

    void GetROI(cv::Rect &roi_rect, const cv::Mat &src_image);

    /**
    * \brief Generate a packet according to data inside.
    * \return Send packet to serial port.
    */
    [[nodiscard]] inline SendPacket GenerateSendPacket() const;

private:
    Entity::Colors enemy_color_;  ///< Target's color.

    std::shared_ptr<Armor> last_target_{nullptr};

    int grey_buffer_{0};   ///< times of gray armors appearing in succession

    SpinDetector spin_detector_{SpinDetector::kSpherical, 0.05, 1.2,
                                0.625, 0.125};

    coordinate::TranslationVector predict_world_vector_, predict_cam_vector_, shoot_point_vector_;
    Eigen::Vector2d predict_speed_;   ///< x_v,y_v ; norm() = (x_v^2 + y_v^2)^(1/2)

    ExtendedKalmanFilter<5,3> ekf_;

    int fire_{0}; ///< to judge whether fire.

    uint64_t last_time_{0};

    static std::shared_ptr<Armor> SameArmorByPicDis(const cv::Point2f &target_center, const std::vector<Armor> &armors, double threshold);

    void InitializeEKF(const std::array<float, 3> &yaw_pitch_roll,
                       const coordinate::TranslationVector &translation_vector_world);

    /// Update shoot point and predict camera vector.
    inline void UpdateShootPointAndPredictCam(const std::array<float, 3>& yaw_pitch_roll);

    void Predict(const Armor& armor,double delta_t,double bullet_speed,const std::array<float, 3>& yaw_pitch_roll,double shoot_delay);

    /// Update a new armor
    inline void Update(const Armor& armor);

};


#endif //PREDICTOR_ARMOR_H_
