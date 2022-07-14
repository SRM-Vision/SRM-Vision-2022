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

    /// reset predictor
    inline void Clear();

    /**
     * @brief if tracking a armor, count a shoot point in picture.
     * @param intrinsic_matrix intrinsic matrix of camera
     * @param size size of frame
     * @return  point of shooting point.
     */
    cv::Point2f ShootPointInPic(const cv::Mat& intrinsic_matrix,cv::MatSize size);

    /// used to offset
    double GetTargetDistance();

    /// target center in picture.
    cv::Point2f TargetCenter();

    /**
     * @brief Initialized the predictor before using. If enter a car name in constructor, initialization is not necessary.
     * @param car_name car name for read configuration file.
     */
    static void Initialize(const std::string& car_name);

    /**
     * @brief run to predict shooting point and get send packet.
     * @param battlefield battlefield information.
     * @param size size of frame that be detected.
     * @param bullet_speed The speed of the bullet provided by the electronic controller.
     * @return the sending packet to serials.
     */
    SendPacket Run(const Battlefield &battlefield, const cv::MatSize &size, double bullet_speed = 15);

    /**
     * @brief get the roi to detect.
     * @param roi_rect the rect that hed detected in the range of.
     * @param src_image the frame that is detected.
     */
    void GetROI(cv::Rect &roi_rect, const cv::Mat &src_image);

    /**
    * \brief Generate a packet according to data inside.
    * \return Send packet to serial port.
    */
    [[nodiscard]] inline SendPacket GenerateSendPacket() const;

private:
    Entity::Colors enemy_color_;  ///< Target's color.

    std::shared_ptr<Armor> last_target_{nullptr};   ///< target in last frame.

    int grey_buffer_{0};   ///< times of gray armors appearing in succession

    /// used to detect spin.
    SpinDetector spin_detector_{SpinDetector::kSpherical, 0.05, 1.2,
                                0.625, 0.125};

    ///target information.
    coordinate::TranslationVector predict_world_vector_, predict_cam_vector_, shoot_point_vector_;
    Eigen::Vector2d predict_speed_;   ///< x_v,y_v ; norm() = (x_v^2 + y_v^2)^(1/2)

    ExtendedKalmanFilter<5,3> ekf_;

    int fire_{0}; ///< to judge whether fire.

    uint64_t last_time_{0};

    /// find the same armor to target by picture distance.
    static std::shared_ptr<Armor> SameArmorByPicDis(const cv::Point2f &target_center, const std::vector<Armor> &armors, double threshold);

    void InitializeEKF(const std::array<float, 3> &yaw_pitch_roll,
                       const coordinate::TranslationVector &translation_vector_world);

    /// Update shoot point and predict camera vector.
    inline void UpdateShootPointAndPredictCam(const std::array<float, 3>& yaw_pitch_roll);

    /// predict ekf
    void Predict(const Armor& armor,double delta_t,double bullet_speed,const std::array<float, 3>& yaw_pitch_roll,double shoot_delay);

    /// copy the new armor information
    inline void Update(const Armor& armor);

};


#endif //PREDICTOR_ARMOR_H_
