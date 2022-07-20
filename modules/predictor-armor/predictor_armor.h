//
// Created by xiguang on 2022/7/14.
//

#ifndef PREDICTOR_ARMOR_H_
#define PREDICTOR_ARMOR_H_

#include "data-structure/communication.h"
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "digital-twin/battlefield.h"
#include "debug-tools/painter.h"
#include "predictor-spin/spin_predictor.h"
#include "math-tools/ekf.h"
#include "trajectory-compensator/trajectory-compensator.h"

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
     * \brief if tracking a armor, count a shoot point in picture.
     * \param intrinsic_matrix [IN] intrinsic matrix of camera
     * \param size size of frame
     * \return  point of shooting point.
     */
    [[nodiscard]] cv::Point2f ShootPointInPic(const cv::Mat& intrinsic_matrix, cv::MatSize size);

    /// target center in picture.
    [[nodiscard]] cv::Point2f TargetCenter();

    /**
     * \brief Initialized the predictor before using. If enter a car name in constructor, initialization is not necessary.
     * \param car_name [IN] car name for read configuration file.
     */
    static void Initialize(const std::string& car_name);

    /**
     * \brief run to predict shooting point and get send packet.
     * \param battlefield [IN] battlefield information.
     * \param size [IN] size of frame that be detected.
     * \param bullet_speed The speed of the bullet provided by the electronic controller.
     * \return the sending packet to serials.
     */
    SendPacket Run(const Battlefield &battlefield, const cv::MatSize &size, Entity::Colors color);

    double GetTargetDistance();

    /**
    * \brief Generate a packet according to data inside.
    * \return Send packet to serial port.
    */
    [[nodiscard]] inline SendPacket
    GenerateSendPacket();

private:
    Entity::Colors enemy_color_;  ///< Target's color.

    std::shared_ptr<Armor> last_target_{nullptr};   ///< target in last frame.

    int grey_buffer_{0};   ///< times of gray armors appearing in succession

    /// used to detect spin.
    PredictorSpin spin_predictor_{PredictorSpin::kSpherical, 0.05, 1,
                                  0.625, 0.125};

    ///target information.
    coordinate::TranslationVector predict_world_vector_, predict_cam_vector_, shoot_point_vector_;
    Eigen::Vector2d predict_speed_;   ///< x_v,y_v ; norm() = (x_v^2 + y_v^2)^(1/2)
    Eigen::Vector2d predict_acc_;

    ExtendedKalmanFilter<7,3> ekf_;

    int fire_{0}; ///< to judge whether fire.

    uint64_t last_time_{0};

    /**
     * \brief a method to find the same armor to target by picture distance.
     * \param target_center [IN] center of locked last target.
     * \param armors [IN/OUT] where to find the new armor.
     * \param threshold the allowed longest distance between last armor and new armor.
     * \return the iterator of the new target in armors.
     */
    static auto SameArmorByPictureDistance(const cv::Point2f &target_center,
                                                             std::vector<Armor> &armors,
                                                             double threshold);

    /**
     * \brief find the best matched armor which is closest to the last one and is not too oblique.
     * \param target [OUT] a shared ptr that wants to catch target.
     * \param target_center [IN] center of last armor
     * \param armors [IN/OUT] where to find the new armor.
     * \param distance_threshold the allowed longest distance between last armor and new armor.
     * \param oblique_threshold when length divided by width less than this, consider it`s too oblique.
     * \return whether the target is the same as the last one.
     */
    static bool FindMatchArmor(std::shared_ptr<Armor> &target, const cv::Point2f &target_center,
                               std::vector<Armor> &armors, double distance_threshold,
                               double oblique_threshold);

    void InitializeEKF(const std::array<float, 3> &yaw_pitch_roll,
                       const coordinate::TranslationVector &translation_vector_world);

    /// UpdateLastArmor shoot point and predict camera vector.
    inline void UpdateShootPointAndPredictCam(const std::array<float, 3>& yaw_pitch_roll);

    /// predict ekf
    void Predict(const Armor& armor, double delta_t, double bullet_speed, const std::array<float, 3>& yaw_pitch_roll, double shoot_delay);

    /// copy the new armor information
    inline void UpdateLastArmor(const Armor& armor);

};


#endif //PREDICTOR_ARMOR_H_
