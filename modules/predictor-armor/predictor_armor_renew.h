//
// Created by xiguang on 2022/4/28.
//

#ifndef PREDICTOR_ARMOR_RENEW_H_
#define PREDICTOR_ARMOR_RENEW_H_

#include "data-structure/communication.h"
#include "lang-feature-extension/disable_constructors.h"
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "digital-twin/battlefield.h"
#include "antitop_detector.h"
#include "predictor_fsm.h"
#include "predict_armor.h"
#include "antitop_detector_renew.h"
#include "debug-tools/painter.h"


class PredictorArmorRenew: NO_COPY, NO_MOVE {
public:
    /// Map structure of robots data.
    typedef std::unordered_map<Entity::Colors, std::unordered_map<Robot::RobotTypes, std::shared_ptr<Robot>>> RobotMap;

    PredictorArmorRenew() = delete;
    explicit PredictorArmorRenew(Entity::Colors enemy_color=Entity::Colors::kBlue, const std::string& car_name="infantry")
        : enemy_color_(enemy_color),
        target_(-1){
        Initialize(car_name);
    }

    /// Set enemy`s color.
    void SetColor(const Entity::Colors& enemy_color){enemy_color_ = enemy_color;}

    inline void Clear(){
        predict_armors_.clear();
        for(auto& anti_top_detector:anti_top_detectors)
            anti_top_detector.second = AntiTopDetectorRenew();
        for(auto& buffer:grey_buffers_)  /// pending
            buffer.second = 0;
    }

    cv::Point2f ShootPointInPic(const cv::Mat& intrinsic_matrix,cv::MatSize size){
        if(target_ != -1)
            return predict_armors_[target_].ShootPointInPic(intrinsic_matrix);
        return {float(size().width / 2.0), float(size().height / 2.0)};
    }

    void AllShootPoint(const cv::Mat& intrinsic_matrix){
        for(auto& armor:predict_armors_){
            debug::Painter::Instance()->DrawPoint(armor.ShootPointInPic(intrinsic_matrix),
                                            cv::Scalar(0, 0, 255), 1, 10);
        }
    }

    // used to setoff
    double GetTargetDistance(){
        if(target_ != -1)
            return predict_armors_[target_].Distance();
        else
            return 0;
    }

    void Initialize(const std::string& car_name);

    SendPacket Run(const Battlefield &battlefield, const cv::MatSize &size,AimModes mode = kNormal, double bullet_speed = 15);

private:
    Entity::Colors enemy_color_;  ///< Target's color.

    std::vector<PredictArmor> predict_armors_;

    int target_;    ///< Target`s num in predict armors.

    std::unordered_map<Robot::RobotTypes, int> grey_buffers_;   ///< times of gray armors appearing in succession

    std::unordered_map<Robot::RobotTypes, AntiTopDetectorRenew> anti_top_detectors; ///< used to judge whether is top

    /// Used to merge grey and enemy armors. but also pick the right pair armors.
    std::unordered_map<Robot::RobotTypes, std::vector<Armor>> ReviseRobots(const RobotMap& robots,bool exist_enemy,
                                                                            bool exist_grey);

    /**
    * \brief Examine 2 armors by distance.
    * \param armor_1 First armor.
    * \param armor_2 Second armor.
    * \param threshold Distance threshold.
    * \return Whether they're getting close.
    */
    static inline bool IsSameArmorByDistance(const Armor &armor_1,
                                             const Armor &armor_2,
                                             double threshold) {
        auto dis = (armor_2.TranslationVectorWorld() - armor_1.TranslationVectorWorld()).norm();
        DLOG(INFO) << "DISTANCE: " << dis;
        return dis < threshold;
    }   /// TODO make it more rigorous

    static inline std::pair<std::vector<Armor>::iterator, std::vector<Armor> *>SameArmorByDistance(const Armor &target,
                                                                                            std::unordered_map<Robot::RobotTypes, std::vector<Armor>> &robots,
                                                                                            double threshold);

};


#endif //PREDICTOR_ARMOR_RENEW_H_
