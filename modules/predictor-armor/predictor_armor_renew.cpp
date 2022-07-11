//
// Created by xiguang on 2022/4/28.
//

#include "predictor_armor_renew.h"

/// When an armor lasts gray for time below this value, it will be considered as hit.
const unsigned int kMaxGreyCount = 20;

/// Used to judge whether two armors really belong to a car
const double kBestDistanceRatio = 4.0;

///< Distance threshold to judge whether a target is too far.
const double kDistanceThreshold = 0.5;

///< Picture Distance threshold to judge whether a target is too far.
const double kPicDistanceThreshold = 30;

///< Switch target in anti-top When new armor bigger than 0.7 * old armor.
const double kSwitchByAreaThreshold = 0.7;

const double kSpeedDecayRatioX = 1;

const double kSpeedDecayRatioY = 1;

void PredictorArmorRenew::Initialize(const std::string &car_name) {
    for (auto i = 0; i < Robot::RobotTypes::SIZE; ++i) {
        grey_buffers_[Robot::RobotTypes(i)] = 0;
//        anti_top_detectors[Robot::RobotTypes(i)] = AntiTopDetectorRenew(0);
    }
    ArmorPredictorDebug::Instance().Initialize("../config/" + car_name + "/predict-param.yaml",
                                               CmdlineArgParser::Instance().DebugUseTrackbar());
}

SendPacket PredictorArmorRenew::Run(const Battlefield &battlefield, const cv::MatSize &size, AimModes mode, double bullet_speed) {
    auto& robots = battlefield.Robots();

    // judge whether existing gray or enemy armor.
    bool exist_grey(true), exist_enemy(true), target_locked(false);
    cv::Point2f pic_center{float(size().width / 2.0),float(size().height / 2.0)};

    static uint64_t timestamp = 0;
    double delta_t = double(battlefield.TimeStamp() - timestamp) * 1e-9;
    timestamp = battlefield.TimeStamp();
    Robot::RobotTypes target_id{Robot::SIZE};
    if(target_ != -1){
        target_id = static_cast<Robot::RobotTypes>(predict_armors_[target_].ID());
    }

    // ================================================
    // Find grey armors.
    if (robots.find(Entity::Colors::kGrey) == robots.end())
        exist_grey = false;
    if (robots.find(enemy_color_) == robots.end())
        exist_enemy = false;

    // Do nothing if nothing is found.
    if(!exist_enemy && !exist_grey){
        Clear();
        return {0, 0, 0, 0, 0};
    }

    /// Update grey armor buffers
    if(exist_grey)
        for(auto& robot:robots.at(Entity::Colors::kGrey))
            ++grey_buffers_[robot.first];
    if(exist_enemy)
        for(auto& robot:robots.at(enemy_color_))
            grey_buffers_[robot.first] = 0; // reset grey buffers.

    auto preprocessed_robots = ReviseRobots(robots,exist_enemy,exist_grey);   // robots after preprocessing

    /// Update EKF
    if(!predict_armors_.empty()){
        int num(0); // which armor currently
        for(auto begin = predict_armors_.begin();begin != predict_armors_.end();++num){
            bool found_armor(false);
//            auto target(SameArmorByDistance(*begin,preprocessed_robots,kDistanceThreshold));
            auto matched_armor(SameArmorByPicDis(*begin, preprocessed_robots, kPicDistanceThreshold));
            if(matched_armor.second != nullptr){
                begin->Predict(*matched_armor.first, delta_t, bullet_speed, battlefield.YawPitchRoll(), ArmorPredictorDebug::Instance().ShootDelay());
                matched_armor.second->erase(matched_armor.first);
                found_armor = true;
            }
            // if not found, delete it
            if(!found_armor) {
                if(target_ > num)   // When delete armor, the position of target will change.
                    --target_;
                else if(target_ == num) // When target will be deleted, target lost
                    target_ = -1;
                begin = predict_armors_.erase(begin);
                --num;
            }else
                ++begin;
        }
    }

    /// create and initialize predict armors.
    for(auto& robot:preprocessed_robots){
        for(auto& armor:robot.second){
            predict_armors_.emplace_back(armor);
            predict_armors_.back().Initialize(battlefield.YawPitchRoll());
        }
    }

    /// When target does`t lost
    if(target_ != -1)
        target_locked = true;

    /// Speed Decay
    if(spin_detector_.IsSpin()){
        for(auto &predict_armor:predict_armors_){
            if(predict_armor.ID() == int(target_id))
                predict_armor.SpeedDecay(kSpeedDecayRatioX,kSpeedDecayRatioY);
        }
    }

    /// AntiTop
    if(target_locked){
        if(spin_detector_.IsSpin()){
            for(int i{0};i < predict_armors_.size();++i){
                if(i != target_ && predict_armors_[i].ID() == target_id && spin_detector_.Clockwise() != -1 &&
                        predict_armors_[i].Area() > predict_armors_[target_].Area() * kSwitchByAreaThreshold &&
                        (spin_detector_.Clockwise() ^ (predict_armors_[target_].Center().x > predict_armors_[i].Center().x))){
                    // Update speed in ekf, to speed up fitting.
                    predict_armors_[i].UpdateSpeed(predict_armors_[target_].Speed()(0,0),
                                                   -predict_armors_[target_].Speed()(1,0));
                    target_ = i;
                    break;
                }
            }
        }
        spin_detector_.Update(predict_armors_[target_], timestamp);
        if(spin_detector_.IsSpin())
            predict_armors_[target_].AntiSpin(spin_detector_.JumpPeriod(),spin_detector_.LastJumpPosition(),
                                          timestamp, spin_detector_.LastJumpTime(), battlefield.YawPitchRoll());
    }

    /// Find the armor which is nearest to picture center.
    if(!target_locked){
        auto min_distance(DBL_MAX);
        for(int i = 0;i < predict_armors_.size();++i){
            auto center_distance = norm((predict_armors_[i].Center() - pic_center));
            if(center_distance < min_distance){
                target_locked = true;
                target_ = i;
                min_distance = center_distance;
            }
        }
    }

    if(!target_locked)
        return {0, 0, 0, 0, 0};

    return predict_armors_[target_].GenerateSendPacket();
}

std::unordered_map<Robot::RobotTypes, std::vector<Armor>>
PredictorArmorRenew::ReviseRobots(const PredictorArmorRenew::RobotMap &robots, bool exist_enemy, bool exist_grey) {
    std::unordered_map<Robot::RobotTypes, std::vector<Armor>> preprocessing_robots;
    if(exist_enemy)
        for(auto& robot:robots.at(enemy_color_))
            for(auto& armor:robot.second->Armors())
                preprocessing_robots[robot.first].emplace_back(armor);
    if(exist_grey)
        for(auto& robot:robots.at(Entity::Colors::kGrey))
            if(grey_buffers_[robot.first] < kMaxGreyCount)  // not exceeding the limit
                for(auto& armor:robot.second->Armors())
                    preprocessing_robots[robot.first].emplace_back(armor);

    for(auto& armors:preprocessing_robots) {
        // DLOG(INFO) << "ARMOR NUM: " << armors.second.size();
        if (armors.second.size() > 2) {
            /// pick out the two armors in one car by distance between two armors and light length.
            auto min_error(DBL_MAX);
            std::vector<Armor>::iterator real_armor_1, real_armor_2;
            for (auto begin = armors.second.begin(); begin != armors.second.end(); ++begin) {
                for (auto begin2 = begin + 1; begin2 != armors.second.end(); ++begin2) {
                    // light length
                    auto light = cv::min(norm(begin->Corners()[0] - begin->Corners()[1]),
                                         norm(begin->Corners()[1] - begin->Corners()[2]));
                    auto distance = norm(begin->Center() - begin2->Center());   // distance between two armors
                    auto ratio = distance / light;
                    if (cv::abs(ratio - kBestDistanceRatio) < min_error) {
                        min_error = cv::abs(ratio - kBestDistanceRatio);
                        real_armor_1 = begin;
                        real_armor_2 = begin2;
                    }
                }
            }
            armors.second = {real_armor_1, real_armor_2};    // remain the real armors.
        }
    }
    return preprocessing_robots;
}

std::pair<std::vector<Armor>::iterator, std::vector<Armor> *> PredictorArmorRenew::SameArmorByDistance(const Armor &target,
                                                                                                       std::unordered_map<Robot::RobotTypes, std::vector<Armor>> &robots,
                                                                                                       double threshold) {
    std::vector<Armor> *armors_point(nullptr);
    std::vector<Armor>::iterator target_iter;
    auto min_distance(DBL_MAX);
    for(auto &armors:robots){
        for(auto begin(armors.second.begin());begin < armors.second.end();++begin){
            auto distance = (target.TranslationVectorWorld() - begin->TranslationVectorWorld()).norm();
            if(distance < threshold && distance < min_distance){
                armors_point = &armors.second;
                target_iter = begin;
                min_distance = distance;
            }
        }
    }
    return {target_iter,armors_point};
}

std::pair<std::vector<Armor>::iterator, std::vector<Armor> *> PredictorArmorRenew::SameArmorByPicDis(const Armor &target,
                                                                                                       std::unordered_map<Robot::RobotTypes, std::vector<Armor>> &robots,
                                                                                                       double threshold) {
    std::vector<Armor> *armors_point(nullptr);
    std::vector<Armor>::iterator target_iter;
    auto min_distance(DBL_MAX);
    for(auto &armors:robots){
        for(auto begin(armors.second.begin());begin < armors.second.end();++begin){
            auto distance = norm(target.Center() - begin->Center());
            if(distance < threshold && distance < min_distance){
                armors_point = &armors.second;
                target_iter = begin;
                min_distance = distance;
            }
        }
    }
    return {target_iter,armors_point};
}

