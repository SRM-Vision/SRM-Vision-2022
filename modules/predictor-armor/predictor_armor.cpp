#include "predictor_armor.h"

const double kDistanceThreshold = 6;            ///< Distance threshold to judge whether a target is too far.
const float kACSpeedXCoefficient = .5f;         ///< Coefficient of inherit anti-top candidate's speed x.
const float kACSpeedYCoefficient = .5f;         ///< Coefficient of inherit anti-top candidate's speed y.
const double kACInitMinLastingTime = 1;         ///< Minimal lasting time to enter anti-top candidates.
const double kAccelerationThreshold = 100;        ///< Maximal acceleration allow to fire.
const float kSwitchArmorAreaProportion = 1.1f;  ///< Minimal area of armor to switch to.

SendPacket ArmorPredictor::Run(const Battlefield &battlefield, Modes mode) {
    auto &robots = battlefield.Robots();

    // Do nothing if nothing is found.
    // ================================================
    // Find grey armors.
    bool exist_grey = true;
    if (robots.find(Entity::Colors::kGrey) == robots.end())
        exist_grey = false;

    fire_ = false;

    // Find enemy armors.
    bool exist_enemy = true;
    if (robots.find(color_) == robots.end())
        exist_enemy = false;

    if (!exist_enemy && !exist_grey) {
        Clear();
        return {0, 0, 0, false,0};
    }

    // Reset and update grey counts of all armors found.
    // ================================================
    // Reset gray counts of color armors.
    if (exist_enemy)
        for (auto &robot: robots.at(color_))
            grey_count_[robot.first] = 0;

    // Update grey counts of grey armors.
    unsigned int grey_count_min = 0x3f3f3f3f;
    if (exist_grey)
        for (auto &robot: robots.at(Entity::Colors::kGrey)) {
            ++grey_count_[robot.first];
            if (grey_count_[robot.first] < grey_count_min)
                grey_count_min = grey_count_[robot.first];
        }

    // All grey armors last too much time and are useless.
    // ================================================
    if (grey_count_min >= kMaxGreyCount && !exist_enemy) {
        Clear();
        return {0, 0, 0, false,0};
    }

    // TODO Calibrate bullet speed and shoot delay.
    const double bullet_speed = 15;
    const double shoot_delay = 0.02;
    double tm_cam_to_imu_data[] = {0, -0.026, -0.075};
    const static coordinate::TranslationMatrix camera_to_imu_translation_matrix(tm_cam_to_imu_data);

    static uint64_t time_stamp = 0;
    double delta_t = double(battlefield.TimeStamp() - time_stamp) * 1e-9;
    time_stamp = battlefield.TimeStamp();

    std::shared_ptr<Armor> target;  ///< Target selected in this round.
    bool same_target = false;       ///< Target to shoot is same as the one on last time.
    bool same_id = false;           ///< Target is not the same but has same id.
    bool switch_armor = false;      ///< Switch target to lock to another.
    bool need_init = false;         ///< Missed conditions to keep locking previous target.
    bool antitop = (mode == kAntiTop || (mode == kAutoAntitop && antitop_));    /// Is antitop
    int target_selected = 0;   ///< Where a target is selected.
    uint8_t armor_num;

    // Find and select the same target as pre-locked one by anti-top.
    // ================================================
    if (target_locked_) {
        armor_num = GetSameIDArmorNum(color_, *target_.armor, robots, grey_count_, exist_enemy, exist_grey);

        if (armor_num_ == 1 && armor_num == 1) {
            // Continue to lock this target.
            // ================================================
            target = CopyArmorDataFromArmorPredictorNode(color_, target_, robots, exist_enemy);
            target_selected = 1;
            same_target = true;
            same_id = false;
            switch_armor = false;
            need_init = false;
        } else if (armor_num_ > 1 && armor_num > 1) {
            // Choose the nearer target.
            // ================================================
            auto temp_matched_armor_ptr = MatchArmorsAndPickOne(color_, target_.Type(), target_selected,
                                                                target_locked_, robots,
                                                                target_is_the_right_,
                                                                exist_enemy, exist_grey);
            if (target_selected) {
                target.reset(temp_matched_armor_ptr);
                same_target = true;
                same_id = false;
                switch_armor = false;
                need_init = false;
            }
        } else if (armor_num_ > 1 && armor_num == 1 && antitop) {
            // The interfering armor disappeared.
            // ================================================
            target_selected = 2;

            // The right / left armor on a clockwise / anti-clockwise rotating robot has disappeared.
            if (anticlockwise_ ^ target_is_the_right_) {
                // The pre-locked target is not the disappeared one. Keep locking it.
                // ================================================
                target = CopyArmorDataFromArmorPredictorNode(color_, target_, robots, exist_enemy);
                same_target = true;
                same_id = false;
                switch_armor = false;
                need_init = false;
            } else {
                // The pre-locked target has just disappeared, execute anti-top procedure.
                // ================================================
                same_id = true;
                switch_armor = true;
                target = CopyArmorDataFromArmorPredictorNode(color_, target_, robots, exist_enemy);

                if (!antitop_candidates_.empty()) {
                    auto min_distance = DBL_MAX;
                    auto min_distance_index = -1;
                    for (auto i = 0; i < antitop_candidates_.size(); ++i) {
                        auto distance = (antitop_candidates_[i].armor->TranslationVectorWorld() -
                                         target->TranslationVectorWorld()).norm();
                        if (distance < min_distance) {
                            min_distance_index = i;
                            min_distance = distance;
                        }
                    }

                    antitop_candidates_.emplace_back(target_);
                    antitop_candidates_.back().armor = target;
                    antitop_candidates_.back().need_update = false;
                    antitop_candidates_.back().need_init = need_init;

                    target_.ekf = antitop_candidates_[min_distance_index].ekf;
                    target_.armor = antitop_candidates_[min_distance_index].armor;

                    antitop_candidates_.erase(antitop_candidates_.begin() + min_distance_index);

                    same_target = true;
                    need_init = false;
                } else {
                    target_selected = false;
                    same_target = false;
                    same_id = false;
                    switch_armor = false;
                    need_init = true;
                }
            }
        } else if (armor_num_ == 1 && armor_num > 1 && antitop) {
            // Another armor appeared.
            // ================================================
            target_selected = 3;
            same_target = true;
            same_id = false;
            need_init = false;

            // Get all armors.
            std::vector<const Armor *> temp_armors;
            if (exist_enemy)
                for (const auto &robot: robots.at(color_))
                    if (robot.first == target_.Type()) {
                        for (auto &armor: robot.second->Armors())
                            temp_armors.emplace_back(&armor);
                        break;
                    }
            if (exist_grey && temp_armors.size() == 1)
                for (auto &robot: robots.at(Entity::Colors::kGrey))
                    if (robot.first == target_.Type()) {
                        for (auto &armor: robot.second->Armors())
                            temp_armors.emplace_back(&armor);
                        break;
                    }

            // Select the armor in the middle.
            // ================================================
            if (anticlockwise_) {
                auto middle_target_position_x = -DBL_MAX;
                auto middle_target_index = -1;
                for (auto i = 0; i < temp_armors.size(); ++i)
                    if (temp_armors.at(i)->TranslationVectorWorld()[0] > middle_target_position_x) {
                        middle_target_position_x = temp_armors.at(i)->TranslationVectorWorld()[0];
                        middle_target_index = i;
                    }
                target = std::make_shared<Armor>(*temp_armors.at(middle_target_index));
                target_is_the_right_ = true;
            } else {
                auto middle_target_position_x = DBL_MAX;
                auto middle_target_index = -1;
                for (auto i = 0; i < temp_armors.size(); ++i)
                    if (temp_armors.at(i)->TranslationVectorWorld()[0] < middle_target_position_x) {
                        middle_target_position_x = temp_armors.at(i)->TranslationVectorWorld()[0];
                        middle_target_index = i;
                    }
                target = std::make_shared<Armor>(*temp_armors.at(middle_target_index));
                target_is_the_right_ = false;
            }
        } else
            target_selected = false;
    }

    // Find and select the same target as pre-locked one by ROI and distance.
    // ================================================
    if (!target_selected && target_locked_) {
        // Find colored armor.
        // ================================================
        if (exist_enemy)
            for (auto &robot: robots.at(color_))
                for (auto &armor: robot.second->Armors())
                    if (armor.Center().inside(GetROI(*target_.armor))
                        || IsSameArmorByDistance(*target_.armor,
                                                 armor,
                                                 kDistanceThreshold)) {
                        target = std::make_shared<Armor>(armor);
                        target_selected = 4;
                        same_target = true;
                        same_id = false;
                        switch_armor = false;
                        need_init = false;
                        break;
                    }

        // Find grey armor.
        // ================================================
        if (!target_selected && exist_grey)
            for (auto &robot: robots.at(Entity::Colors::kGrey))
                if (grey_count_[robot.first] < kMaxGreyCount)
                    for (auto &armor: robot.second->Armors())
                        if (target_locked_ && (armor.Center().inside(GetROI(*target_.armor)) ||
                                               IsSameArmorByDistance(*target_.armor,
                                                                     armor,
                                                                     kDistanceThreshold))) {
                            target = std::make_shared<Armor>(armor);
                            target_selected = 5;
                            same_target = true;
                            same_id = false;
                            switch_armor = false;
                            need_init = false;
                            break;
                        }
    }

    // Find an armor by its area.
    // ================================================
    if (!target_selected) {
        auto max_area = DBL_MIN;
        const Armor *max_area_target;
        if (exist_enemy) {
            for (auto &robot: robots.at(color_))
                for (auto &armor: robot.second->Armors()) {
                    auto area = armor.Area();
                    if (area > max_area) {
                        max_area_target = &armor;
                        max_area = area;
                    }
                }
        }
        if (exist_grey)
            for (auto &robot: robots.at(Entity::Colors::kGrey))
                if (grey_count_[robot.first] < kMaxGreyCount)
                    for (auto &armor: robot.second->Armors()) {
                        auto area = armor.Area();
                        if (area > max_area) {
                            max_area_target = &armor;
                            max_area = area;
                        }
                    }
        target = std::make_shared<Armor>(*max_area_target);
        target_selected = 6;
        same_target = false;
        same_id = false;
        switch_armor = false;
        need_init = true;
    }

    // Till now, target must have been found. Step into next stage.
    // ================================================
    armor_num = GetSameIDArmorNum(color_, *target, robots, grey_count_, exist_enemy, exist_grey);

    // Confirm whether target selected is the right one.
    // ================================================
    if (armor_num > 1) {
        bool confirmed_target_is_the_right_ = false;
        if (exist_enemy)
            for (auto &robot: robots.at(color_))
                for (auto &armor: robot.second->Armors())
                    if (armor.ID() == target->ID() && armor != *target) {
                        if (target->TranslationVectorWorld()[0] > armor.TranslationVectorWorld()[0])
                            target_is_the_right_ = true;
                        else
                            target_is_the_right_ = false;
                        confirmed_target_is_the_right_ = true;
                        break;
                    }

        if (!confirmed_target_is_the_right_ && exist_grey) {
            for (auto &robot: robots.at(Entity::Colors::kGrey))
                for (auto &armor: robot.second->Armors())
                    if (armor.ID() == target->ID() && armor != *target) {
                        if (target->TranslationVectorWorld()[0] >
                            armor.TranslationVectorWorld()[0])
                            target_is_the_right_ = true;
                        else
                            target_is_the_right_ = false;
                        break;
                    }
        }
    }

    for (auto &node: antitop_candidates_) {
        node.need_update = false;
        node.need_init = false;
    }

    // Update target's left and right position when another armor disappeared.
    // ================================================
    if (same_id && !antitop) {
        auto antitop_candidate_ = antitop_candidates_.begin();
        for (; antitop_candidate_ != antitop_candidates_.end(); ++antitop_candidate_)
            if (target->Center().inside(GetROI(*antitop_candidate_->armor))
                || IsSameArmorByDistance(*antitop_candidate_->armor,
                                         *target,
                                         kDistanceThreshold)) {
                target_.ekf = antitop_candidate_->ekf;
                target_.armor = antitop_candidate_->armor;

                // Reverse.
                if (armor_num > 1)
                    target_is_the_right_ = !target_is_the_right_;
                break;
            }
        // Combine the same ID target to the same target.
        if (antitop_candidate_ != antitop_candidates_.end()) {
            antitop_candidates_.erase(antitop_candidate_);
            same_target = true;
            need_init = false;
        } else
            need_init = true;
    }

    // Target has been selected and confirmed whether is the same as pre-locked. Now process prediction.
    // ================================================
    PredictFunction predict;  ///< Predicting function.
    MeasureFunction measure;  ///< Measuring function.

    // Target is the same as the pre-locked one.
    // ================================================
    if (same_target) {
        target_.armor = target;
        coordinate::TranslationVector shoot_point_rectangular;
        Eigen::Vector3d shoot_point_spherical;

        predict.delta_t = delta_t;
        Eigen::Vector2d predict_speed;
        Eigen::Matrix<double, 5, 1> x_real;
        Eigen::Matrix<double, 3, 1> y_real;

        x_real << target->TranslationVectorWorld()[0],
                0,
                target->TranslationVectorWorld()[1],
                0,
                target->TranslationVectorWorld()[2];

        measure(x_real.data(), y_real.data());

        Eigen::Matrix<double, 5, 1> x_predict = target_.ekf.Predict(predict);
        Eigen::Matrix<double, 5, 1> x_estimate = target_.ekf.Update(measure, y_real);

        auto delta_t_predict = target->TranslationVectorWorld().norm() / bullet_speed + shoot_delay;
        predict.delta_t = delta_t_predict;
        predict(x_estimate.data(), x_predict.data());
        Eigen::Vector3d tv_world_current{x_estimate(0, 0), x_estimate(2, 0), x_estimate(4, 0)};
        Eigen::Vector3d tv_world_predict{x_predict(0, 0), x_predict(2, 0), x_predict(4, 0)};
        shoot_point_rectangular = coordinate::transform::WorldToCamera(
                tv_world_predict,
                coordinate::transform::QuaternionToRotationMatrix(battlefield.Quaternion()),
                Eigen::Vector3d::Zero(),
                Eigen::Matrix3d::Identity());

        translation_vector_cam_predict_ = coordinate::transform::WorldToCamera(
                tv_world_predict,
                coordinate::transform::QuaternionToRotationMatrix(battlefield.Quaternion()),
                camera_to_imu_translation_matrix,
                Eigen::Matrix3d::Identity());

        // TODO Rotation vector to camera required.

        shoot_point_spherical = coordinate::convert::Rectangular2Spherical(shoot_point_rectangular);
        target_.yaw = (float) shoot_point_spherical(0, 0);
        target_.pitch = (float) shoot_point_spherical(1, 0);
        target_.long_distance = long_distance_;

        predict.delta_t = 0.001;
        Eigen::Matrix<double, 5, 1> x_predict_delta;
        predict(x_predict.data(), x_predict_delta.data());
        Eigen::Vector3d tv_world_predict_delta{x_predict_delta(0, 0),
                                               x_predict_delta(2, 0),
                                               x_predict_delta(4, 0)};
        Eigen::Vector3d tv_imu_delta = coordinate::transform::WorldToCamera(
                tv_world_predict_delta,
                coordinate::transform::QuaternionToRotationMatrix(battlefield.Quaternion()),
                Eigen::Vector3d::Zero(),
                Eigen::Matrix3d::Identity());
        Eigen::Vector3d tv_imu_current = coordinate::transform::WorldToCamera(
                tv_world_predict,
                coordinate::transform::QuaternionToRotationMatrix(battlefield.Quaternion()),
                Eigen::Vector3d::Zero(),
                Eigen::Matrix3d::Identity());

        Eigen::Vector3d predict_delta = coordinate::convert::Rectangular2Spherical(tv_imu_delta);
        Eigen::Vector3d predict_current = coordinate::convert::Rectangular2Spherical(tv_imu_current);

        predict_speed(0, 0) = (predict_delta(0, 0) - predict_current(0, 0)) / 0.001;
        predict_speed(1, 0) = (predict_delta(1, 0) - predict_current(1, 0)) / 0.001;
    }

    // There's no reference, re-initialize anti-top.
    // ================================================
    if (need_init) {
        target_.armor = target;
        Eigen::Matrix<double, 5, 1> x_real;

        x_real << target->TranslationVectorWorld()[0],
                0,
                target->TranslationVectorWorld()[1],
                0,
                target->TranslationVectorWorld()[2];

        target_.ekf.Initialize(x_real);
        coordinate::TranslationVector shoot_point_rectangular = coordinate::transform::WorldToCamera(
                target->TranslationVectorWorld(),
                coordinate::transform::QuaternionToRotationMatrix(battlefield.Quaternion()),
                Eigen::Vector3d::Zero(),
                Eigen::Matrix3d::Identity());

        translation_vector_cam_predict_ = coordinate::transform::WorldToCamera(
                target->TranslationVectorWorld(),
                coordinate::transform::QuaternionToRotationMatrix(battlefield.Quaternion()),
                camera_to_imu_translation_matrix,
                Eigen::Matrix3d::Identity());

        // TODO Rotation vector to camera required.
        Eigen::Vector3d shoot_point_spherical =
                coordinate::convert::Rectangular2Spherical(shoot_point_rectangular);
        target_.yaw = (float) shoot_point_spherical(0, 0);
        target_.pitch = (float) shoot_point_spherical(1, 0);
        antitop_candidates_.clear();
    }

    std::vector<Armor> all_armors;  ///< All possible target armors.
    if (exist_enemy)
        for (auto &robot: robots.at(color_))
            for (const auto &armor: robot.second->Armors())
                all_armors.emplace_back(armor);
    if (exist_grey)
        for (auto &robot: robots.at(Entity::Colors::kGrey))
            if (grey_count_[robot.first] < kMaxGreyCount)
                for (const auto &armor: robot.second->Armors())
                    all_armors.emplace_back(armor);

    // Update anti-top candidates.
    // ================================================
    for (const auto &armor: all_armors) {
        if (armor.ID() != target->ID() || armor == *target) continue;
        bool exist_same_armor = false;
        for (auto &antitop_candidate: antitop_candidates_) {
            if (antitop_candidate.need_update)
                continue;

            int is_same_armor = false;
            auto same_armor = armor;

            if (armor.Center().inside(GetROI(*antitop_candidate.armor)) ||
                IsSameArmorByDistance(*antitop_candidate.armor, armor,
                                      kDistanceThreshold)) {
                is_same_armor = true;
            } else if (armor_num_ > 1 && armor_num > 1) {
                auto temp_armor_ptr = MatchArmorsAndPickOne(
                        color_,
                        Robot::RobotTypes(same_armor.ID()),
                        is_same_armor,
                        target_locked_, robots,
                        !target_is_the_right_,
                        exist_enemy, exist_grey);
                if (is_same_armor) {
                    same_armor = *temp_armor_ptr;
                    delete temp_armor_ptr;
                }
            }
            if (is_same_armor) {
                exist_same_armor = true;

                antitop_candidate.armor = std::make_shared<Armor>(same_armor);
                antitop_candidate.need_update = true;

                ++antitop_candidate.lasting_time;
                if (antitop_candidate.lasting_time <= kACInitMinLastingTime) {
                    antitop_candidate.need_init = true;
                    Eigen::Vector3d tv_world_measure = antitop_candidate.armor->TranslationVectorWorld();
                    Eigen::Matrix<double, 5, 1> x_real;

                    // Set a reasonable initial data.
                    x_real << tv_world_measure(0, 0),
                            -kACSpeedXCoefficient * target_.ekf.x_estimate_(1, 0),
                            tv_world_measure(1, 0),
                            -kACSpeedYCoefficient * target_.ekf.x_estimate_(3, 0),
                            tv_world_measure(2, 0);

                    antitop_candidate.ekf.Initialize(x_real);
                    antitop_candidate.armor = std::make_shared<Armor>(armor);
                } else {
                    antitop_candidate.need_init = false;
                    Eigen::Vector3d tv_world_measure = antitop_candidate.armor->TranslationVectorWorld();
                    Eigen::Vector2d predict_speed;
                    coordinate::TranslationVector shoot_point_rectangular;
                    predict.delta_t = delta_t;
                    Eigen::Matrix<double, 5, 1> x_real;
                    Eigen::Matrix<double, 3, 1> y_real;

                    x_real << tv_world_measure[0], 0, tv_world_measure[1], 0, tv_world_measure[2];

                    measure(x_real.data(), y_real.data());
                    Eigen::Matrix<double, 5, 1> x_predict = antitop_candidate.ekf.Predict(predict);
                    Eigen::Matrix<double, 5, 1> x_estimate = antitop_candidate.ekf.Update(measure, y_real);

                    double predict_time = tv_world_measure.norm() / (bullet_speed) + shoot_delay;
                    predict.delta_t = predict_time;
                    predict(x_estimate.data(), x_predict.data());

                    Eigen::Vector3d tv_world_current{x_estimate(0, 0),
                                                     x_estimate(2, 0),
                                                     x_estimate(4, 0)},
                            tv_world_predict{x_predict(0, 0),
                                             x_predict(2, 0),
                                             x_predict(4, 0)};

                    shoot_point_rectangular = coordinate::transform::WorldToCamera(
                            tv_world_predict,
                            coordinate::transform::QuaternionToRotationMatrix(battlefield.Quaternion()),
                            Eigen::Vector3d::Zero(),
                            Eigen::Matrix3d::Identity());

                    Eigen::Vector3d shoot_point_spherical =
                            coordinate::convert::Rectangular2Spherical(shoot_point_rectangular);

                    antitop_candidate.yaw = (float) shoot_point_spherical(0, 0);
                    antitop_candidate.pitch = (float) shoot_point_spherical(1, 0);
                    antitop_candidate.long_distance = long_distance_;

                    predict.delta_t = 0.001;

                    Eigen::Matrix<double, 5, 1> x_delta;
                    predict(x_predict.data(), x_delta.data());

                    Eigen::Vector3d tv_world_delta{x_delta(0, 0),
                                                   x_delta(2, 0),
                                                   x_delta(4, 0)};
                    Eigen::Vector3d tv_imu_delta = coordinate::transform::WorldToCamera(
                            tv_world_delta,
                            coordinate::transform::QuaternionToRotationMatrix(battlefield.Quaternion()),
                            Eigen::Vector3d::Zero(),
                            Eigen::Matrix3d::Identity());
                    Eigen::Vector3d tv_imu_current = coordinate::transform::WorldToCamera(
                            tv_world_predict,
                            coordinate::transform::QuaternionToRotationMatrix(battlefield.Quaternion()),
                            Eigen::Vector3d::Zero(),
                            Eigen::Matrix3d::Identity());

                    Eigen::Vector3d predict_delta = coordinate::convert::Rectangular2Spherical(tv_imu_delta);
                    Eigen::Vector3d predict_current = coordinate::convert::Rectangular2Spherical(tv_imu_current);

                    predict_speed(0, 0) = (predict_delta(0, 0) - predict_current(0, 0)) / 0.001;
                    predict_speed(1, 0) = (predict_delta(1, 0) - predict_current(1, 0)) / 0.001;
                    if(abs((last_armor_speed-predict_speed).norm())/delta_t < kAccelerationThreshold)
                        fire_ = true;
                    last_armor_speed = predict_speed;
                    LOG(WARNING) <<"Whether fire:  "<<fire_<<std::endl;
                }
                break;
            }
        }

        // Create a new node when there's no same armor.
        // ================================================
        if (!exist_same_armor) {
            Node new_node(std::make_shared<Armor>(armor));
            Eigen::Vector3d tv_world_measure = armor.TranslationVectorWorld();

            Eigen::Matrix<double, 5, 1> x_real;
            x_real << tv_world_measure(0, 0),
                    0.5 * target_.ekf.x_estimate_(1, 0),
                    tv_world_measure(1, 0),
                    0.25 * target_.ekf.x_estimate_(3, 0),
                    tv_world_measure(2, 0);

            new_node.ekf.Initialize(x_real);
            new_node.armor = std::make_shared<Armor>(armor);
            new_node.need_update = true;
            new_node.need_init = true;

            antitop_candidates_.emplace_back(new_node);
        }
    }

    // Clean old anti-top candidates.
    // ================================================
    for (auto antitop_candidate = antitop_candidates_.begin();
         antitop_candidate != antitop_candidates_.end();) {
        if (!antitop_candidate->need_update)
            antitop_candidate = antitop_candidates_.erase(antitop_candidate);
        else
            ++antitop_candidate;
    }

    // Judge whether to switch to another armor.
    // ================================================
    for (auto i = 0; i < antitop_candidates_.size(); ++i) {
        auto &antitop_candidate = antitop_candidates_[i];
        if (antitop_candidate.need_init)
            continue;
        if (antitop_candidate.armor->Area() > kSwitchArmorAreaProportion * target->Area()) {
            switch_armor = true;

            antitop_candidates_.emplace_back(target_);
            antitop_candidates_.back().need_update = true;
            antitop_candidates_.back().need_init = need_init;

            target_.armor = antitop_candidate.armor;
            target_.ekf = antitop_candidate.ekf;
            target_.pitch = antitop_candidate.pitch;
            target_.yaw = antitop_candidate.yaw;
            long_distance_ = antitop_candidate.long_distance;

            antitop_candidates_.erase(antitop_candidates_.begin() + i);

            if (armor_num > 1)
                target_is_the_right_ = !target_is_the_right_;

            anticlockwise_ = target_.yaw <= 0;
            break;
        }
    }

    antitop_ = antitop_detector_.Is_Top(target_.armor->ID() == target->ID(), anticlockwise_, switch_armor, battlefield.TimeStamp());
    LOG(INFO)<<"top_period"<< antitop_detector_.GetTopPeriod();
    target_.armor = target;
    target_locked_ = true;
    armor_num_ = armor_num;
    return target_.GenerateSendPacket(fire_);
}
