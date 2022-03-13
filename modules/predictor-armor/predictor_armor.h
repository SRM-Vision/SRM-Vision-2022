/**
 * Armor predictor header.
 * \author trantuan-20048607
 * \date 2022.1.30
 */

#ifndef PREDICTOR_ARMOR_H_
#define PREDICTOR_ARMOR_H_

#include "data-structure/communication.h"
#include "lang-feature-extension/disable_constructor.h"
#include "digital-twin/battlefield.h"
#include "antitop_detector.h"
#include "ekf.h"

/// Predicting function template structure.
struct PredictFunction {
    PredictFunction() : delta_t(0) {}

    /**
     * \brief Uniform linear motion.
     * \details It's supposed that target is doing uniform linear motion.
     * \tparam T Data type.
     * \param [in] x_0 Input original x.
     * \param [out] x Output predicted x.
     */
    template<typename T>
    void operator()(const T x_0[5], T x[5]) {
        x[0] = x_0[0] + delta_t * x_0[1];  // 0.1
        x[1] = x_0[1];  // 100
        x[2] = x_0[2] + delta_t * x_0[3];  // 0.1
        x[3] = x_0[3];  // 100
        x[4] = x_0[4];  // 0.01
    }

    double delta_t;
};

/// Measuring function template structure.
struct MeasureFunction {
    /**
     * \brief Collect positioning data and convert it to spherical coordinate.
     * \tparam T Data type.
     * \param [in] x Input x data.
     * \param [out] y Output target position in spherical coordinate system.
     */
    template<typename T>
    void operator()(const T x[5], T y[3]) {
        T _x[3] = {x[0], x[2], x[4]};
        coordinate::convert::Rectangular2Spherical<T>(_x, y);
    }
};

class ArmorPredictor : NO_COPY, NO_MOVE {
public:
    /// When an armor lasts gray for time below this value, it will be considered as hit.
    const static unsigned int kMaxGreyCount = 20;

    /// Map structure of robots data.
    typedef std::unordered_map<Entity::Colors, std::unordered_map<Robot::RobotTypes, std::shared_ptr<Robot>>> RobotMap;

    /**
     * \brief Use enumeration instead of too many booleans to express flags.
     * \note Flag enumerate types do NOT contain member SIZE.
     * \attention All of flag enumerations' member should be binaries. By default uint8_t should be used.
     */
    enum Flags {
        kDebug = 0b00000001
    };

    /// Predictor's running mode.
    enum Modes {
        kNormal = 0,
        kAntiTop = 1,
        kAutoAntitop = 2,
        SIZE [[maybe_unused]] = 3
    };

    struct Node {
        std::shared_ptr<Armor> armor;  ///< Armor data, shared pointer is used for deleted default constructor.
        /// EKF object. Order of data inside is X: x, v_x, y, v_y, z and Y: yaw, pitch, distance.
        ExtendedKalmanFilter<5, 3> ekf;
        double yaw, pitch;
        int lasting_time;

        bool need_init;      ///< Switch this flag to enforce this node.
        bool need_update;    ///< Switch this flag to control when to update EKF.
        bool long_distance;  ///< There are 2 different methods to deal with targets on different distance.

        /// A tricky use of attr_reader to replace a variable stored in data struct.
        ATTR_READER(Robot::RobotTypes(armor.get()->ID()), Type)

        Node() : yaw(0),
                 pitch(0),
                 lasting_time(0),
                 need_update(false),
                 need_init(true),
                 long_distance(false) {}

        explicit Node(std::shared_ptr<Armor> _armor) :
                armor(std::move(_armor)),
                yaw(0),
                pitch(0),
                lasting_time(0),
                need_update(false),
                need_init(true),
                long_distance(false) {}

        inline Node &operator=(const Node &rhs) {
            if (this == &rhs)
                return SELF;
            armor = rhs.armor;
            ekf = rhs.ekf;
            yaw = rhs.yaw;
            pitch = rhs.pitch;
            need_update = rhs.need_update;
            long_distance = rhs.long_distance;
            return SELF;
        }

        /**
         * \brief Generate a packet according to data inside.
         * \return Send packet to serial port.
         */
        [[nodiscard]] inline SendPacket GenerateSendPacket(bool fire) const {
            auto delay = 1.f;// TODO Add delay and check_sum here.
            SendPacket send_packet = {float(yaw), float(pitch),delay,fire,float(yaw+pitch+delay+fire)};
            return send_packet;
        }
    };

    ATTR_READER_REF(translation_vector_cam_predict_, TranslationVectorCamPredict)

    ATTR_READER(bool(flag_ &kDebug), Debug)

    ArmorPredictor(Entity::Colors color, uint8_t flag) :
            color_(color),
            flag_(flag) {
        Initialize();
    }

    inline void Clear() {
        target_locked_ = false;
        antitop_candidates_.clear();
    }

    SendPacket Run(const Battlefield &battlefield, Modes mode = kNormal);

    inline bool Initialize() {
        ArmorPredictorDebug::Instance().Initialize("../config/sentry/ekf-param.yaml");
        for (auto i = 0; i < Robot::RobotTypes::SIZE; ++i)
            grey_count_[Robot::RobotTypes(i)] = 0;
        return true;
    }

    ~ArmorPredictor() = default;

private:
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
        return (armor_2.TranslationVectorWorld() - armor_1.TranslationVectorWorld()).norm() < threshold;
    }

    /**
     * \brief Get number of armors with the same id.
     * \param color Working color mode.
     * \param [in] armor The armor to match.
     * \param [in] robots Battlefield robot data.
     * \param [in] grey_count Grey time counting.
     * \param exist_enemy There are enemy armors on the battlefield.
     * \param exist_grey There are grey armors on the battlefield.
     * \return Int8 number of armors.
     */
    static inline uint8_t GetSameIDArmorNum(
            Entity::Colors color,
            const Armor &armor,
            const RobotMap &robots,
            const std::unordered_map<Robot::RobotTypes, unsigned int> &grey_count,
            bool exist_enemy,
            bool exist_grey) {
        auto type = Robot::RobotTypes(armor.ID());
        uint8_t armor_num = 0;

        // Find color armors.
        if (exist_enemy && robots.at(color).find(type) != robots.at(color).end())
            armor_num += robots.at(color).at(type)->Armors().size();

        // Find valid grey armors.
        if (exist_grey && grey_count.at(type) < kMaxGreyCount
            && robots.at(Entity::Colors::kGrey).find(type) != robots.at(Entity::Colors::kGrey).end())
            armor_num += robots.at(Entity::Colors::kGrey).at(type)->Armors().size();

        return armor_num;
    }

    /**
     * \brief Copy an armor object for archiving when the there's only one armor of this id.
     * \param color Working color mode.
     * \param [in] node The pre-locked target node.
     * \param [in] robots Battlefield robot data.
     * \param exist_enemy There are enemy armors on the battlefield.
     * \return A shared pointer of armor newly created.
     */
    static inline std::shared_ptr<Armor> CopyArmorDataFromArmorPredictorNode(
            Entity::Colors color,
            const Node &node,
            const RobotMap &robots,
            bool exist_enemy) {
        return std::make_shared<Armor>(
                robots.at(exist_enemy && robots.at(color).find(node.Type()) != robots.at(color).end() ?
                          color : Entity::Colors::kGrey).at(node.Type())->Armors().front());
    }

    /**
     * \brief Match and select an armor when more than 1 armor was detected.
     * \param color Working color mode.
     * \param type Target type.
     * \param [out] target_selected Whether armor is selected.
     * \param target_locked This function only works when target is pre-locked.
     * \param [in] robots Battlefield robot data.
     * \param target_is_the_right Match the armor on the right.
     * \param exist_enemy There are enemy armors on the battlefield.
     * \param exist_grey There are grey armors on the battlefield.
     * \return A pointer to the new armor.
     * \warning You MUST manually manage this pointer to avoid memory leak.
     */
    static inline Armor *MatchArmorsAndPickOne(
            Entity::Colors color,
            Robot::RobotTypes type,
            int &target_selected,
            bool target_locked,
            const RobotMap &robots,
            bool target_is_the_right,
            bool exist_enemy,
            bool exist_grey) {

        if (!target_locked) {
            target_selected = false;
            return nullptr;
        }

        std::vector<Armor> same_id_armors;
        if (exist_enemy)
            for (auto &r: robots.at(color))
                if (r.first == type)
                    same_id_armors.insert(same_id_armors.end(),
                                          r.second->Armors().begin(),
                                          r.second->Armors().end());
        if (exist_grey)
            for (auto &r: robots.at(Entity::Colors::kGrey))
                if (r.first == type)
                    same_id_armors.insert(same_id_armors.end(),
                                          r.second->Armors().begin(),
                                          r.second->Armors().end());
        // This may fail when grey armors of own side are scanned.
        if (same_id_armors.size() > 1) {
            target_selected = 7;
            if (target_is_the_right) {
                auto middle_target_position_x = -DBL_MAX;
                auto middle_target_index = -1;
                for (auto i = 0; i < same_id_armors.size(); ++i)
                    if (same_id_armors.at(i).TranslationVectorWorld()[0] > middle_target_position_x) {
                        middle_target_position_x = same_id_armors.at(i).TranslationVectorWorld()[0];
                        middle_target_index = i;
                    }
                return new Armor(same_id_armors.at(middle_target_index));
            } else {
                auto middle_target_position_x = DBL_MAX;
                auto middle_target_index = -1;
                for (auto i = 0; i < same_id_armors.size(); ++i)
                    if (same_id_armors.at(i).TranslationVectorWorld()[0] < middle_target_position_x) {
                        middle_target_position_x = same_id_armors.at(i).TranslationVectorWorld()[0];
                        middle_target_index = i;
                    }
                return new Armor(same_id_armors.at(middle_target_index));
            }
        } else {
            target_selected = false;
            return nullptr;
        }
    }

    /**
     * \brief Get an ROI area.
     * \param [in] armor Center of ROI area.
     * \param coefficient ROI length and width coefficient.
     * \return An ROI rect.
     */
    static inline cv::Rect2f GetROI(const Armor &armor, float coefficient = 1) {
        auto w = std::max({armor.Corners()[0].x, armor.Corners()[1].x, armor.Corners()[2].x, armor.Corners()[3].x}) -
                 std::min({armor.Corners()[0].x, armor.Corners()[1].x, armor.Corners()[2].x, armor.Corners()[3].x});
        auto h = std::max({armor.Corners()[0].y, armor.Corners()[1].y, armor.Corners()[2].y, armor.Corners()[3].y}) -
                 std::min({armor.Corners()[0].y, armor.Corners()[1].y, armor.Corners()[2].y, armor.Corners()[3].y});
        return {armor.Center().x - w / 2, armor.Center().y - h / 2, w * coefficient, h * coefficient};
    }

    [[maybe_unused]] uint8_t flag_;    ///< Predictor flags.
    std::unordered_map<Robot::RobotTypes, unsigned int>
            grey_count_;  ///< Frame counts of lasting time of grey armors.

    Node target_;  ///< Cached and currently locked target.
    Entity::Colors color_;  ///< Target's color.

    bool fire_ = false;                 ///< Sentry is firing ,only sentry use.
    bool target_locked_ = false;       ///< Target is currently locked.
    bool long_distance_ = false;       ///< Current target is far from self.
    bool target_is_the_right_ = true;  ///< Target is on the right, opposite left.
    bool anticlockwise_ = true;        ///< Target robot is rotating anticlockwise.
    bool antitop_ = false;             ///< Only used in autoantitop.
    Eigen::Vector2d last_armor_speed{0,0};       ///< Armor's last speed.
    uint8_t armor_num_ = 0;            ///< Num of armors with same id as target's.

    Eigen::Vector3d translation_vector_cam_predict_;
    std::vector<Node> antitop_candidates_;
    AntitopDetector antitop_detector_;
};

#endif  // PREDICTOR_ARMOR_H_
