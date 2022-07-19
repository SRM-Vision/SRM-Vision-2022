#include <opencv2/core.hpp>
#include <glog/logging.h>
#include "trajectory-compensator.h"

using namespace compensator;
using namespace trajectory_solver;

bool CompensatorTraj::Initialize(const std::string &type) {
    double pressure, temperature, latitude;
    struct {
        double c, m, d;
    } bullet{};

    cv::FileStorage config;
    config.open("../config/" + type + "/trajectory-param.yaml", cv::FileStorage::READ);
    if (!config.isOpened()) {
        LOG(ERROR) << "Failed to open trajectory config file " << "../config/" + type + "/trajectory-param.yaml.";
        return false;
    }

    if (config["PNP_MAP"].empty() ||
        config["PNP_MAP"]["SMALL_ARMOR"].empty() ||
        config["PNP_MAP"]["BIG_ARMOR"].empty()) {
        LOG(ERROR) << "PnP map is not found in trajectory config file "
                   << "../config/" + type + "/trajectory-param.yaml.";
    }

    if (config["PNP_MAP"]["SMALL_ARMOR"]["A"].empty() || config["PNP_MAP"]["SMALL_ARMOR"]["B"].empty() ||
        config["PNP_MAP"]["SMALL_ARMOR"]["C"].empty() || config["PNP_MAP"]["SMALL_ARMOR"]["D"].empty()) {
        LOG(ERROR) << "Invalid PnP map data for small armors in trajectory config file.";
        return false;
    }

    config["PNP_MAP"]["SMALL_ARMOR"]["A"] >> small_pnp_map[0];
    config["PNP_MAP"]["SMALL_ARMOR"]["B"] >> small_pnp_map[1];
    config["PNP_MAP"]["SMALL_ARMOR"]["C"] >> small_pnp_map[2];
    config["PNP_MAP"]["SMALL_ARMOR"]["D"] >> small_pnp_map[3];

    if (config["PNP_MAP"]["BIG_ARMOR"]["A"].empty() || config["PNP_MAP"]["BIG_ARMOR"]["B"].empty() ||
        config["PNP_MAP"]["BIG_ARMOR"]["C"].empty() || config["PNP_MAP"]["BIG_ARMOR"]["D"].empty()) {
        LOG(ERROR) << "Invalid PnP map data for big armors in trajectory config file.";
        return false;
    }

    config["PNP_MAP"]["BIG_ARMOR"]["A"] >> big_pnp_map[0];
    config["PNP_MAP"]["BIG_ARMOR"]["B"] >> big_pnp_map[1];
    config["PNP_MAP"]["BIG_ARMOR"]["C"] >> big_pnp_map[2];
    config["PNP_MAP"]["BIG_ARMOR"]["D"] >> big_pnp_map[3];

    if (config["GND_TGT_OFFSET"].empty()) {
        LOG(ERROR) << "Ground target offset config is not found in trajectory config file "
                   << "../config/" + type + "/trajectory-param.yaml.";
    }

    if (config["GND_TGT_OFFSET"]["A"].empty() || config["GND_TGT_OFFSET"]["B"].empty() ||
        config["GND_TGT_OFFSET"]["C"].empty() || config["GND_TGT_OFFSET"]["D"].empty()) {
        LOG(ERROR) << "Invalid ground target offset data in trajectory config file.";
        return false;
    }

    config["GND_TGT_OFFSET"]["A"] >> gnd_tgt_offset[0];
    config["GND_TGT_OFFSET"]["B"] >> gnd_tgt_offset[1];
    config["GND_TGT_OFFSET"]["C"] >> gnd_tgt_offset[2];
    config["GND_TGT_OFFSET"]["D"] >> gnd_tgt_offset[3];

    if (config["L"].empty() || config["P"].empty() || config["T"].empty()) {
        LOG(ERROR) << "Invalid environment data in trajectory config file.";
        return false;
    }

    config["T"] >> temperature;
    config["P"] >> pressure;
    config["L"] >> latitude;

    if (config["BULLET_TYPE"].empty()) {
        LOG(ERROR) << "Invalid environment data in trajectory config file.";
        return false;
    }
    std::string bullet_type;
    config["BULLET_TYPE"] >> bullet_type;

    config.release();

    config.open("../config/bullet-data.yaml", cv::FileStorage::READ);
    if (!config.isOpened()) {
        LOG(ERROR) << "Failed to open bullet data file " << "../config/bullet-data.yaml.";
        return false;
    }

    if (config[bullet_type].empty()) {
        LOG(ERROR) << "Bullet data is not found in trajectory config file.";
        return false;
    }

    if (config[bullet_type]["c"].empty() || config[bullet_type]["m"].empty() ||
        config[bullet_type]["d"].empty()) {
        LOG(ERROR) << "Invalid bullet data in trajectory config file.";
        return false;
    }

    config[bullet_type]["c"] >> bullet.c;
    config[bullet_type]["m"] >> bullet.m;
    config[bullet_type]["d"] >> bullet.d;

    config.release();

    auto air_resistance_model = AirResistanceModel();
    air_resistance_model.SetParam(bullet.c, pressure, temperature, bullet.d, bullet.m);
    ballistic_model.SetParam(air_resistance_model, latitude);

    return true;
}

Eigen::Vector3d CompensatorTraj::AnyTargetOffset(double bullet_speed, const Armor &armor,
                                                 double min_theta, double max_theta,
                                                 double max_error, unsigned int max_iter) const {
    double target_pitch = coordinate::convert::Rectangular2Spherical(armor.TranslationVectorWorld()).y();

    // Calculate the real horizontal distance.
    // f(x) = A + Bx + Cx^2 + Dx^3
    // TODO [@screw-44] [Refactor] Move this switch & case to armor.h.
    // FIXME Replace this method to the one calculating HORIZONTAL distance.
    double target_x = 0, pnp_distance = armor.Distance(), temp_pnp_distance = 1;
    switch (armor.Size()) {
        case Armor::SIZE:
            LOG(WARNING) << "Wrong armor size (Armor::SIZE) detected, fallback to kAuto.";
        case Armor::kAuto:
        case Armor::kSmall: {
            for (auto &&pnp_map_coefficient: small_pnp_map) {
                target_x += pnp_map_coefficient * temp_pnp_distance;
                temp_pnp_distance *= pnp_distance;
            }
            break;
        }
        case Armor::kBig: {
            for (auto &&pnp_map_coefficient: big_pnp_map) {
                target_x += pnp_map_coefficient * temp_pnp_distance;
                temp_pnp_distance *= pnp_distance;
            }
            break;
        }
    }

    double delta_h = target_x * tan(target_pitch), start_h, target_h;
    if (delta_h < 0) {
        start_h = -2 * delta_h;
        target_h = -delta_h;
    } else {
        start_h = delta_h;
        target_h = 2 * delta_h;
    }

    auto solver = PitchAngleSolver();
    solver.SetParam(ballistic_model, bullet_speed, start_h, target_h, target_x);
    auto result = solver.Solve(min_theta, max_theta, max_error, max_iter);

    DLOG(INFO) << "tX: " << target_x << ", dH: " << delta_h << ", tP: " << target_pitch
               << ", theta: " << result.x() << ", time: " << result.y() << ".";

    return result;
}

double CompensatorTraj::GroundTargetOffset(double bullet_speed, const Armor &armor) const {
    double offset = 0, temp_x = 1;
    for (double coefficient: gnd_tgt_offset) {
        offset += temp_x * coefficient;
        // FIXME Replace this method to the one calculating HORIZONTAL distance.
        temp_x *= armor.Distance();
    }
    return offset;
}
