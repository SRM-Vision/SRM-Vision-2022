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

    if (config["A"].empty() || config["B"].empty() || config["C"].empty() || config["D"].empty()) {
        LOG(ERROR) << "Invalid PnP map data in trajectory config file.";
        return false;
    }

    config["A"] >> pnp_map_coefficients[0];
    config["B"] >> pnp_map_coefficients[1];
    config["C"] >> pnp_map_coefficients[2];
    config["D"] >> pnp_map_coefficients[3];

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

Eigen::Vector3d CompensatorTraj::Solve(double bullet_speed, const Armor &armor,
                                       double min_theta, double max_theta,
                                       double max_error, unsigned int max_iter) const {
    double pnp_distance = armor.Distance(),
            target_pitch = coordinate::convert::Rectangular2Spherical(armor.TranslationVectorWorld()).y();

    // Calculate the real horizontal distance.
    // f(x) = A + Bx + Cx^2 + Dx^3
    double target_x = 0, temp_pnp_distance = 1;
    for (auto &&pnp_map_coefficient: pnp_map_coefficients) {
        target_x += pnp_map_coefficient * temp_pnp_distance;
        temp_pnp_distance *= pnp_distance;
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
    auto result = solver.Solve(min_theta, max_theta, 0.01, 16);

    DLOG(INFO) << "tX: " << target_x << ", dH: " << delta_h << ", tP: " << target_pitch
               << ", theta: " << result.x() << ", time: " << result.y() << ".";

    return result;
}
