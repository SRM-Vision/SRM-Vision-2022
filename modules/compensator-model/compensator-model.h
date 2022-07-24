#ifndef COMPENSATOR_MODEL_H_
#define COMPENSATOR_MODEL_H_

#include "data-structure/communication.h"
#include "digital-twin/components/armor.h"
#include "trajectory-solver/trajectory-solver.h"
#include "compensator/compensator.h"

class CompensatorModel {
public:
    bool Initialize(const std::string &type);

    Eigen::Vector3d GetOffset(float pitch, double bullet_speed, const Armor &armor);

private:
    trajectory_solver::BallisticModel ballistic_model;
    double small_pnp_map[4], big_pnp_map[4];
};

#endif  // COMPENSATOR_MODEL_H_
