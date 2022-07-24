//
// Created by lzy on 2022/5/6.
//

#ifndef COMPENSATOR_H_
#define COMPENSATOR_H_

#include <opencv2/core/persistence.hpp>
#include <glog/logging.h>
#include "data-structure/communication.h"
#include "trajectory-solver/trajectory-solver.h"

#include "compensator-approximate/compensator-approximate.h"
#include "compensator-model/compensator-model.h"

class Compensator{
public:
    bool Initialize(const std::string& robot_name_);
    void GetOffset(const float &pitch, const double &bullet_speed, const double &distance);

private:
    CompensatorApproximate compensator_approximate_;

};
#endif //COMPENSATOR_H_
