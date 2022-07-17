//
// Created by lzy on 2022/5/6.
//

#ifndef CONTROLLER_SENTRY_HIGHER_H_
#define CONTROLLER_SENTRY_HIGHER_H_

#include "controller-base/controller_factory.h"
#include "controller_sentry_higher_debug.h"

class [[maybe_unused]] SentryHigherController final : public Controller {
public:
    bool Initialize() final;

    void Run() final;

private:
    /**
     * \brief Draw the output point from predictor.
     * \param [in,out] image Source image.
     * \param [in] camera_matrix Camera internal matrix.
     * \param [in] camera_point Point in camera coordinate.
     */

    /// Own registry in controller factory.
    [[maybe_unused]] static ControllerRegistry<SentryHigherController> sentry_higher_controller_registry_;

    ControllerSentryHigherDebug controller_sentry_higher_debug_;
};
#endif //CONTROLLER_SENTRY_HIGHER_H_
