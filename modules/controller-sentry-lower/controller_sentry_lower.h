/**
 * Infantry controller header.
 * \author trantuan-20048607, screw-44
 * \date 2022.2.8
 * \warning NEVER include this file except in ./controller_sentry_lower.cpp.
 */

#ifndef CONTROLLER_SENTRY_LOWER_H_
#define CONTROLLER_SENTRY_LOWER_H_

#include "controller-base/controller_factory.h"
#include "controller_sentry_lower_debug.h"

class [[maybe_unused]] SentryLowerController final : public Controller {
public:
    bool Initialize() final;

    void Run() final;

private:
    /// Own registry in controller factory.
    [[maybe_unused]] static ControllerRegistry<SentryLowerController> sentry_lower_controller_registry_;
    debug::IPainter *painter_;
    ControllerSentryLowerDebug controller_sentry_lower_debug_;
};

#endif  // CONTROLLER_SENTRY_LOWER_H_
