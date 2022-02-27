/**
 * Controller base class header.
 * \author trantuan-20048607, lzy20020320
 * \date 2022.1.28
 * \details Include this file only to declare or use pointers of controller.
 */

#ifndef CONTROLLER_BASE_H_
#define CONTROLLER_BASE_H_

#include "lang-feature-extension/disable_constructor.h"
#include "digital-twin/battlefield.h"
#include "serial/serial.h"
#include "image-provider-base/image_provider_base.h"
#include "detector-armor/detector_armor.h"

/**
 * \brief Controller base class.
 * \note You cannot directly construct objects.  \n
 *   Instead, find controller types in subclass documents,
 *   include controller_factory.h and use
 *   ControllerFactory::Instance()::CreateController(controller_type_name).
 */
class Controller : NO_COPY, NO_MOVE {
    /// \brief Catch ctrl+c and exit safely.
    friend void SignalHandler(int signal);

public:
    Controller() :
            image_provider_(nullptr),
            serial_(nullptr),
            armor_detector_(),
            send_packet_(),
            receive_packet_() {
        armor_detector_.Initialize("../assets/models/armor_detector_model.onnx");
    }

    virtual bool Initialize() = 0;

    virtual void Run() = 0;

protected:
    std::unique_ptr<ImageProvider> image_provider_;  ///< Image provider handler.
    Frame frame_;
    std::unique_ptr<Serial> serial_;  ///< Serial communication handler.
    SendPacket send_packet_;
    ReceivePacket receive_packet_;
    ArmorDetector armor_detector_;
    std::vector<bbox_t> boxes_;  ///< Store boxes here to speed up.
    std::vector<Armor> armors_;  ///< Store armors here to speed up.
    Battlefield battlefield_;

    static bool exit_signal_;  ///< Global normal exit signal.

    /**
     * \brief Convert boxes to armors.
     * \attention Since std::vector is not threading safe, do not use it in different threads.
     */
    inline void BboxToArmor() {
        armors_.clear();
        for (const auto &box: boxes_)
            armors_.emplace_back(box,
                                 image_provider_->IntrinsicMatrix(),
                                 image_provider_->DistortionMatrix(),
                                 receive_packet_.quaternion);
    }
};

#endif  // CONTROLLER_BASE_H_
