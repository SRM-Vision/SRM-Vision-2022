/**
 * Controller base class header.
 * \author trantuan-20048607, lzy20020320
 * \date 2022.1.28
 * \details Include this file only to declare or use pointers of controller.
 */

#ifndef CONTROLLER_BASE_H_
#define CONTROLLER_BASE_H_

#include <opencv2/highgui.hpp>
#include "lang-feature-extension/disable-constructors.h"
#include "serial/serial.h"
#include "digital-twin/battlefield.h"
#include "image-provider-base/image-provider-base.h"
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

    virtual ~Controller() = default;

protected:
    [[nodiscard]] bool InitializeImageProvider();

    [[nodiscard]] bool InitializeGimbal();

    template<bool flip>
    [[nodiscard]] bool GetImage() {
        if (!image_provider_->GetFrame(frame_)) {
            LOG(WARNING) << "Failed to get image, wait for camera or press ctrl-c to quit.";
#if NDEBUG
            sleep(1);
#else
            cv::waitKey(1000);
#endif
            return false;
        }

        if (flip)  // Flip image when using camera.
            if (CmdlineArgParser::Instance().RunWithCamera()) {
                cv::flip(frame_.image, frame_.image, 0);
                cv::flip(frame_.image, frame_.image, 1);
            }
        return true;
    }

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
    inline void BboxToArmor(Armor::ArmorSize size = Armor::ArmorSize::kAuto) {
        armors_.clear();
        for (const auto &box: boxes_)
            armors_.emplace_back(box,
                                 image_provider_->IntrinsicMatrix(),
                                 image_provider_->DistortionMatrix(),
                                 receive_packet_.yaw_pitch_roll,
                                 size);
    }
};

#endif  // CONTROLLER_BASE_H_
