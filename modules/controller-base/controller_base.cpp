
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "controller_base.h"

bool Controller::InitializeImageProvider(std::string type) {
    // Use reset here to allocate memory for an abstract class.
    image_provider_.reset(CREATE_IMAGE_PROVIDER(CmdlineArgParser::Instance().RunWithCamera() ? "camera" : "video"));
    if (!image_provider_->Initialize(
            CmdlineArgParser::Instance().RunWithCamera() ?
            "../config/" + type + "/camera-init.yaml" : "../config/" + type + "/video-init.yaml")) {
        LOG(ERROR) << "Failed to initialize image provider.";
        // Till now the camera may be open, it's necessary to reset image_provider_ manually to release camera.
        image_provider_.reset();
        return false;
    }
    return true;
}

bool Controller::InitializeGimbalSerial() {
    if (CmdlineArgParser::Instance().RunWithGimbal() || CmdlineArgParser::Instance().RunWithSerial()) {
        serial_ = std::make_unique<Serial>();
        if (!serial_->StartCommunication()) {
            LOG(ERROR) << "Failed to start serial communication.";
            serial_->StopCommunication();
            // To use std::make_unique will automatically reset serial_ at the next time.
            // So, there's no need to reset it manually.
            return false;
        }
    }
    return true;
}

void Controller::RunGimbal() {
    if (CmdlineArgParser::Instance().RunWithGimbal()) {
        SerialReceivePacket serial_receive_packet{};
        serial_->GetData(serial_receive_packet, std::chrono::milliseconds(5));
        receive_packet_ = ReceivePacket(serial_receive_packet);
    }
}
