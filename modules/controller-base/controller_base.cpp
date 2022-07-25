#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "image-provider-base/image-provider-factory.h"
#include "compensator/compensator.h"
#include "controller_base.h"

void Controller::ReceiveSerialData() {
    receive_packet_ = frame_.receive_packet;
}

void Controller::SendSerialData() {
    if (CmdlineArgParser::Instance().RunWithSerial())
        serial_->SendData(send_packet_, std::chrono::milliseconds(5));
}

void Controller::CountPerformanceData() {
    static auto time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time);
    LOG(INFO) << "Cost: " << double(duration.count()) / 1e6 << ", FPS: " << 1e9 / double(duration.count()) << ".";
    time = std::chrono::steady_clock::now(); // don`t remove
}

bool Controller::Initialize(const std::string &type) {
    // Use reset here to allocate memory for an abstract class.
    image_provider_.reset(CREATE_IMAGE_PROVIDER(CmdlineArgParser::Instance().RunWithCamera() ? "camera" : "video"));
    if (!image_provider_->Initialize(
            CmdlineArgParser::Instance().RunWithCamera() ?
            "../config/" + type + "/camera-init.yaml" : "../config/" + type + "/video-init.yaml",
            CmdlineArgParser::Instance().Record())) {
        LOG(ERROR) << "Failed to initialize image provider.";
        // Till now the camera may be open, it's necessary to reset image_provider_ manually to release camera.
        image_provider_.reset();
        return false;
    }

    if (CmdlineArgParser::Instance().RunWithSerial()) {
        serial_ = std::make_unique<Serial>();
        if (!serial_->StartCommunication()) {
            LOG(ERROR) << "Failed to start serial communication.";
            serial_->StopCommunication();
            // To use std::make_unique will automatically reset serial_ at the next time.
            // So, there's no need to reset it manually.
            return false;
        }
        image_provider_->SetSerialHandle(serial_.get());
    }

    if (coordinate::InitializeMatrix("../config/" + type + "/matrix-init.yaml"))
        LOG(INFO) << "Camera initialized.";
    else {
        LOG(ERROR) << "Camera coordinate matrix initialize failed.";
        return false;
    }

    return true;
}
