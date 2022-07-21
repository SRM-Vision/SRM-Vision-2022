#include <opencv2/videoio.hpp>
#include "camera-base/camera-factory.h"
#include "image-provider-base/image-provider-factory.h"
#include "image-provider-camera.h"

/**
 * \warning Image provider registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ImageProviderRegistry<ImageProviderCamera> ImageProviderCamera::registry_ =
        ImageProviderRegistry<ImageProviderCamera>("camera");

ImageProviderCamera::~ImageProviderCamera() {
    if (camera_) {
        camera_->StopStream();
        camera_->CloseCamera();
        delete camera_;
        camera_ = nullptr;
        intrinsic_matrix_.release();
        distortion_matrix_.release();
        video_writer_.release();
    }
}

bool ImageProviderCamera::Initialize(const std::string &file_path, bool record) {
    // Load initialization configuration file.
    cv::FileStorage camera_init_config;
    camera_init_config.open(file_path, cv::FileStorage::READ);
    if (!camera_init_config.isOpened()) {
        LOG(ERROR) << "Failed to open camera initialization file " << file_path << ".";
        return false;
    }

    // Load global cameras' configuration file.
    std::string all_cams_config_file;
    camera_init_config["ALL_CAMS_CONFIG_FILE"] >> all_cams_config_file;
    if (all_cams_config_file.empty()) {
        LOG(ERROR) << "All cameras' config file configuration not found.";
        return false;
    }
    cv::FileStorage all_cams_config;
    all_cams_config.open(all_cams_config_file, cv::FileStorage::READ);
    if (!all_cams_config.isOpened()) {
        LOG(ERROR) << "Failed to open all cameras' config file " << all_cams_config_file << ".";
        return false;
    }

    // Load global lens' configuration file.
    std::string all_lens_config_file;
    camera_init_config["ALL_LENS_CONFIG_FILE"] >> all_lens_config_file;
    if (all_lens_config_file.empty()) {
        LOG(ERROR) << "All lens' config file configuration not found.";
        return false;
    }
    cv::FileStorage all_lens_config;
    all_lens_config.open(all_lens_config_file, cv::FileStorage::READ);
    if (!all_lens_config.isOpened()) {
        LOG(ERROR) << "Failed to open all lens' config file " << all_lens_config_file << ".";
        return false;
    }

    // Create a camera.
    std::string camera_type;
    all_cams_config[camera_init_config["CAMERA"]]["TYPE"] >> camera_type;
    if (camera_type.empty()) {
        LOG(ERROR) << "Camera type configuration not found.";
        return false;
    }
    camera_ = CameraFactory::Instance().CreateCamera(camera_type);
    if (!camera_) {
        LOG(ERROR) << "Failed to create camera object of type " << camera_type << ".";
        return false;
    }

    // Get matrix for PnP solving.
    std::string len_type;
    all_cams_config[camera_init_config["CAMERA"]]["LEN"] >> len_type;
    all_lens_config[len_type]["IntrinsicMatrix"] >> intrinsic_matrix_;
    all_lens_config[len_type]["DistortionMatrix"] >> distortion_matrix_;
    if (intrinsic_matrix_.empty() || distortion_matrix_.empty()) {
        LOG(ERROR) << "Camera len configurations not found.";
        delete camera_;
        camera_ = nullptr;
        intrinsic_matrix_.release();
        distortion_matrix_.release();
        return false;
    }

    // Open camera.
    std::string serial_number, camera_config_file;
    all_cams_config[camera_init_config["CAMERA"]]["SN"] >> serial_number;
    all_cams_config[camera_init_config["CAMERA"]]["CONFIG"] >> camera_config_file;
    if (serial_number.empty() || camera_config_file.empty()) {
        LOG(ERROR) << "Camera configurations not found.";
        delete camera_;
        camera_ = nullptr;
        intrinsic_matrix_.release();
        distortion_matrix_.release();
        return false;
    }
    if (!camera_->OpenCamera(serial_number, camera_config_file)) {
        delete camera_;
        camera_ = nullptr;
        intrinsic_matrix_.release();
        distortion_matrix_.release();
        return false;
    }

    // Start stream.
    if (!camera_->StartStream()) {
        camera_->CloseCamera();
        delete camera_;
        camera_ = nullptr;
        intrinsic_matrix_.release();
        distortion_matrix_.release();
        return false;
    }

    // open record
    if (true)
    {
        initRecord();
    }


    return true;
}
