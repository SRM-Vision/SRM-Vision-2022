#include <opencv2/videoio.hpp>
#include "image-provider-base/image-provider-factory.h"
#include "image-provider-video.h"

/**
 * \warning Image provider registry will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
[[maybe_unused]] ImageProviderRegistry<ImageProviderVideo> ImageProviderVideo::registry_ =
        ImageProviderRegistry<ImageProviderVideo>("video");

ImageProviderVideo::~ImageProviderVideo() {
    if (video_.isOpened()) {
        video_.release();
        intrinsic_matrix_.release();
        distortion_matrix_.release();
    }
}

bool ImageProviderVideo::Initialize(const std::string &file_path) {
    // Load initialization configuration file.
    cv::FileStorage video_init_config;
    video_init_config.open(file_path, cv::FileStorage::READ);
    if (!video_init_config.isOpened()) {
        LOG(ERROR) << "Failed to open camera initialization file " << file_path << ".";
        return false;
    }

    // Load global cameras' configuration file.
    std::string all_cams_config_file;
    video_init_config["ALL_CAMS_CONFIG_FILE"] >> all_cams_config_file;
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
    video_init_config["ALL_LENS_CONFIG_FILE"] >> all_lens_config_file;
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

    // Get matrix for PnP.
    std::string len_type;
    all_cams_config[video_init_config["CAMERA"]]["LEN"] >> len_type;
    all_lens_config[len_type]["IntrinsicMatrix"] >> intrinsic_matrix_;
    all_lens_config[len_type]["DistortionMatrix"] >> distortion_matrix_;
    if (intrinsic_matrix_.empty() || distortion_matrix_.empty()) {
        LOG(ERROR) << "Camera len configurations not found.";
        intrinsic_matrix_.release();
        distortion_matrix_.release();
        return false;
    }

    // Open video file.
    std::string video_file;
    video_init_config["VIDEO"] >> video_file;
    if (video_file.empty()) {
        LOG(ERROR) << "Video configuration not found.";
        intrinsic_matrix_.release();
        distortion_matrix_.release();
        return false;
    }
    video_.open(video_file);
    if (!video_.isOpened()) {
        LOG(ERROR) << "Failed to open video file " << video_file << ".";
        intrinsic_matrix_.release();
        distortion_matrix_.release();
        return false;
    }

    return true;
}
