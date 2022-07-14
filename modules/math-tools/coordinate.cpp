#include "math-tools/coordinate.h"

coordinate::RotationMatrix coordinate::camera_to_imu_rotation_matrix(rm_cam_to_imu_data);
coordinate::TranslationMatrix coordinate::camera_to_imu_translation_matrix(tm_cam_to_imu_data);

 bool coordinate::InitializeMatrix(const std::string &path) {
    cv::FileStorage r_and_t_matrix_param_config;
    r_and_t_matrix_param_config.open(path, cv::FileStorage::READ);
    if (!r_and_t_matrix_param_config.isOpened()) {
        LOG(ERROR) << "Failed to open rotation and translation matrix initialization file " << path << ".";
        return false;
    }
    cv::Mat rm_c_to_i, tm_c_to_i;
    r_and_t_matrix_param_config["RM_CAM_TO_IMU_DATA"] >> rm_c_to_i;
    r_and_t_matrix_param_config["TM_CAM_TO_IMU_DATA"] >> tm_c_to_i;
    cv::cv2eigen(rm_c_to_i, camera_to_imu_rotation_matrix);
    cv::cv2eigen(tm_c_to_i, camera_to_imu_translation_matrix);

    return true;
}
