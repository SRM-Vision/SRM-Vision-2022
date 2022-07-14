#ifndef PARAMETER_MAINTAIN_H_
#define PARAMETER_MAINTAIN_H_

#include <iostream>
#include <opencv2/opencv.hpp>

// predictor rune needed data structure
#include <predictor-rune/predictor-rune.h>


// coordinate, image_provider isn't contained in this file, since it is been welly sealed
/// \brief This is parameter maintain class, other debug class is friend to this class
/// \note Don't use this class alone. Coordinate, image_provider isn't contained in this file, since it is been welly sealed.
class ParameterMaintain : NO_COPY, NO_MOVE
{
    ParameterMaintain(const std::string& controller_type_name="infantry")
    {
        controller_type_name_ = controller_type_name;
        initDetectorRuneParameters();
        initPredictorRuneParameters();
        initPredictorArmorParameters();
    }
private:
    cv::FileStorage config_;
    cv::string controller_type_name_;
private:   // detector rune debug
    const std::string rune_detector_config_path = "../config/" + controller_type_name_ +"/rune-detector-param.yaml";
    void initDetectorRuneParameters();
    void saveDetectorRuneParameters();
    // Trackbar value cache.
    int split_gray_thresh_;  ///< Binarization threshold.
    int min_bounding_box_area_;
    int min_fan_area_;
    int max_fan_area_;
    int min_armor_area_;
    int max_armor_area_;
    int kernel_size_;  ///< MorphologyEx kernel size
    int min_R_area_;
    int max_R_area_;
    int max_R_wh_deviation_;  ///< Maximum energy center encircle rect weight - height deviation.
    double min_armor_wh_ratio_;  ///< Minimum armor weight / height ratio.
    double max_armor_wh_ratio_;  ///< Maximum armor weight / height ratio.
    double min_fan_wh_ratio_;  ///< Minimum fan rect weight / height ratio.
    double max_fan_wh_ratio_;  ///< Maximum fan rect weight / height ratio.
    int delta_u_;  ///< Horizontal ballistic compensation
    int delta_v_;  ///< Vertical ballistic compensation

private:  // predictor rune debug
    const std::string rune_predictor_config_path = "../config/" + controller_type_name_ + "/rune-predictor-param.yaml";
    void initPredictorRuneParameters();
    void savePredictorRuneParameters();
    predictor::rune::RotationalSpeed rotational_speed_;

private:  // predictor armor debug
    const std::string armor_predictor_config_path = "../config/" + controller_type_name_ + "/predict-param.yaml";
    void initPredictorArmorParameters();
    void savePredictorArmorParameters();
    double p_xz_noise_ = 0.01;
    double p_y_noise_ = 0.01;
    double p_x_speed_noise_ = 10;
    double p_y_speed_noise_ = 10;
    double m_x_noise_ = 1;
    double m_y_noise_ = 1;
    double m_z_noise_ = 800;
    double shoot_delay_ = 0.02;

};

#endif