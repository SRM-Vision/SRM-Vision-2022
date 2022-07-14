#ifndef PARAMETER_MAINTAIN_H_
#define PARAMETER_MAINTAIN_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include "controller-hero/controller_hero_debug.h"
#include "controller-infantry/controller_infantry_debug.h"
#include "controller-sentry-lower/controller_sentry_lower_debug.h"
#include "controller-sentry-higher/controller_sentry_higher_debug.h"


// predictor rune needed data structure
#include <predictor-rune/predictor-rune.h>


// coordinate, image_provider isn't contained in this file, since it is been welly sealed
/// \brief This is parameter maintain class, other debug class is friend to this class
/// \note Don't use this class alone. Coordinate, image_provider isn't contained in this file, since it is been welly sealed.
class ParameterMaintain
{
public:
    explicit ParameterMaintain(const std::string& controller_type_name)
    {
        rune_detector_config_path = "../config/" + controller_type_name +"/rune-detector-param.yaml";
        armor_predictor_config_path = "../config/" + controller_type_name + "/predict-param.yaml";
        rune_predictor_config_path = "../config/" + controller_type_name + "/rune-predictor-param.yaml";

        initDetectorRuneParameters();
        initPredictorRuneParameters();
        initPredictorArmorParameters();
        initPredictorOutpostParameters();
    }

private:
    cv::FileStorage config_;
    friend class ControllerInfantryDebug;
    friend class RuneDetectorDebug;
    friend class RunePredictorDebug;
    friend class OutpostPredictorDebug;

private:   ///< detector rune debug
    std::string rune_detector_config_path;
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

private:  ///< predictor rune debug
    std::string rune_predictor_config_path;
    void initPredictorRuneParameters();
    void savePredictorRuneParameters();
    predictor::rune::RotationalSpeed rotational_speed_;

private:  ///< predictor armor debug
    std::string armor_predictor_config_path;
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

private:  ///< predictor outpost debug
    const std::string outpost_predictor_config_path = "../config/hero/outpost-param.yaml";
    void initPredictorOutpostParameters();
    void savePredictorOutpostParameters();
    double outpost_shoot_delay_ = 0.02;
    double delta_pitch_up_ = 0.0;
    double delta_pitch_down_ = 0.0;
    double delta_yaw_left_ = 0.0;
    double delta_yaw_right_ = 0.0;
};

#endif