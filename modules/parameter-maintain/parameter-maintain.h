#ifndef PARAMETER_MAINTAIN_H_
#define PARAMETER_MAINTAIN_H_

#include <iostream>
#include <opencv2/opencv.hpp>

// predictor rune needed data structure
#include <predictor-rune/predictor-rune.h>


// coordinate, image_provider isn't contained in this file, since it is been welly sealed
/// \brief This is parameter maintain class, other debug class is friend to this class
/// \note Don't use this class alone. Coordinate, image_provider isn't contained in this file, since it is been well-sealed.
class ParameterMaintain {
public:
    explicit ParameterMaintain(const std::string &controller_type_name) {
        rune_detector_config_path = "../config/infantry/rune-detector-param.yaml";
        rune_predictor_config_path = "../config/infantry/rune-predictor-param.yaml";
        armor_predictor_config_path = "../config/" + controller_type_name + "/predict-param.yaml";

        initDetectorRuneParameters();
        initPredictorRuneParameters();
        initPredictorArmorParameters();
        initPredictorOutpostParameters();
    }
    // should be same as the construct function
    void ManualInit(const std::string &controller_type_name) {
            rune_detector_config_path = "../config/infantry/rune-detector-param.yaml";
            rune_predictor_config_path = "../config/infantry/rune-predictor-param.yaml";
            armor_predictor_config_path = "../config/" + controller_type_name + "/predict-param.yaml";

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

    friend class ArmorPredictorDebug;

private:   ///< detector rune debug
    std::string rune_detector_config_path;

    bool initDetectorRuneParameters();

    bool saveDetectorRuneParameters();

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
    int delta_u_;  ///< Horizontal ballistic compensation.
    int delta_v_;  ///< Vertical ballistic compensation.
    int compensate_time_;

private:  ///< predictor rune debug
    std::string rune_predictor_config_path;

    bool initPredictorRuneParameters();

    bool savePredictorRuneParameters();

    // Trackbar value cache.
    predictor::rune::RotationalSpeed rotational_speed_;

private:  ///< predictor armor debug
    std::string armor_predictor_config_path;

    bool initPredictorArmorParameters();

    bool savePredictorArmorParameters();

    // Trackbar value cache.
    double p_xz_noise_ = 0.1;
    double p_y_noise_ = 0.1;
    double p_x_speed_noise_ = 10;
    double p_x_acceleration_noise_ = 100;
    double p_y_speed_noise_ = 10;
    double p_y_acceleration_noise_ = 100;
    double m_x_noise_ = 1;
    double m_y_noise_ = 1;
    double m_z_noise_ = 800;
    double shoot_delay_ = 0.02;

private:  ///< predictor outpost debug
    const std::string outpost_predictor_config_path = "../config/hero/outpost-param.yaml";

    bool initPredictorOutpostParameters();

    bool savePredictorOutpostParameters();

    // Trackbar value cache.
    double outpost_shoot_delay_3m_ = 0.2;
    double outpost_shoot_delay_5m_ = 0.02;
    double outpost_shoot_delay_6m_ = 0;
    double delta_pitch_up_ = 0.0;
    double delta_pitch_down_ = 0.0;
    double delta_yaw_left_ = 0.0;
    double delta_yaw_right_ = 0.0;
};

#endif