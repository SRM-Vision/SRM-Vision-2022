/*
    Create by wangyw15 on 29.03.2023
    Detect rune through yolo network
    Uses yolo-network moudle
*/
#ifndef DETECTOR_RUNE_YOLO_H_
#define DETECTOR_RUNE_YOLO_H_

#include <NvInfer.h>
#include <lang-feature-extension/disable-constructors.h>
#include "digital-twin/facilities/power_rune.h"
#include "data-structure/frame.h"
#include <lang-feature-extension/attr-reader.h>
#include <opencv2/opencv.hpp>
#include "../yolo-network/yolo_network.h"

struct BuffObject
{
    cv::Point2f apex[5];
    cv::Rect_<float> rect;
    int cls;
    int color;
    float prob;
    std::vector<cv::Point2f> pts;
};

class RuneDetectorYolo : NO_COPY, NO_MOVE {
    static constexpr int kTopkNum = 128;
    static constexpr float kKeepThreshold = 0.1f;
    static constexpr int INPUT_W = 416;    // Width of input
    static constexpr int INPUT_H = 416;    // Height of input
    static constexpr int NUM_CLASSES = 2;  // Number of classes
    static constexpr int NUM_COLORS = 2;   // Number of color
    static constexpr int TOPK = 32;       // TopK
    static constexpr float NMS_THRESH  = 0.1;
    static constexpr float BBOX_CONF_THRESH = 0.95;
    static constexpr float MERGE_CONF_ERROR = 0.15;
    static constexpr float MERGE_MIN_IOU = 0.2;

public:
    RuneDetectorYolo() : engine_(),
                            context_(),
                            device_buffer_(),
                            output_buffer_(),
                            stream_(),
                            input_index_(),
                            output_index_(),
                            input_size_(),
                            output_size_() {}

    ~RuneDetectorYolo();

    /**
     * \brief Load and initialize model.
     * \param [in] onnx_file ONNX file path.
     */
    bool Initialize(const std::string &onnx_file);



    /**
     * \brief Run detector once.
     * \param [in] frame Input original frame.
     * \return Output power rune data.
     */
    [[nodiscard]] PowerRune Run(Entity::Colors color, Frame &frame);

    // model part
private:
    YoloNetwork yolo_network_;

    // traditional part
private:
    cv::Point2f energy_center_r_;
    cv::Point2f armor_center_p_;
    cv::Point2f rtp_vec_;
    double time_gap_{};  ///< Time gap between two valid frames.
    double current_time_{};
    std::chrono::high_resolution_clock::time_point last_time_{};  ///< Last frame's time.

    int clockwise_ = 0;

    void FindRotateDirection();
};


#endif // DETECTOR_RUNE_YOLO_H_
