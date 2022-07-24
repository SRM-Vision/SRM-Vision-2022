//
// Created by screw on 2022/7/16.
//

#ifndef DETECTOR_RUNE_NETWORK_H_
#define DETECTOR_RUNE_NETWORK_H_

#include <NvInfer.h>
#include <lang-feature-extension/disable-constructors.h>
#include "digital-twin/facilities/power_rune.h"
#include "data-structure/frame.h"
#include <lang-feature-extension/attr-reader.h>
#include <opencv2/opencv.hpp>

struct BuffObject
{
    cv::Point2f apex[5];
    cv::Rect_<float> rect;
    int cls;
    int color;
    float prob;
    std::vector<cv::Point2f> pts;
};

class RuneDetectorNetwork : NO_COPY, NO_MOVE {
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
    RuneDetectorNetwork() : engine_(),
                            context_(),
                            device_buffer_(),
                            output_buffer_(),
                            stream_(),
                            input_index_(),
                            output_index_(),
                            input_size_(),
                            output_size_() {}

    ~RuneDetectorNetwork();

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
    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    void BuildEngineFromONNX(const std::string &);

    void BuildEngineFromCache(const std::string &);

    void CacheEngine(const std::string &);

    BuffObject ModelRun(const cv::Mat &image);

    static void generate_grids_and_stride(std::vector<int>& strides, std::vector<GridAndStride>& grid_strides);

    static void generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr,
                                       std::vector<BuffObject>& objects);

    void qsort_descent_inplace(std::vector<BuffObject>& faceobjects, int left, int right);

    void qsort_descent_inplace(std::vector<BuffObject>& objects);

    static void nms_sorted_bboxes(std::vector<BuffObject>& faceobjects, std::vector<int>& picked);
    nvinfer1::ICudaEngine *engine_;         ///< CUDA engine handle.
    nvinfer1::IExecutionContext *context_;  ///< CUDA execution context handle.

    mutable void *device_buffer_[2];
    float *output_buffer_;
    cudaStream_t stream_;
    int input_index_, output_index_;
    size_t input_size_, output_size_;
    const int roi_half_width = 213;  ///< 213 is corresponding to the 416*416 model input size.

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

    cv::Point2i roi_point_tl_;
    const float kMaxDeviation = 50;
};


#endif //DETECTOR_RUNE_NETWORK_H_