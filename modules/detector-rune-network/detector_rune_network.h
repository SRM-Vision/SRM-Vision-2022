//
// Created by screw on 2022/7/16.
//

#ifndef DETECTOR_RUNE_NETWORK_H_
#define DETECTOR_RUNE_NETWORK_H_

#include <NvInfer.h>
#include <lang-feature-extension/disable-constructors.h>
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
    void Initialize(const std::string &onnx_file);

    /**
     * \brief Run detection model.
     * \param [in] image Input image.
     * \return 4-point structures in a vector.
     */
    std::vector<BuffObject> operator()(const cv::Mat &image) const;


private:
    void BuildEngineFromONNX(const std::string &);

    void BuildEngineFromCache(const std::string &);

    void CacheEngine(const std::string &);

    nvinfer1::ICudaEngine *engine_;         ///< CUDA engine handle.
    nvinfer1::IExecutionContext *context_;  ///< CUDA execution context handle.

    mutable void *device_buffer_[2];
    float *output_buffer_;
    cudaStream_t stream_;
    int input_index_, output_index_;
    size_t input_size_, output_size_;
};


#endif //DETECTOR_RUNE_NETWORK_H_
