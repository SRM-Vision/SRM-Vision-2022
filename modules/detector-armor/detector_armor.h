/**
 * Armor detector header.
 * \author anonymity, screw-44
 * \date 2022.1.28
 */

#ifndef DETECTOR_ARMOR_H_
#define DETECTOR_ARMOR_H_

#include <opencv2/core.hpp>
#include <NvInfer.h>
#include "data-structure/bbox_t.h"
#include "lang-feature-extension/disable-constructors.h"

/// \brief Armor detector based on TensorRT.
class ArmorDetector : NO_COPY, NO_MOVE {
    static constexpr int kTopkNum = 128;
    static constexpr float kKeepThreshold = 0.1f;

public:
    ArmorDetector() : engine_(),
                      context_(),
                      device_buffer_(),
                      output_buffer_(),
                      stream_(),
                      input_index_(),
                      output_index_(),
                      input_size_(),
                      output_size_() {}

    ~ArmorDetector();

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
    std::vector<bbox_t> operator()(const cv::Mat &image) const;

    void UpdateROI(const cv::Rect &roi);

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

    cv::Rect roi_;

};

#endif  // DETECTOR_ARMOR_H_
