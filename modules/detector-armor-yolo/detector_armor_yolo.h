/**
 * Armor detector using yolo.
 * \author wangyw15, XY_CPP
 * \date 2023.03.29
 */

#ifndef DETECTOR_ARMOR_H_
#define DETECTOR_ARMOR_H_

#include <opencv2/core.hpp>
#include <NvInfer.h>
#include "data-structure/bbox_t.h"
#include "lang-feature-extension/disable-constructors.h"
#include "yolo-network/yolo_network.h"

/// \brief Armor detector based on TensorRT.
class ArmorDetector : NO_COPY, NO_MOVE {
    static constexpr int kTopkNum = 128;
    static constexpr float kKeepThreshold = 0.1f;

public:
    ArmorDetector() {}

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
    YoloNetwork network_;

    cv::Rect roi_;
};

#endif  // DETECTOR_ARMOR_H_
