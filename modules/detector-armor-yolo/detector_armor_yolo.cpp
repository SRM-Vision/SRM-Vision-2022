#include <filesystem>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <NvOnnxParser.h>
#include <logger.h>
#include <glog/logging.h>
#include "detector_armor.h"
#include "yolo-network/yolo_network.h"

#define TRT_ASSERT(expr)                                              \
    if(!(expr)) {                                                     \
        LOG(ERROR) << "TensorRT assertion failed: " << #expr << ".";  \
        exit(-1);                                                     \
    }

static inline size_t get_dims_size(const nvinfer1::Dims &dims) {
    size_t sz = 1;
    for (int i = 0; i < dims.nbDims; ++i) sz *= dims.d[i];
    return sz;
}

template<class F, class T, class ...Ts>
T reduce(F &&func, T x, Ts... xs) {
    if constexpr (sizeof...(Ts) > 0) {
        return func(x, reduce(std::forward<F>(func), xs...));
    } else {
        return x;
    }
}

template<class T, class ...Ts>
T reduce_max(T x, Ts... xs) {
    return reduce([](auto &&a, auto &&b) { return std::max(a, b); }, x, xs...);
}

template<class T, class ...Ts>
T reduce_min(T x, Ts... xs) {
    return reduce([](auto &&a, auto &&b) { return std::min(a, b); }, x, xs...);
}

static inline bool is_overlap(const float pts1[8], const float pts2[8]) {
    cv::Rect2f bbox1, bbox2;

    bbox1.x = reduce_min(pts1[0], pts1[2], pts1[4], pts1[6]);
    bbox1.y = reduce_min(pts1[1], pts1[3], pts1[5], pts1[7]);
    bbox1.width = reduce_max(pts1[0], pts1[2], pts1[4], pts1[6]) - bbox1.x;
    bbox1.height = reduce_max(pts1[1], pts1[3], pts1[5], pts1[7]) - bbox1.y;

    bbox2.x = reduce_min(pts2[0], pts2[2], pts2[4], pts2[6]);
    bbox2.y = reduce_min(pts2[1], pts2[3], pts2[5], pts2[7]);
    bbox2.width = reduce_max(pts2[0], pts2[2], pts2[4], pts2[6]) - bbox2.x;
    bbox2.height = reduce_max(pts2[1], pts2[3], pts2[5], pts2[7]) - bbox2.y;
    return (bbox1 & bbox2).area() > 0;
}

static inline int argmax(const float *ptr, int len) {
    int arg_max = 0;
    for (int i = 1; i < len; ++i)
        if (ptr[i] > ptr[arg_max])
            arg_max = i;
    return arg_max;
}

inline constexpr float inv_sigmoid(float x) {
    return -std::log(1 / x - 1);
}

inline constexpr float sigmoid(float x) {
    return 1 / (1 + std::exp(-x));
}

void ArmorDetector::Initialize(const std::string &onnx_file) {
    network_ = YoloNetwork(onnx_file, 36, 4);
}

ArmorDetector::~ArmorDetector() {

}

std::vector<bbox_t> ArmorDetector::operator()(const cv::Mat &image) const {
    // ROI
    cv::Mat roi_image;
    if(roi_.size() != cv::Size(0,0))
        roi_image = image(roi_);
    else
        roi_image = image;

    // Pre-process. [bgr2rgb & resize]
    cv::Mat x;
    float fx = (float) roi_image.cols / 640.f, fy = (float) roi_image.rows / 384.f;
    cv::cvtColor(roi_image, x, cv::COLOR_BGR2RGB);

    if (roi_image.cols != 640 || roi_image.rows != 384)
        cv::resize(x, x, {640, 384});

    x.convertTo(x, CV_32F);

    // Run model.
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

    auto rawResult = network_.Inference(x);

    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    auto dur = end - start;
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(dur);

    // Post-process.
    std::vector<bbox_t> result;

    for (auto &r: rawResult) {
        result.push_back({ r.pts, r.prob, r.cls / 9, r.cls % 9 });
    }

    // ROI reduction
    if(!roi_.empty()){
        for(auto &bbox:result){
            for(auto &point:bbox.points){
                point += cv::Point2f(roi_.tl());
            }
        }
    }

    return result;
}

void ArmorDetector::UpdateROI(const cv::Rect &roi) {
    roi_ = roi;
}
