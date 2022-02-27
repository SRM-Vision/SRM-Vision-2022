#include <filesystem>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <NvOnnxParser.h>
#include <logger.h>
#include <glog/logging.h>
#include "detector_armor.h"

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
    std::filesystem::path onnx_file_path(onnx_file);
    auto cache_file_path = onnx_file_path;
    cache_file_path.replace_extension("cache");

    if (std::filesystem::exists(cache_file_path)) {
        BuildEngineFromCache(cache_file_path.c_str());
    } else {
        BuildEngineFromONNX(onnx_file_path.c_str());
        CacheEngine(cache_file_path.c_str());
    }

    TRT_ASSERT((context_ = engine_->createExecutionContext()) != nullptr)
    TRT_ASSERT((input_index_ = engine_->getBindingIndex("input")) == 0)
    TRT_ASSERT((output_index_ = engine_->getBindingIndex("output-topk")) == 1)

    auto input_dims = engine_->getBindingDimensions(input_index_);
    auto output_dims = engine_->getBindingDimensions(output_index_);
    input_size_ = get_dims_size(input_dims);
    output_size_ = get_dims_size(output_dims);

    TRT_ASSERT(cudaMalloc(&device_buffer_[input_index_], input_size_ * sizeof(float)) == 0)
    TRT_ASSERT(cudaMalloc(&device_buffer_[output_index_], output_size_ * sizeof(float)) == 0)
    TRT_ASSERT(cudaStreamCreate(&stream_) == 0)

    output_buffer_ = new float[output_size_];

    TRT_ASSERT(output_buffer_ != nullptr)
}

ArmorDetector::~ArmorDetector() {
    delete[] output_buffer_;

    cudaStreamDestroy(stream_);
    cudaFree(device_buffer_[output_index_]);
    cudaFree(device_buffer_[input_index_]);

    engine_->destroy();
}

void ArmorDetector::BuildEngineFromONNX(const std::string &onnx_file) {
    LOG(INFO) << "Engine will be built from ONNX.";
    auto builder = nvinfer1::createInferBuilder(sample::gLogger);
    TRT_ASSERT(builder != nullptr)

    const auto explicitBatch = 1U <<
                                  static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);
    TRT_ASSERT(network != nullptr)

    auto parser = nvonnxparser::createParser(*network, sample::gLogger);
    TRT_ASSERT(parser != nullptr)

    parser->parseFromFile(onnx_file.c_str(),
                          static_cast<int>(nvinfer1::ILogger::Severity::kINFO));

    auto yolov5_output = network->getOutput(0);

    auto slice_layer = network->addSlice(*yolov5_output,
                                         nvinfer1::Dims3{0, 0, 8},
                                         nvinfer1::Dims3{1, 15120, 1},
                                         nvinfer1::Dims3{1, 1, 1});

    auto yolov5_conf = slice_layer->getOutput(0);

    auto shuffle_layer = network->addShuffle(*yolov5_conf);
    shuffle_layer->setReshapeDimensions(nvinfer1::Dims2{1, 15120});

    yolov5_conf = shuffle_layer->getOutput(0);

    auto topk_layer = network->addTopK(*yolov5_conf,
                                       nvinfer1::TopKOperation::kMAX,
                                       kTopkNum,
                                       1 << 1);

    auto topk_idx = topk_layer->getOutput(1);

    auto gather_layer = network->addGather(*yolov5_output, *topk_idx, 1);
    gather_layer->setNbElementWiseDims(1);

    auto yolov5_output_topk = gather_layer->getOutput(0);
    yolov5_output_topk->setName("output-topk");

    network->getInput(0)->setName("input");
    network->markOutput(*yolov5_output_topk);
    network->unmarkOutput(*yolov5_output);

    auto config = builder->createBuilderConfig();

    if (builder->platformHasFastFp16()) {
        LOG(INFO) << "Platform supports fp16, fp16 is enabled.";
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    } else {
        LOG(INFO) << "Platform does not support fp16, enable fp32 instead.";
    }

    size_t free, total;
    cuMemGetInfo(&free, &total);

    LOG(INFO) << "GPU memory total: " << (total >> 20) << "MB, free: " << (free >> 20) << "MB.";
    LOG(INFO) << "Max workspace size will use all of free GPU memory.";

    config->setMaxWorkspaceSize(free >> 1);

    TRT_ASSERT((engine_ = builder->buildEngineWithConfig(*network, *config)) != nullptr)

    config->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();
}

void ArmorDetector::BuildEngineFromCache(const std::string &cache_file) {
    LOG(INFO) << "Engine will be built from cache.";
    std::ifstream ifs(cache_file, std::ios::binary);
    ifs.seekg(0, std::ios::end);
    size_t sz = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    auto buffer = std::make_unique<char[]>(sz);
    ifs.read(buffer.get(), (std::streamsize) sz);
    auto runtime = nvinfer1::createInferRuntime(sample::gLogger);

    TRT_ASSERT(runtime != nullptr)
    TRT_ASSERT((engine_ = runtime->deserializeCudaEngine(buffer.get(), sz)) != nullptr)

    runtime->destroy();
}

void ArmorDetector::CacheEngine(const std::string &cache_file) {
    auto engine_buffer = engine_->serialize();
    TRT_ASSERT(engine_buffer != nullptr)

    std::ofstream ofs(cache_file, std::ios::binary);
    ofs.write(static_cast<const char *>(engine_buffer->data()), (std::streamsize) engine_buffer->size());
    engine_buffer->destroy();
}

std::vector<bbox_t> ArmorDetector::operator()(const cv::Mat &image) const {
    // Pre-process. [bgr2rgb & resize]
    cv::Mat x;
    float fx = (float) image.cols / 640.f, fy = (float) image.rows / 384.f;
    cv::cvtColor(image, x, cv::COLOR_BGR2RGB);

    if (image.cols != 640 || image.rows != 384)
        cv::resize(x, x, {640, 384});

    x.convertTo(x, CV_32F);

    // Predict model.
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

    cudaMemcpyAsync(device_buffer_[input_index_], x.data, input_size_ * sizeof(float), cudaMemcpyHostToDevice, stream_);
    context_->enqueue(1, device_buffer_, stream_, nullptr);
    cudaMemcpyAsync(output_buffer_, device_buffer_[output_index_], output_size_ * sizeof(float), cudaMemcpyDeviceToHost,
                    stream_);
    cudaStreamSynchronize(stream_);

    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    auto dur = end - start;
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(dur);

#if ELPP_FEATURE_PERFORMANCE_TRACKING
    LOG(INFO) << "Armor detector inference time : [" << (double) time.count() / 1000.0 << " ms]";
#endif

    // Post-process. [nms]
    std::vector<bbox_t> result;

    result.reserve(kTopkNum);
    std::vector<uint8_t> removed(kTopkNum);

    for (int i = 0; i < kTopkNum; ++i) {
        auto *box_buffer = output_buffer_ + i * 20;  // 20 -> 23

        if (box_buffer[8] < inv_sigmoid(kKeepThreshold))
            break;
        else if (removed[i])
            continue;

        result.emplace_back();
        auto &box = result.back();

        memcpy(&box.points, box_buffer, 8 * sizeof(float));

        for (auto &point: box.points)
            point.x *= fx, point.y *= fy;

        box.confidence = sigmoid(box_buffer[8]);
        box.color = argmax(box_buffer + 9, 4);
        box.id = argmax(box_buffer + 13, 7);

        for (int j = i + 1; j < kTopkNum; ++j) {
            auto *box2_buffer = output_buffer_ + j * 20;
            if (box2_buffer[8] < inv_sigmoid(kKeepThreshold))
                break;
            else if (removed[j])
                continue;
            if (is_overlap(box_buffer, box2_buffer))
                removed[j] = true;
        }
    }

    return result;
}
