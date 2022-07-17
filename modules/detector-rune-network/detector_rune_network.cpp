//
// Created by screw on 2022/7/16.
//

#include "detector_rune_network.h"
#include <filesystem>
#include <fstream>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <NvOnnxParser.h>
#include <logger.h>
#include <glog/logging.h>
#include <Eigen/Eigen>


// static constexpr int INPUT_W = 640;    // Width of input
// static constexpr int INPUT_H = 384;    // Height of input
static constexpr int INPUT_W = 416;    // Width of input
static constexpr int INPUT_H = 416;    // Height of input
static constexpr int NUM_CLASSES = 2;  // Number of classes
static constexpr int NUM_COLORS = 2;   // Number of color
static constexpr int TOPK = 128;       // TopK
static constexpr float NMS_THRESH  = 0.1;
static constexpr float BBOX_CONF_THRESH = 0.1;
static constexpr float MERGE_CONF_ERROR = 0.15;
static constexpr float MERGE_MIN_IOU = 0.2;

#define TRT_ASSERT(expr)                                              \
    if(!(expr)) {                                                     \
        LOG(ERROR) << "TensorRT assertion failed: " << #expr << ".";  \
        exit(-1);                                                     \
    }

static inline int argmax(const float *ptr, int len)
{
    int max_arg = 0;
    for (int i = 1; i < len; i++) {
        if (ptr[i] > ptr[max_arg]) max_arg = i;
    }
    return max_arg;
}


static inline size_t get_dims_size(const nvinfer1::Dims &dims) {
    size_t sz = 1;
    for (int i = 0; i < dims.nbDims; ++i) sz *= dims.d[i];
    return sz;
}

inline constexpr float inv_sigmoid(float x) {
    return -std::log(1 / x - 1);
}

inline constexpr float sigmoid(float x) {
    return 1 / (1 + std::exp(-x));
}


struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};


/**
 * @brief Generate grids and stride.
 * @param target_w Width of input.
 * @param target_h Height of input.
 * @param strides A vector of stride.
 * @param grid_strides Grid stride generated in this function.
 */
static void generate_grids_and_stride(const int target_w, const int target_h,
                                      std::vector<int>& strides, std::vector<GridAndStride>& grid_strides)
{
    for (auto stride : strides)
    {
        int num_grid_w = target_w / stride;
        int num_grid_h = target_h / stride;

        for (int g1 = 0; g1 < num_grid_h; g1++)
        {
            for (int g0 = 0; g0 < num_grid_w; g0++)
            {
                grid_strides.push_back((GridAndStride){g0, g1, stride});
            }
        }
    }
}


/**
 * @brief Generate Proposal
 * @param grid_strides Grid strides
 * @param feat_ptr Original predition result.
 * @param prob_threshold Confidence Threshold.
 * @param objects Objects proposed.
 */
static void generateYoloxProposals(
        std::vector<GridAndStride> grid_strides, const float* feat_ptr,
        float prob_threshold,
        std::vector<BuffObject>& objects)
{

    const int num_anchors = grid_strides.size();
    //Travel all the anchors
    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
    {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

        const int basic_pos = anchor_idx * (11 + NUM_COLORS + NUM_CLASSES);

        // yolox/models/yolo_head.py decode logic
        //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
        //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
        float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
        float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
        float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
        float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
        float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
        float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
        float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
        float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;
        float x_5 = (feat_ptr[basic_pos + 8] + grid0) * stride;
        float y_5 = (feat_ptr[basic_pos + 9] + grid1) * stride;

        int box_color = argmax(feat_ptr + basic_pos + 11, NUM_COLORS);
        int box_class = argmax(feat_ptr + basic_pos + 11 + NUM_COLORS, NUM_CLASSES);

        float box_objectness = (feat_ptr[basic_pos + 10]);

        float color_conf = (feat_ptr[basic_pos + 11 + box_color]);
        float cls_conf = (feat_ptr[basic_pos + 11 + NUM_COLORS + box_class]);

        // cout<<box_objectness<<endl;
        // float box_prob = (box_objectness + cls_conf + color_conf) / 3.0;
        float box_prob = box_objectness;

        if (box_prob >= prob_threshold)
        {
            BuffObject obj;

            Eigen::Matrix<float,3,5> apex_norm;
            Eigen::Matrix<float,3,5> apex_dst;

            apex_norm << x_1,x_2,x_3,x_4,x_5,
                    y_1,y_2,y_3,y_4,y_5,
                    1,1,1,1,1;

            apex_dst = apex_norm;

            for (int i = 0; i < 5; i++)
                obj.apex[i] = cv::Point2f(apex_dst(0,i),apex_dst(1,i));
            for (int i = 0; i < 5; i++)
            {
                obj.apex[i] = cv::Point2f(apex_dst(0,i),apex_dst(1,i));
                obj.pts.push_back(obj.apex[i]);
            }
            std::vector<cv::Point2f> tmp(obj.apex,obj.apex + 5);
            obj.rect = cv::boundingRect(tmp);

            obj.cls = box_class;
            obj.color = box_color;
            obj.prob = box_prob;

            objects.push_back(obj);
        }

    } // point anchor loop
}

static void qsort_descent_inplace(std::vector<BuffObject>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

#pragma omp parallel sections
    {
#pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
#pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

static void qsort_descent_inplace(std::vector<BuffObject>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}


static void nms_sorted_bboxes(std::vector<BuffObject>& faceobjects, std::vector<int>& picked,
                              float nms_threshold)
{
    picked.clear();
    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        std::vector<cv::Point2f> object_apex_tmp(faceobjects[i].apex, faceobjects[i].apex + 5);
        areas[i] = contourArea(object_apex_tmp);
        // areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        BuffObject& a = faceobjects[i];
        std::vector<cv::Point2f> apex_a(a.apex, a.apex + 5);
        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            BuffObject& b = faceobjects[picked[j]];
            std::vector<cv::Point2f> apex_b(b.apex, b.apex + 5);
            std::vector<cv::Point2f> apex_inter;
            // intersection over union
            // float inter_area = intersection_area(a, b);
            // float union_area = areas[i] + areas[picked[j]] - inter_area;
            //TODO:此处耗时较长，大约1ms，可以尝试使用其他方法计算IOU与多边形面积
            float inter_area = intersectConvexConvex(apex_a,apex_b,apex_inter);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            float iou = inter_area / union_area;

            if (iou > nms_threshold || isnan(iou))
            {
                keep = 0;
                //Stored for Merge
                if (iou > MERGE_MIN_IOU && abs(a.prob - b.prob) < MERGE_CONF_ERROR
                    && a.cls == b.cls && a.color == b.color)
                {
                    for (int i = 0; i < 5; i++)
                    {
                        b.pts.push_back(a.apex[i]);
                    }
                }
                // cout<<b.pts_x.size()<<endl;
            }
        }

        if (keep)
            picked.push_back(i);
    }
}




RuneDetectorNetwork::~RuneDetectorNetwork() {
    delete[] output_buffer_;

    cudaStreamDestroy(stream_);
    cudaFree(device_buffer_[output_index_]);
    cudaFree(device_buffer_[input_index_]);

    engine_->destroy();
}

void RuneDetectorNetwork::Initialize(const std::string &onnx_file) {
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
    TRT_ASSERT((output_index_ = engine_->getBindingIndex("output")) == 1)

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

std::vector<BuffObject> RuneDetectorNetwork::operator()(const cv::Mat &image) const {

    // Pre-process. [bgr2rgb & resize]
    cv::Mat x = image.clone();
    if (image.cols != 416 || image.rows != 416)
        cv::resize(x, x, {416, 416});
    x.convertTo(x, CV_32F);
    cv::Mat x_split[3];
    cv::split(x, x_split);
    //cv::cvtColor(image, x, cv::COLOR_BGR2RGB);


    float *input_data = new float[416*416*3];
    //Copy img into blob
    for(int c = 0;c < 3;c++)
    {
        memcpy(input_data, x_split[c].data, INPUT_W * INPUT_H * sizeof(float));
        input_data += INPUT_W * INPUT_H;
    }
    input_data -= INPUT_W * INPUT_H * 3;

    // Run model.
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

    cudaMemcpyAsync(device_buffer_[input_index_], input_data, input_size_ * sizeof(float), cudaMemcpyHostToDevice, stream_);
    context_->enqueue(1, device_buffer_, stream_, nullptr);
    cudaMemcpyAsync(output_buffer_, device_buffer_[output_index_], output_size_ * sizeof(float), cudaMemcpyDeviceToHost,
                    stream_);
    cudaStreamSynchronize(stream_);

    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    auto dur = end - start;
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(dur);
    DLOG(INFO) << "rune model detection time cost:" << time.count();

    // Post-process. [nms]
    std::vector<BuffObject> results;

    results.reserve(kTopkNum);
    std::vector<uint8_t> removed(kTopkNum);

    std::vector<int> strides = {8, 16, 32};
    std::vector<GridAndStride> grid_strides;

    generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
    generateYoloxProposals(grid_strides, output_buffer_, BBOX_CONF_THRESH, results);

    qsort_descent_inplace(results);

    std::cout << "results nums is " << results.size() << std::endl;

    cv::Mat draw_image = image.clone();
    for (auto result : results)
    {
        for (auto apex : result.apex)
        {
            cv::circle(draw_image, apex, 2, cv::Scalar(100, 150, 200), 3);
        }
    }

    cv::imshow("detector rune network", draw_image);


    if (results.size() >= TOPK)
        results.resize(TOPK);
    std::vector<int> picked;
    nms_sorted_bboxes(results, picked, NMS_THRESH);
    int count = picked.size();
    results.resize(count);

    for (int i = 0; i < count; i++)
    {
        results[i] = results[picked[i]];
    }

    for (auto object = results.begin(); object != results.end(); ++object)
    {
        if ((*object).pts.size() >= 10)
        {
            auto N = (*object).pts.size();
            cv::Point2f pts_final[5];

            for (int i = 0; i < N; i++)
            {
                pts_final[i % 5]+=(*object).pts[i];
            }

            for (int i = 0; i < 5; i++)
            {
                pts_final[i].x = pts_final[i].x / (N / 5);
                pts_final[i].y = pts_final[i].y / (N / 5);
            }

            (*object).apex[0] = pts_final[0];
            (*object).apex[1] = pts_final[1];
            (*object).apex[2] = pts_final[2];
            (*object).apex[3] = pts_final[3];
            (*object).apex[4] = pts_final[4];
        }
        // (*object).area = (int)(calcTetragonArea((*object).apex));
    }




    return results;
}

void RuneDetectorNetwork::BuildEngineFromONNX(const std::string &onnx_file) {
    LOG(INFO) << "Engine will be built from ONNX.";
    auto builder = nvinfer1::createInferBuilder(sample::gLogger);
    TRT_ASSERT(builder != nullptr)

    const auto explicitBatch =
            1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);
    TRT_ASSERT(network != nullptr)

    auto parser = nvonnxparser::createParser(*network, sample::gLogger);
    TRT_ASSERT(parser != nullptr)

    parser->parseFromFile(onnx_file.c_str(),
                          static_cast<int>(nvinfer1::ILogger::Severity::kINFO));


    network->getInput(0)->setName("input");
    network->getOutput(0)->setName("output");

    auto config = builder->createBuilderConfig();

    config->setFlag(nvinfer1::BuilderFlag::kFP16);



//    if (builder->platformHasFastFp16()) {
//        LOG(INFO) << "Platform supports fp16, fp16 is enabled.";
//        config->setFlag(nvinfer1::BuilderFlag::kFP16);
//    } else {
//        LOG(INFO) << "Platform does not support fp16, enable fp32 instead.";
//    }

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

void RuneDetectorNetwork::BuildEngineFromCache(const std::string &cache_file) {
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

void RuneDetectorNetwork::CacheEngine(const std::string &cache_file) {
    auto engine_buffer = engine_->serialize();
    TRT_ASSERT(engine_buffer != nullptr)

    std::ofstream ofs(cache_file, std::ios::binary);
    ofs.write(static_cast<const char *>(engine_buffer->data()), (std::streamsize) engine_buffer->size());
    engine_buffer->destroy();
}

