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

RuneDetectorNetwork::~RuneDetectorNetwork() {
    delete[] output_buffer_;

    cudaStreamDestroy(stream_);
    cudaFree(device_buffer_[output_index_]);
    cudaFree(device_buffer_[input_index_]);

    engine_->destroy();
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

PowerRune RuneDetectorNetwork::Run(Entity::Colors color, Frame &frame) {
    BuffObject buff_from_model = ModelRun(frame.image);

    // mean filter to get stable center R
    if(abs(buff_from_model.apex[2].x - energy_center_r_.x) < 100)
        energy_center_r_ = buff_from_model.apex[2] /2 + energy_center_r_ / 2;
    else
        energy_center_r_ = buff_from_model.apex[2];

    for (int i=0; i<5; i++)
    {
        if (i == 2)
            continue;
        armor_center_p_ += buff_from_model.apex[i];
    }
    armor_center_p_ /= 5;
    rtp_vec_ = armor_center_p_ - energy_center_r_;

    if (!clockwise_)
        FindRotateDirection();

    return {color,
            clockwise_,
            time_gap_,
            current_time_,
            rtp_vec_,
            energy_center_r_,
            armor_center_p_,
            cv::Point2f(float(frame.image.cols >> 1), float(frame.image.rows >> 1))};
}


/**
 * @brief Generate grids and stride.
 * @param target_w Width of input.
 * @param target_h Height of input.
 * @param strides A vector of stride.
 * @param grid_strides Grid stride generated in this function.
 */
void RuneDetectorNetwork::generate_grids_and_stride(std::vector<int>& strides, std::vector<GridAndStride>& grid_strides)
{
    for (auto stride : strides)
    {
        int num_grid_w = INPUT_W / stride;
        int num_grid_h = INPUT_H / stride;

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
void RuneDetectorNetwork::generateYoloxProposals(
        std::vector<GridAndStride> grid_strides, const float* feat_ptr,
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

        if (box_prob >= BBOX_CONF_THRESH)
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

void RuneDetectorNetwork::qsort_descent_inplace(std::vector<BuffObject>& faceobjects, int left, int right)
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

void RuneDetectorNetwork::qsort_descent_inplace(std::vector<BuffObject>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}


void RuneDetectorNetwork::nms_sorted_bboxes(std::vector<BuffObject>& faceobjects, std::vector<int>& picked)
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

            if (iou > NMS_THRESH || isnan(iou))
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

BuffObject RuneDetectorNetwork::ModelRun(const cv::Mat &image)
{
    auto current_time_chrono = std::chrono::high_resolution_clock::now();
    current_time_ = double(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    time_gap_ = (static_cast<std::chrono::duration<double, std::milli>>(
            current_time_chrono - last_time_)).count();
    last_time_ = current_time_chrono;
    current_time_ /= 1000;

    cv::Mat image_ = image.clone();
    cv::Rect roi_rect = cv::Rect(0, 0, image.cols, image.rows);
    // last detection is valid.
    if (energy_center_r_ != cv::Point2f(0, 0)) {
        roi_point_tl_ = cv::Point2i(std::max(0, int(energy_center_r_.x - 213)), std::max(0, int(energy_center_r_.y - 213)));
        roi_rect = cv::Rect(roi_point_tl_.x, roi_point_tl_.y, 2 * 213, 2 * 213) &
                            cv::Rect(0, 0, image.cols, image.rows);
    }
    image_ = image(roi_rect);  // Use ROI

    // Pre-process. [resize]
    float fx = (float) image_.cols / 416.f; float fy = (float) image_.rows / 416.f;
    if (image.cols != 416 || image.rows != 416)
        cv::resize(image_, image_, {416, 416});
    image_.convertTo(image_, CV_32F);

    cv::Mat image_splits[3];
    cv::split(image_, image_splits);
//    cv::cvtColor(image_, image_, cv::COLOR_BGR2RGB);

    auto *input_data = new float[416*416*3];
    //Copy img into blob
    for(auto & image_split : image_splits)
    {
        memcpy(input_data, image_split.data, INPUT_W * INPUT_H * sizeof(float));
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

    std::vector<int> strides = {8, 16, 32};
    std::vector<GridAndStride> grid_strides;

    generate_grids_and_stride(strides, grid_strides);
    generateYoloxProposals(grid_strides, output_buffer_,  results);

    qsort_descent_inplace(results);

    if (results.size() >= TOPK)
        results.resize(TOPK);

    std::vector<BuffObject> filtered;
    for (int i=0; i<results.size(); i++)
    {
        if (results[i].cls == 1)
        {
            filtered.push_back(results[i]);
        }
    }
    results = filtered;

    std::vector<int> picked;
    nms_sorted_bboxes(results, picked);
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
                pts_final[i].x = pts_final[i].x / (N / 5) * fx + roi_point_tl_.x;
                pts_final[i].y = pts_final[i].y / (N / 5) * fy + roi_point_tl_.y;
            }

            (*object).apex[0] = pts_final[0];
            (*object).apex[1] = pts_final[1];
            (*object).apex[2] = pts_final[2];
            (*object).apex[3] = pts_final[3];
            (*object).apex[4] = pts_final[4];
        }
        // (*object).area = (int)(calcTetragonArea((*object).apex));
    }


    // if vector is empty, return empty point.
    if (results.empty())
        return {};

    std::sort(results.begin(), results.end(), [](BuffObject a, BuffObject b){
        return a.prob > b.prob;
    });

    delete [] input_data;

    return results.at(0);
}


void RuneDetectorNetwork::FindRotateDirection() {
    static std::vector<cv::Point2f> r_to_p_vec;  ///< Vector Used for deciding rotation direction.
    const float kMaxRatio = 0.1;
    static float rune_radius_ = 120;

    // detection is not found.
    if (rtp_vec_ == cv::Point2f(0, 0))
        return;

    if (clockwise_ == 0) {
        r_to_p_vec.emplace_back(rtp_vec_);
    }
    // note:frame lost is deleted

    if (clockwise_ == 0 && static_cast<int>(r_to_p_vec.size()) > 20)
    {
        cv::Point2f first_rotation = r_to_p_vec[5];  // The fist five frames may be invalid.
        float radius, final_radius = 0;
        for (auto current_rotation = r_to_p_vec.begin() + 6; current_rotation != r_to_p_vec.end(); ++current_rotation) {
            double cross = first_rotation.cross(cv::Point2f(current_rotation->x, current_rotation->y));
            radius = algorithm::SqrtFloat(
                    current_rotation->x * current_rotation->x + current_rotation->y * current_rotation->y);
            final_radius +=
                    std::min(rune_radius_ * (1 + kMaxRatio), std::max(rune_radius_ * (1 - kMaxRatio), radius)) / 15;

            if (cross > 0.0)
                ++clockwise_;
            else if (cross < 0.0)
                --clockwise_;
        }
        if (clockwise_ > 8) {
            DLOG(INFO) << "Power rune's direction is clockwise." << " \tRadius: " << rune_radius_;
            clockwise_ = 1;
        } else if (clockwise_ < -8) {
            DLOG(INFO) << "Power rune's direction is anti-clockwise." << " \tRadius: " << rune_radius_;
            clockwise_ = -1;
        } else {
            clockwise_ = 0;
            DLOG(WARNING) << "Rotating direction is not decided!" << " \tRadius: " << rune_radius_;
            r_to_p_vec.clear();
        }
        rune_radius_ = final_radius;
    }
}



