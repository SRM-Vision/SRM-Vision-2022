/*
    Create by wangyw15 on 29.03.2023
    Detect rune through yolo network
    Uses yolo-network moudle
*/

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


bool RuneDetectorYolo::Initialize(const std::string &onnx_file) {
    yolo_network_ = YoloNetwork(onnx_file, 2, 5);
    
    return true;
}

RuneDetectorYolo::~RuneDetectorYolo() {
    
}

PowerRune RuneDetectorYolo::Run(Entity::Colors color, Frame &frame) {
    // calculate run time
    auto current_time_chrono = std::chrono::high_resolution_clock::now();
    current_time_ = double(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    time_gap_ = (static_cast<std::chrono::duration<double, std::milli>>(
            current_time_chrono - last_time_)).count();
    last_time_ = current_time_chrono;
    current_time_ /= 1000;

    // inference
    auto result = yolo_network_.Inference(frame.image);

    // empty result
    if (result.empty())
    {
        return PowerRune();
    }

    // mean filter to get stable center R
    if(abs(result[0].pts[3].x - energy_center_r_.x) < 100)
        energy_center_r_ = result[0].pts[3] / 2 + energy_center_r_ / 2;
    else
        energy_center_r_ = result[0].pts[3];

    // calculate the center of armor
    armor_center_p_ = cv::Point2f(0, 0);
    for (int i = 0; i < 5; i++)
    {
        if (i == 3)
            continue;
        armor_center_p_ += result[0].pts[i];
    }
    armor_center_p_ /= 4;
    rtp_vec_ = armor_center_p_ - energy_center_r_;

    if (!clockwise_)
        FindRotateDirection();

//    if (armor_center_p_ != cv::Point2f(0, 0))
//    {
//        time_gap_ = static_cast<double>(frame.time_stamp * 1e-6) - current_time_;
//        current_time_ = static_cast<double>(frame.time_stamp * 1e-6);
//        DLOG(INFO) << "current time" << current_time_;
//    }

    return {color,
            clockwise_,
            time_gap_,
            current_time_,
            rtp_vec_,
            energy_center_r_,
            armor_center_p_,
            cv::Point2f(float(frame.image.cols >> 1), float(frame.image.rows >> 1))};
}

void RuneDetectorYolo::FindRotateDirection() {
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
