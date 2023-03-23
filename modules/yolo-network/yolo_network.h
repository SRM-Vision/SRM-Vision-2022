/*
    created by XY_cpp on 21.3.2023
    this class only support yolov7-keypoint model
    see https://github.com/SRM-Vision/yolov7-keypoint for more information
*/
#ifndef YOLO_NETWORK_H
#define YOLO_NETWORK_H
#include <vector>
#include <glog/logging.h>

#include <opencv2/opencv.hpp>

#include <NvInfer.h>
#include <logger.h>

using std::shared_ptr;
template<typename _T>
shared_ptr<_T> make_nvshared(_T *ptr) {
	return shared_ptr<_T>(ptr, [](_T* p){p->destroy();});
}

inline const char* severity_string(nvinfer1::ILogger::Severity t) 
{
	switch(t) 
    {
		case nvinfer1::ILogger::Severity::kINTERNAL_ERROR: return "internal_error";
		case nvinfer1::ILogger::Severity::kERROR: return "error";
		case nvinfer1::ILogger::Severity::kWARNING: return "warning";
		case nvinfer1::ILogger::Severity::kINFO: return "info";
		case nvinfer1::ILogger::Severity::kVERBOSE: return "verbose";
		default: return "unknown";
	}
}

class TRTLogger : public nvinfer1::ILogger 
{
public:
	virtual void log(Severity severity, nvinfer1::AsciiChar const* msg) noexcept override 
    {
        if(severity <= Severity::kERROR) LOG(ERROR) << severity_string(severity) << " " << msg;
        else if(severity <= Severity::kWARNING) LOG(WARNING) << severity_string(severity) << " " << msg;
        else if(severity <= Severity::kINFO) LOG(INFO) << severity_string(severity) << " " << msg;
	}
};

struct Objects
{
    float x1,y1;//(x1, y1): top-left corner
    float x2,y2;//(x2, y2): bottom-right corner
    float prob;
    int cls;
    std::vector<cv::Point2f> pts;
};

class YoloNetwork
{
public:
    const int INPUT_W;  
    const int INPUT_H;
    const int NUM_CLASSES;
    const int NUM_POINTS;

    const float BOX_CONF_THRESH;
    const int MAX_NMS;
    const float IOU_THRESH;

public:
    void Initialize(const std::string &);
    std::vector<Objects> Inference(cv::Mat);
    YoloNetwork(
        int input_w, 
        int input_h,
        int num_classes,
        int num_points,

        float box_conf_thresh = 0.55,
        int max_nms = 3000, 
        float iou_thresh = 0.25
    ):
        INPUT_W(input_w) ,
        INPUT_H(input_h),
        NUM_CLASSES(num_classes),
        NUM_POINTS(num_points),

        BOX_CONF_THRESH(box_conf_thresh),
        MAX_NMS(max_nms),
        IOU_THRESH(iou_thresh)
    {}

    ~YoloNetwork()
    {
	    delete execution_context;
	    delete engine;
	    cudaFreeHost(input_data_host);
   	    cudaFree(input_data_device);
   	    cudaFreeHost(output_data_host);
   	    cudaFree(output_data_device); 
    }
    
protected:
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* execution_context = nullptr;
    cudaStream_t stream = nullptr;
	TRTLogger logger;

    int input_numel = 0;
	float* input_data_host = nullptr;
	float* input_data_device = nullptr; 

    int output_numel = 0;
    float* output_data_host = nullptr;
	float* output_data_device = nullptr;

protected:
    // virtual void InitialParameter()=0;

    ///Initialize
    void BuildEngineFromONNX(const std::string &);
    void BuildEngineFromCache(const std::string &);
    void CacheEngine(const std::string &);    

    //Inference
    void LetterBox(cv::Mat&, float&, float&, float&);
    void GetObjects(std::vector<Objects>&);
    void nms(std::vector<Objects>&);
};
#endif