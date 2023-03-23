#include "yolo_network.h"

#include <vector>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <glog/logging.h>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <NvInferPlugin.h>

#include <cuda_runtime.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#define checkRuntime(op) __check_cuda_runtime((op), #op, __FILE__, __LINE__)
inline bool __check_cuda_runtime(cudaError_t code, const char* op, const char* file, int line)
{
	if (code != cudaSuccess) 
	{
		const char* err_name = cudaGetErrorName(code);
		const char* err_message = cudaGetErrorString(code);
		LOG(ERROR) << "runtime eror " << file << ": " << line << " " << op << " failed.\n code = " << err_name << ", message = " << err_message;
		return false;
	}
	return true;
} 

static inline int argmax(const float *ptr, int len)
{
	int max_arg = 0;
	for (int i = 1; i < len; i++) {
		if (ptr[i] > ptr[max_arg]) max_arg = i;
	}
	return max_arg;
}

void YoloNetwork::BuildEngineFromONNX(const std::string &onnx_file)
{
	LOG(INFO) << "Engine will be built from cache.";
	auto builder = make_nvshared(nvinfer1::createInferBuilder(logger));
	auto config = make_nvshared(builder->createBuilderConfig());
	auto network = make_nvshared(builder->createNetworkV2(1));
	auto parser = make_nvshared(nvonnxparser::createParser(*network, logger));

	if (!parser->parseFromFile(onnx_file.c_str(), 1))
		LOG(ERROR) << "Failed to parse " << onnx_file;

	auto profile = builder->createOptimizationProfile();

	// auto input_tensor = network->getInput(0);
	// auto input_dims = input_tensor->getDimensions();
	// input_dims.d[0] = 1;
	// profile->setDimensions(input_tensor->getName(), nvinfer1::OptProfileSelector::kMIN, input_dims);
	// profile->setDimensions(input_tensor->getName(), nvinfer1::OptProfileSelector::kOPT, input_dims);
	// profile->setDimensions(input_tensor->getName(), nvinfer1::OptProfileSelector::kMAX, input_dims);

	config->addOptimizationProfile(profile);

	if(builder->platformHasFastFp16())
	{
		LOG(INFO) << "Platform supports fp16, fp16 is enabled.";
		config->setFlag(nvinfer1::BuilderFlag::kFP16);
		// config->setFlag(nvinfer1::BuilderFlag::kSTRICT_TYPES);
		// auto formats = 1U << int(nvinfer1::TensorFormat::kHWC8);
		// network->getInput(0)->setAllowedFormats(formats);
		// network->getInput(0)->setType(nvinfer1::DataType::kHALF);
	}
	else
	{
		LOG(INFO) << "Platform does not support fp16, enable fp32 instead.";
		config->setFlag(nvinfer1::BuilderFlag::kTF32);
	}

	size_t free, total;
	cudaMemGetInfo(&free, &total);
	LOG(INFO) << "GPU memory total: " << (total >> 20) << "MB, free: " << (free >> 20) << "MB.";
	LOG(INFO) << "Max workspace size will use all of free GPU memory.";
	config->setMaxWorkspaceSize(free >> 1);

	engine = builder->buildEngineWithConfig(*network, *config);
}

void YoloNetwork::CacheEngine(const std::string &cache_file)    
{
	auto model_data = make_nvshared(engine->serialize());
	FILE* f = fopen(cache_file.c_str(), "wb");
	fwrite(model_data->data(), 1, model_data->size(), f);
	fclose(f);
}

void YoloNetwork::BuildEngineFromCache(const std::string &cache_file)
{
	LOG(INFO) << "Engine will be built from Cache.";
	auto load_file = [&] (const std::string &file)->std::vector<unsigned char>
	{
		std::ifstream in(file, std::ios::in | std::ios::binary);
		if (!in.is_open()) return {};

		in.seekg(0, std::ios::end);
		size_t length = in.tellg();

		std::vector<uint8_t> data;
		if (length > 0)
		{
			in.seekg(0, std::ios::beg);
			data.resize(length);

			in.read((char*)&data[0], length);
		}
		in.close();
		return data;
	};
	auto engine_data = load_file(cache_file.c_str());
	auto runtime = make_nvshared(nvinfer1::createInferRuntime(logger));
	engine = runtime->deserializeCudaEngine(engine_data.data(), engine_data.size());
}

void YoloNetwork::Initialize(const std::string &onnx_file)
{
	std::filesystem::path onnx_file_path(onnx_file);
	auto cache_file_path = onnx_file_path;
	cache_file_path.replace_extension("cache");

	initLibNvInferPlugins(&logger,"");
	if(!std::filesystem::exists(cache_file_path))
	{
		BuildEngineFromONNX(onnx_file);
		CacheEngine(cache_file_path.c_str());
	} 
	else BuildEngineFromCache(cache_file_path.c_str());

	if(engine == nullptr) LOG(ERROR) << "Build engine failed." ;
	else LOG(INFO) << "Build done.";

	if (engine->getNbBindings() != 2)
	{
		LOG(ERROR) << "Must be single input, single Output. Please Modify the onnx file.";
		return ;
	}

	checkRuntime(cudaStreamCreate(&stream));
	execution_context = engine->createExecutionContext();

	input_numel = 3 * INPUT_H * INPUT_W;
	checkRuntime(cudaMallocHost(&input_data_host, input_numel * sizeof(float)));
	checkRuntime(cudaMalloc(&input_data_device, input_numel * sizeof(float)));

	output_numel = 3 * (80 * 80 + 40 * 40 + 20 * 20) * (5 + NUM_CLASSES + 3 * NUM_POINTS);
	checkRuntime(cudaMallocHost(&output_data_host, output_numel * sizeof(float)));
	checkRuntime(cudaMalloc(&output_data_device, output_numel * sizeof(float)));
}

std::vector<Objects> YoloNetwork::Inference(cv::Mat image)
{
	//process image and move it into input_data_host
	float ro, dw, dh;//ratio, delta weight, delta height
	LetterBox(image, ro, dw, dh);

	image.convertTo(image, CV_32F);
	image /= 255.f;

	cv::Mat image_splits[3];
    cv::split(image, image_splits);
	std::swap(image_splits[0], image_splits[2]);

	for(auto & image_split : image_splits)
    {
		memcpy(input_data_host, image_split.data, INPUT_W * INPUT_H * sizeof(float));
        input_data_host += INPUT_W * INPUT_H;
    }
    input_data_host -= input_numel;

	//Inference
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	checkRuntime(cudaMemcpyAsync(input_data_device, input_data_host, input_numel * sizeof(float), cudaMemcpyHostToDevice, stream));
	float* bindings[] = {input_data_device, output_data_device};
	bool success = execution_context->enqueueV2((void**)bindings, stream, nullptr);
	checkRuntime(cudaMemcpyAsync(output_data_host, output_data_device, output_numel * sizeof(float), cudaMemcpyDeviceToHost, stream));
	checkRuntime(cudaStreamSynchronize(stream));
	std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    auto dur = end - start;
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(dur);
    DLOG(INFO) << "rune model detection time cost:" << time.count();

	// process data
	std::vector<Objects>objs;
	GetObjects(objs);
	nms(objs);

	for(auto &[x1, y1, x2, y2, prob, cls, apex] : objs)
	{
		x1 -= dw, x2 -= dw, y1 -= dh, y2 -= dh;
		x1 /= ro, x2 /= ro, y1 /= ro, y2 /= ro;

		for(auto &[x, y] : apex)
		{
			x -= dw, y -= dh;
			x /= ro, y /= ro;
		}
	}

	return objs;	
}

void YoloNetwork::LetterBox(cv::Mat& image, float &ro, float &dw, float &dh)
{
	cv::Size shape = image.size();
	cv::Size new_shape = {INPUT_W,INPUT_W};
    ro = std::min( new_shape.width / (float)shape.width, new_shape.height / (float)shape.height);

    // Compute padding
    cv::Size new_unpad = {(int)round(shape.width * ro), (int)round(shape.height * ro)};
    dw = new_shape.width - new_unpad.width, dh = new_shape.height - new_unpad.height;  // wh padding

	// divide padding into 2 sides
    dw /= 2.0, dh /= 2.0;

	if (shape != new_unpad)   // resize
        cv::resize(image, image, new_unpad, 0, 0, cv::INTER_LINEAR);
	
	int top = round(dh - 0.1), bottom = round(dh + 0.1);
    int left = round(dw - 0.1), right = round(dw + 0.1);
    cv::copyMakeBorder(image, image, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114});  // add border
}

void YoloNetwork::GetObjects(std::vector<Objects>& objs)
{
	const int& n = output_numel;
	const int m = (5 + NUM_CLASSES + 3 * NUM_POINTS);
	
	for(int i = 0; i < n; i += m)
	{
		Objects obj;	
		const float* data = output_data_host + i;

		float box_conf = data[4];
		if(box_conf < BOX_CONF_THRESH)
			continue;

		int box_cls = argmax(data + 5, NUM_CLASSES);
		float cls_conf = data[5 + box_cls];

		obj.cls = box_cls;
		obj.prob = cls_conf * box_conf;

		float x = data[0];
		float y = data[1];
		float w = data[2];
		float h = data[3];

		obj.x1 = x - w/2;
		obj.y1 = y - h/2;
		obj.x2 = x + w/2;
		obj.y2 = y + h/2;
		
		for(int j = 5 + NUM_CLASSES; j < m; j += 3)
			obj.pts.push_back({data[j], data[j + 1]});
		
		objs.push_back(obj);
	}
}

void YoloNetwork::nms(std::vector<Objects> &objs)
{
	std::sort(objs.begin(), objs.end(), [](Objects &a, Objects &b){return a.prob > b.prob;});
	if(objs.size() > MAX_NMS) 
		objs.resize(MAX_NMS);
	std::vector<float>vArea(objs.size());
	for(size_t i = 0; i < objs.size(); i++)
	{
		vArea[i] = (objs[i].x2 - objs[i].x1 + 1)
			* (objs[i].y2 - objs[i].y1 + 1);
	}
	for(size_t i = 0; i < objs.size(); i++)
	{
		for(size_t j = i + 1; j < objs.size();)
		{
			float xx1 = std::max(objs[i].x1, objs[j].x1);
			float yy1 = std::max(objs[i].y1, objs[j].y1);
			float xx2 = std::min(objs[i].x2, objs[j].x2);
			float yy2 = std::min(objs[i].y2, objs[j].y2);
			float w = std::max(float(0), xx2 - xx1 + 1);
			float h = std::max(float(0), yy2 - yy1 + 1);
			float inter = w * h;
			float ovr = inter / (vArea[i] + vArea[j] - inter);
			if (ovr >= IOU_THRESH)
			{
				objs.erase(objs.begin() + j);
				vArea.erase(vArea.begin() + j);
			}
			else j++;
		}
	}
}
