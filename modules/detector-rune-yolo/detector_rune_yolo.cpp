#include "detector_rune_yolo.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

RuneDetectorYolo::RuneDetectorYolo()
{
    _Network = YoloNetwork("yolo-rune.onnx", 2, 5);
    
}

RuneDetectorYolo::~RuneDetectorYolo()
{
    
}

PowerRune RuneDetectorYolo::Inference(cv::Mat image)
{
    auto result = _Network.Inference()
    PowerRune rune;
    rune.rx = result[0].pts[3].x;
    rune.ry = result[0].pts[3].y;
    // calculate center point
    rune.x = (result[0].pts[0].x + result[0].pts[1].x + result[0].pts[2].x + result[0].pts[4].x) / 4;
    rune.y = (result[0].pts[0].y + result[0].pts[1].y + result[0].pts[2].y + result[0].pts[4].y) / 4;
    // calculate width and height
    rune.w = std::max({ result[0].pts[0].x, result[0].pts[1].x, result[0].pts[2].x, result[0].pts[4].x }) -
        std::min({ result[0].pts[0].x, result[0].pts[1].x, result[0].pts[2].x, result[0].pts[4].x });
    rune.h = std::max({ result[0].pts[0].y, result[0].pts[1].y, result[0].pts[2].y, result[0].pts[4].y }) -
        std::min({ result[0].pts[0].y, result[0].pts[1].y, result[0].pts[2].y, result[0].pts[4].y });
    return rune;
}