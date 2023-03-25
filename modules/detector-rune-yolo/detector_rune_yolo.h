/*
    Create by wangyw15 on 25.03.2023
    Detect rune through yolo network
    Uses yolo-network moudle
*/
#ifndef DETECTOR_RUNE_YOLO_H_
#define DETECTOR_RUNE_YOLO_H_

#include "../yolo-network/yolo_network.h"

#include <string>

struct PowerRune
{
    // The big R
    float rx, ry;
    // The center of the object
    float x, y;
    // The size of the object
    float h, w;
};

class RuneDetectorYolo
{
protected:
    YoloNetwork _Network;

public:
    RuneDetectorYolo();
    virtual ~RuneDetectorYolo();
    PowerRune Inference();
};

#endif
