//
// Created by screw on 2022/7/18.
//

#ifndef IMAGE_PROVIDER_CAMERA_RECORD_H_
#define IMAGE_PROVIDER_CAMERA_RECORD_H_

#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

void Record(cv::Mat image)
{
    static time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    static cv::VideoWriter video(
            "../../cache/"+std::string(ctime(&tt))+".mp4",
            cv::CAP_OPENCV_MJPEG, 25, image.size());
    video.write(image);
}

#endif //IMAGE_PROVIDER_CAMERA_RECORD_H_
