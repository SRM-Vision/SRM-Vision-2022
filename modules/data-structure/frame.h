/**
 * Frame data structure header.
 * \author trantuan-20048607
 * \date 2022.1.28
 */

#ifndef FRAME_H_
#define FRAME_H_

#include <opencv2/core/core.hpp>

/**
 * \brief Single frame structure.
 * \details 2 ways of initializing method provided:  \n
 *   (Default) Directly use Frame() to initialize an empty and useless frame.  \n
 *   (Manual) Use Frame(_image, _time_stamp) to initialize a complete frame.
 */
struct Frame {
    cv::Mat image;        ///< OpenCV style image matrix.
    uint64_t time_stamp;  ///< Time stamp in DEC nanoseconds.

    Frame(cv::Mat &_image,
          uint64_t _time_stamp) :
            image(_image.clone()),
            time_stamp(_time_stamp) {}

    Frame() : time_stamp(0) {}
};

#endif  // FRAME_H_
