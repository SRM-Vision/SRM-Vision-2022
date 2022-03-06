/**
 * Power rune detector class header.
 * \author screw-44, LIYunzhe1408, LemonadeJJ
 * \date 2022.2.16
 */

#ifndef PAINTER_H_
#define PAINTER_H_

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace debug {
    class Painter {
    public:
        Painter() = default;

        inline void UpdateImage(const cv::Mat &image) {
            image_ = image.clone();
        }

        inline void ShowImage(const std::string &window_name,
                              int wait_time = 1) {
            cv::imshow(window_name, image_);
            if ((cv::waitKey(1) & 0xff) == 'q')
                cv::waitKey(wait_time);
            else if ((cv::waitKey(1) & 0xff) == 's')
                RuneDetectorDebug::Instance().Save();
        }

        inline void DrawBoundingBox(const cv::RotatedRect &rect,
                                    const cv::Scalar &color,
                                    int thickness) {
            cv::Point2f rect_point[4];
            rect.points(rect_point);
            for (auto i = 0; i < 4; ++i)
                cv::line(image_, rect_point[i], rect_point[(i + 1) % 4], color, thickness);
        }

        inline void DrawRotatedRectangle(const cv::Point2f &left_top,
                                         const cv::Point2f &right_top,
                                         const cv::Point2f &left_bottom,
                                         const cv::Point2f &right_bottom,
                                         const cv::Scalar &color,
                                         int thickness) {
            cv::line(image_, left_top, right_top, color, thickness);
            cv::line(image_, right_top, right_bottom, color, thickness);
            cv::line(image_, right_bottom, left_bottom, color, thickness);
            cv::line(image_, left_bottom, left_top, color, thickness);
        }

        inline void DrawPoint(const cv::Point2f &center,
                              const cv::Scalar &color,
                              int radius = 3,
                              int thickness = 3) {
            cv::circle(image_, center, int(radius), color, thickness);
        }

        inline void DrawLine(const cv::Point2f &initial_point,
                             const cv::Point2f &final_point,
                             const cv::Scalar &color,
                             int thickness = 3) {
            cv::line(image_, initial_point, final_point, color, thickness);
        }

        inline void DrawContours(const std::vector<std::vector<cv::Point>> &contours,
                                 const cv::Scalar &color,
                                 int thickness = 3,
                                 int contourIdx = -1,
                                 int lineType = 8) {
            cv::drawContours(image_, contours, contourIdx, color, thickness, lineType);
        }

        inline void DrawText(const std::string &text,
                             const cv::Point2f &point,
                             const cv::Scalar &color,
                             int thickness) {
            cv::putText(image_, text, point, cv::FONT_HERSHEY_SIMPLEX, 1, color, thickness, 8, false);
        }

        inline static Painter &Instance() {
            static Painter _;
            return _;
        }

    private:
        cv::Mat image_;
    };
}

#endif  // PAINTER_H_
