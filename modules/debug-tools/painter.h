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
    class IPainter {
    public:
        IPainter() = default;

        virtual ~IPainter() = default;

        virtual void UpdateImage(const cv::Mat &image) = 0;

        virtual void ShowImage(const std::string &window_names, int wait_time) = 0;

        virtual void DrawRotatedBox(const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) = 0;

        virtual void DrawBoundingBox(const cv::Rect &rect, const cv::Scalar &color, int thickness) = 0;

        virtual void DrawRotatedRectangle(const cv::Point2f &left_top,
                                          const cv::Point2f &right_top,
                                          const cv::Point2f &right_bottom,
                                          const cv::Point2f &left_bottom,
                                          const cv::Scalar &color,
                                          int thickness) = 0;

        virtual void DrawPoint(const cv::Point2f &center,
                               const cv::Scalar &color,
                               int radius,
                               int thickness) = 0;

        virtual void DrawLine(const cv::Point2f &initial_point,
                              const cv::Point2f &final_point,
                              const cv::Scalar &color,
                              int thickness) = 0;

        virtual void DrawContours(const std::vector<std::vector<cv::Point>> &contours,
                                  const cv::Scalar &color,
                                  int contourIdx,
                                  int thickness,
                                  int lineType) = 0;

        virtual void DrawText(const std::string &text,
                              const cv::Point2f &point,
                              const cv::Scalar &color,
                              int thickness) = 0;
    };


    class Painter : IPainter {
    public:
        Painter() = default;

        inline void UpdateImage(const cv::Mat &image) final {
            image_ = image.clone();
        }

        inline void ShowImage(const std::string &window_name,
                              int wait_time = 1) final {
            cv::imshow(window_name, image_);
            auto key = cv::waitKey(1) & 0xff;

            if (key == 'q')
                cv::waitKey(wait_time);
//            else if (key == 's')
//                OLDRuneDetectorDebug::Instance().Save();
        }

        inline void DrawRotatedBox(const cv::RotatedRect &rect,
                                   const cv::Scalar &color,
                                   int thickness) final {
            cv::Point2f rect_point[4];
            rect.points(rect_point);
            for (auto i = 0; i < 4; ++i)
                cv::line(image_, rect_point[i], rect_point[(i + 1) % 4], color, thickness);
        }

        inline void DrawBoundingBox(const cv::Rect &rect,
                                   const cv::Scalar &color,
                                   int thickness) final {
            cv::rectangle(image_,rect.tl(),rect.br(),color,thickness);
        }

        inline void DrawRotatedRectangle(const cv::Point2f &left_top,
                                         const cv::Point2f &right_top,
                                         const cv::Point2f &right_bottom,
                                         const cv::Point2f &left_bottom,
                                         const cv::Scalar &color,
                                         int thickness) final {
            cv::line(image_, left_top, right_top, color, thickness);
            cv::line(image_, right_top, right_bottom, color, thickness);
            cv::line(image_, right_bottom, left_bottom, color, thickness);
            cv::line(image_, left_bottom, left_top, color, thickness);
        }

        inline void DrawPoint(const cv::Point2f &center,
                              const cv::Scalar &color,
                              int radius = 3,
                              int thickness = 3) final {
            cv::circle(image_, center, int(radius), color, thickness);
        }

        inline void DrawLine(const cv::Point2f &initial_point,
                             const cv::Point2f &final_point,
                             const cv::Scalar &color,
                             int thickness = 3) final {
            cv::line(image_, initial_point, final_point, color, thickness);
        }

        inline void DrawContours(const std::vector<std::vector<cv::Point>> &contours,
                                 const cv::Scalar &color,
                                 int contourIdx = -1,
                                 int thickness = 3,
                                 int lineType = 8) final {
            cv::drawContours(image_, contours, contourIdx, color, thickness, lineType);
        }

        inline void DrawText(const std::string &text,
                             const cv::Point2f &point,
                             const cv::Scalar &color,
                             int thickness) final {
            cv::putText(image_, text, point, cv::FONT_HERSHEY_SIMPLEX, 1, color, thickness, 8, false);
        }

        inline static IPainter* Instance() {
            static Painter _;
            return &_;
        }

    private:
        cv::Mat image_;
    };

    class NoPainter : IPainter {
    public:
        void UpdateImage(const cv::Mat &image) final {};

        void ShowImage(const std::string &window_names, int wait_time = 1) final {};

        void DrawRotatedBox(const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) final {};

        void DrawBoundingBox(const cv::Rect &rect, const cv::Scalar &color, int thickness) final {};

        void DrawRotatedRectangle(const cv::Point2f &left_top,
                                  const cv::Point2f &right_top,
                                  const cv::Point2f &right_bottom,
                                  const cv::Point2f &left_bottom,
                                  const cv::Scalar &color,
                                  int thickness) final {};

        void DrawPoint(const cv::Point2f &center,
                       const cv::Scalar &color,
                       int radius = 3,
                       int thickness = 3) final {};

        void DrawLine(const cv::Point2f &initial_point,
                      const cv::Point2f &final_point,
                      const cv::Scalar &color,
                      int thickness = 3) final {};

        void DrawContours(const std::vector<std::vector<cv::Point>> &contours,
                          const cv::Scalar &color,
                          int contourIdx = -1,
                          int thickness = 3,
                          int lineType = 8) final {};

        void DrawText(const std::string &text,
                      const cv::Point2f &point,
                      const cv::Scalar &color,
                      int thickness) final {};

        inline static IPainter* Instance() {
            static NoPainter _;
            return &_;
        };
    };

}
#endif  // PAINTER_H_
