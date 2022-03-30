/**
 * Power rune detector class header.
 * \author screw-44, LIYunzhe1408, LemonadeJJ
 * \date 2022.2.16
 */

#ifndef TRACKBAR_H_
#define TRACKBAR_H_

#include <string>
#include <opencv2/highgui.hpp>
#include <unordered_map>

namespace debug {
    template<typename T>
    class Trackbar {
    public:
        Trackbar() = default;

        /**
         * \brief Add a track bar to debug window.
         * \param trackbar_name Trackbar name.
         * \param window_name Window name.
         * \param [out] var Current var value.
         * \param var_max Max var value.
         */
        inline void AddTrackbar(const std::string &trackbar_name,
                                const std::string &window_name,
                                T &var,
                                const int &var_max) {
            if (trackbar_window_names_.end() ==
                find(trackbar_window_names_.begin(), trackbar_window_names_.end(), window_name)) {
                cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
                cv::Mat model(50, 1000, CV_8UC1, cv::Scalar(128));
                cv::imshow(window_name, model);
                trackbar_window_names_.emplace_back(window_name);
            }

            trackbar_values_.emplace(trackbar_name, var);

            /** TODO trackbar warning is a new bug from OpenCV 4.5.3.
             * If wanna eliminate it, just give up use pointer to adjust parameter
             */
            cv::createTrackbar(trackbar_name,
                               window_name,
                               &trackbar_values_.at(trackbar_name),
                               var_max);
        }

        inline static Trackbar &Instance() {
            static Trackbar _;
            return _;
        }

    private:
        std::unordered_map<std::string, T &> trackbar_values_;
        std::vector<std::string> trackbar_window_names_;
    };

    template<>
    class Trackbar<double> {
    public:
        inline void AddTrackbar(const std::string &trackbar_name,
                                const std::string &window_name,
                                double &var_value,
                                const double &max_value) {
            if (trackbar_window_names_.end() ==
                find(trackbar_window_names_.begin(), trackbar_window_names_.end(), window_name)) {
                cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
                cv::Mat model(50, 1000, CV_8UC1, cv::Scalar(128));
                cv::imshow(window_name, model);
                trackbar_window_names_.emplace_back(window_name);
            }

            parameter_values_.emplace(trackbar_name, var_value);
            trackbar_values_.emplace(trackbar_name, int(var_value * 1000));

            cv::createTrackbar(trackbar_name,
                               window_name,
                               &trackbar_values_.at(trackbar_name),
                               int(max_value * 1000), Callback);
        }

        static void Callback(int, void *) {
            for (auto &trackbar_value: trackbar_values_) {
                parameter_values_.at(trackbar_value.first) = double(trackbar_value.second) / 1000;
            }
        }

        inline static Trackbar &Instance() {
            static Trackbar _;
            return _;
        }

    private:
        static std::unordered_map<std::string, double &> parameter_values_;
        static std::unordered_map<std::string, int> trackbar_values_;
        std::vector<std::string> trackbar_window_names_;
    };
}

#endif  // TRACKBAR_H_
