#include "detector-rune/detector_rune_debug.h"
#include <opencv2/imgproc.hpp>
#include "detector_rune.h"

[[maybe_unused]] RuneDetector::RuneDetector(Entity::Colors color, bool debug) :
        color_(color),
        debug_(debug),
        clockwise_(0),
        frame_lost_(0),
        rune_radius_(125),
        found_armor_center_p(false),
        found_energy_center_r(false),
        rtp_vec_(cv::Point2f(0, 0)),
        rtg_vec_(cv::Point2f(0, 0)),
        r_offset_(cv::Point2f(0, 0)),
        p_offset_(cv::Point2f(0, 0)),
        fan_center_g_(cv::Point2f(0, 0)),
        armor_center_p_(cv::Point2f(0, 0)),
        energy_center_r_(cv::Point2f(0, 0)),
        send_yaw_pitch_delay_(cv::Point3f(0, 0, 0)) {}

bool RuneDetector::Initialize(const std::string &config_path) {
    found_armor_center_p = found_energy_center_r = false;
    energy_center_r_ = fan_center_g_ = armor_center_p_ = cv::Point2f(0, 0);
    clockwise_ = frame_lost_ = 0;
    rune_radius_ = 125;

#if !NDEBUG
    RuneDetectorDebug::Instance().addTrackbar();
    debug_ = true;
#endif
    return true;
}

PowerRune RuneDetector::Run(Entity::Colors color, Frame &frame) {
    // FIXME The color from electronic control is unknown.
    color_ = color;
    color_ = Entity::kRed;

    if (clockwise_ && found_energy_center_r) {
        int length = int(rune_radius_ * 2);
        ROI_tl_point_.x = std::min(std::max(0, int(energy_center_r_.x) - length), frame.image.cols - 1);
        ROI_tl_point_.y = std::min(std::max(0, int(energy_center_r_.y) - length), frame.image.rows - 1);
        cv::Rect ROI_rect = cv::Rect(ROI_tl_point_.x, ROI_tl_point_.y, 2 * length, 2 * length) &
                            cv::Rect(0, 0, frame.image.cols, frame.image.rows);
        image_ = frame.image(ROI_rect);  // Use ROI

        energy_center_r_ -= cv::Point2f(ROI_tl_point_);  // Update last frame's points according to ROI size.
        armor_center_p_ -= cv::Point2f(ROI_tl_point_);
    } else {
        ROI_tl_point_ = cv::Point2i(0, 0);
        image_ = frame.image;
    }

    if (3 == image_.channels())
        PreProcess();

    if (debug_) {
        cv::imshow("Outline image", image_);  // Show outline image.
    }

    if (!clockwise_)
        FindRotateDirection();
    else
        FindArmorCenterP();

    return {color_,
            clockwise_,
            rtp_vec_,
            rtg_vec_,
            energy_center_r_,
            armor_center_p_,
            fan_center_g_,
            send_yaw_pitch_delay_,
            cv::Point2f(float(frame.image.cols >> 1), float(frame.image.rows >> 1))};
}

void RuneDetector::PreProcess() {
    try {
        cv::split(image_, image_channels_);
    } catch (...) {
        LOG(ERROR) << "Image is binary, Rune PreProcess only take images with three dimension.";
    }


    if (Entity::Colors::kRed == color_)
        cv::subtract(image_channels_.at(2), image_channels_.at(0), image_);     // Target is red energy.
    else if (Entity::Colors::kBlue == color_)
        cv::subtract(image_channels_.at(0), image_channels_.at(2), image_);     // Target is blue energy.
    else
        DLOG(ERROR) << "Input wrong color " << color_ << " of power rune.";

    cv::threshold(image_, image_, RuneDetectorDebug::Instance().SplitGrayThresh(), 255, cv::THRESH_BINARY);

    int structElementSize = 3;
    if (debug_) {
        structElementSize = RuneDetectorDebug::Instance().KernelSize();
        if (structElementSize == 0)
            return;
    }

    cv::Mat element_close = cv::getStructuringElement(cv::MORPH_RECT,
                                                       cv::Size(2 * structElementSize - 1, 2 * structElementSize - 1));
    cv::morphologyEx(image_, image_, cv::MORPH_CLOSE, element_close);
    cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT,
                                                       cv::Size(2 * structElementSize - 1, 2 * structElementSize - 1));
    cv::dilate(image_, image_, element_dilate);
}

bool RuneDetector::FindArmorCenterP() {
    // Find the blade contour in the image and establish the contour hierarchy.
    cv::findContours(image_, fan_contours_,
                     fan_hierarchies_,
                     cv::RETR_TREE,
                     cv::CHAIN_APPROX_SIMPLE);
    found_armor_center_p = false;
    found_energy_center_r = false;

    if (!fan_hierarchies_.empty()) {
        // Traverse the contour according to the horizontal relationship.
        for (int i = 0; i >= 0; i = fan_hierarchies_[i][0]) {
            // Find minimum enclosing rectangle for each selection.
            fan_encircle_rect_ = cv::minAreaRect(fan_contours_[i]);

            // Record fan's enclosing rectangular area.
            const double &bounding_box_area = fan_encircle_rect_.size.area();
            if (bounding_box_area < RuneDetectorDebug::Instance().MinBoundingBoxArea())
                continue;

            double fan_rect_wh_ratio = fan_encircle_rect_.size.width < fan_encircle_rect_.size.height ?
                                       static_cast<double>(fan_encircle_rect_.size.width) /
                                       static_cast<double>(fan_encircle_rect_.size.height) :
                                       static_cast<double>(fan_encircle_rect_.size.height) /
                                       static_cast<double>(fan_encircle_rect_.size.width);

            // Don't satisfy the aspect ratio.
            if (!(fan_rect_wh_ratio > RuneDetectorDebug::Instance().MinFanWHRatio() &&
                  fan_rect_wh_ratio < RuneDetectorDebug::Instance().MaxFanWHRatio()))
                continue;

            double fan_rect_area = cv::contourArea(fan_contours_[i]);
            if (fan_rect_area > RuneDetectorDebug::Instance().MinFanArea() &&
                fan_rect_area < RuneDetectorDebug::Instance().MaxFanArea()) {
                // Traverse sub contour.
                for (int first_child = fan_hierarchies_[i][2];
                     first_child >= 0; first_child = fan_hierarchies_[first_child][0]) {
                    armor_encircle_rect_ = cv::minAreaRect(fan_contours_[first_child]);
                    double armor_rect_area = armor_encircle_rect_.size.area();

                    double armor_rect_wh_ratio =
                            armor_encircle_rect_.size.width < armor_encircle_rect_.size.height ?
                            static_cast<double>(armor_encircle_rect_.size.width) /
                            static_cast<double>(armor_encircle_rect_.size.height) :
                            static_cast<double>(armor_encircle_rect_.size.height) /
                            static_cast<double>(armor_encircle_rect_.size.width);

                    if (armor_rect_area > RuneDetectorDebug::Instance().MinArmorArea() &&
                        armor_rect_area < RuneDetectorDebug::Instance().MaxArmorArea() &&
                        armor_rect_wh_ratio > RuneDetectorDebug::Instance().MinArmorWHRatio()
                        && armor_rect_wh_ratio < RuneDetectorDebug::Instance().MaxArmorWHRatio()) {
                        // Find P Point but it may be wrong
                        rtp_vec_ = armor_encircle_rect_.center - energy_center_r_;
                        float vec_length = std::sqrt(rtp_vec_.x * rtp_vec_.x + rtp_vec_.y * rtp_vec_.y);
                        if (clockwise_ && (vec_length > rune_radius_ * (1 + kMaxRatio) ||
                                           vec_length < rune_radius_ * (1 - kMaxRatio))) {
                            DLOG(WARNING) << "Wrong p point: " << armor_encircle_rect_.center;
                            armor_center_p_ += p_offset_;
                        } else {
                            p_offset_ = armor_encircle_rect_.center - armor_center_p_;
                            armor_center_p_ = armor_encircle_rect_.center;
                        }
                        armor_center_p_ += cv::Point2f(ROI_tl_point_);
                        found_armor_center_p = true;
                        // Have found armor center that meets requirements, exit the sub contour loop.
                        break;
                    }
                }
                // Have found armor center, exit loop.
                if (found_armor_center_p)
                    break;
            }
        }
        if (!found_armor_center_p) {
            if (frame_lost_ >= kMaxFrameLost) {  // Continuous frames lost.
                DLOG(WARNING) << "No P point found for a long time!!! ";
                armor_center_p_ = cv::Point2f(0, 0);
            } else {
                armor_center_p_ += p_offset_ + cv::Point2f(ROI_tl_point_);
                found_armor_center_p = true;
                ++frame_lost_;
                DLOG(WARNING) << "No P point found! " << armor_center_p_;
            }
        }

        if (found_armor_center_p && FindCenterR()) {
            if (debug_)
                debug::Painter::Instance()->DrawLine(armor_center_p_, energy_center_r_, cv::Scalar(0, 255, 255), 2);

            FindFanCenterG();
            return true;
        }
    }

    // Have not found R, P and G at the same time.
    DLOG(WARNING) << "Have not found R, P and G points at the same time!";
    return false;
}

bool RuneDetector::FindCenterR() {
    std::vector<cv::Point2f> possible_center_r;

    // Find possible center points by contour's area.
    for (auto &fan_contour: fan_contours_) {
        cv::RotatedRect encircle_r_rect = cv::minAreaRect(fan_contour);
        double encircle_rect_area = encircle_r_rect.size.area();
        if (encircle_rect_area > float(RuneDetectorDebug::Instance().MinRArea())
            && encircle_rect_area < float(RuneDetectorDebug::Instance().MaxRArea())
            && std::abs(encircle_r_rect.size.width - encircle_r_rect.size.height) <
               float(RuneDetectorDebug::Instance().MaxRWHDeviation())) {
            possible_center_r.emplace_back(encircle_r_rect.center);
        }
    }

    if (possible_center_r.empty()) {
        DLOG(WARNING) << "No possible center R points found.";
        return false;
    }

    // Extract the four vertices of the fan center-area's minimum enclosing rectangle.
    cv::Point2f fan_rect_points_[4];  ///< Four vertices of the enclosing rectangle of a fan
    fan_encircle_rect_.points(fan_rect_points_);
    cv::Point2f possible_ptr_vec = fan_encircle_rect_.center - armor_center_p_ + cv::Point2f(ROI_tl_point_);

    cv::Point2f direction_vec;  ///< Directional vector from fan-center to energy-center R
    auto dis_width = std::hypot(fan_rect_points_[0].x - fan_rect_points_[1].x,
                                fan_rect_points_[0].y - fan_rect_points_[1].y);
    auto dis_height = std::hypot(fan_rect_points_[1].x - fan_rect_points_[2].x,
                                 fan_rect_points_[1].y - fan_rect_points_[2].y);

    // Determine the direction of the center R relative to fan.
    if (dis_width > dis_height) {
        cv::Point2f vec = fan_rect_points_[0] - fan_rect_points_[1];
        if (possible_ptr_vec.dot(vec) > 0)
            direction_vec = vec;
        else
            direction_vec = -vec;
    } else {
        cv::Point2f vec = fan_rect_points_[1] - fan_rect_points_[2];
        if (possible_ptr_vec.dot(vec) > 0)
            direction_vec = vec;
        else
            direction_vec = -vec;
    }

    // Make fan_encircle_rect include R area
    cv::Point2f new_fan_rect_center = fan_encircle_rect_.center + (direction_vec * .75);
    cv::Size new_rect_size;
    fan_encircle_rect_.size.width > fan_encircle_rect_.size.height ?
    (new_rect_size = cv::Size(int(fan_encircle_rect_.size.width) >> 1,
                              int(fan_encircle_rect_.size.height))) :
    (new_rect_size = cv::Size(int(fan_encircle_rect_.size.width),
                              int(fan_encircle_rect_.size.height) >> 1));

    cv::RotatedRect new_R_encircle_rect = cv::RotatedRect(new_fan_rect_center, new_rect_size,
                                                          fan_encircle_rect_.angle);
    cv::Rect R_rect = new_R_encircle_rect.boundingRect();

    for (const auto &center_r: possible_center_r) {
        if (center_r.inside(R_rect)) {
            // Find R point but it may be wrong
            if (clockwise_ && (std::abs(center_r.x - energy_center_r_.x) > kMaxDeviation
                               || std::abs(center_r.y - energy_center_r_.y) > kMaxDeviation)) {
                DLOG(WARNING) << "Wrong R Point: " << center_r;
                cv::Point2f dir_vec = cv::Point2f(center_r.x - energy_center_r_.x, center_r.y - energy_center_r_.y);
                float vec_length = algorithm::SqrtFloat(dir_vec.x * dir_vec.x + dir_vec.y * dir_vec.y);
                // Max compensation on the direction vector
                r_offset_ = kMaxDeviation * cv::Point2f(dir_vec.x / vec_length, dir_vec.y / vec_length);
                energy_center_r_ += r_offset_;
            } else {
                r_offset_ = center_r - energy_center_r_;
                energy_center_r_ = center_r;
            }
            found_energy_center_r = true;
            frame_lost_ = 0;
            energy_center_r_ += cv::Point2f(ROI_tl_point_);
        }
    }

    if (found_energy_center_r) {
        return true;
    }
    energy_center_r_ += r_offset_ + cv::Point2f(ROI_tl_point_);
    if (frame_lost_ >= kMaxFrameLost) {
        DLOG(WARNING) << "No center R found.";
        return false;
    } else {
        ++frame_lost_;
        energy_center_r_ += r_offset_;
        found_energy_center_r = true;
    }
    return true;
}

void RuneDetector::FindFanCenterG() {
    // FIXME Need a better method for calculating.
    fan_center_g_ = 0.5 * (armor_encircle_rect_.center + cv::Point2f(ROI_tl_point_)) + 0.5 * energy_center_r_;
    rtp_vec_ = armor_center_p_ - energy_center_r_;
    rtg_vec_ = fan_center_g_ - energy_center_r_;
}

void RuneDetector::FindRotateDirection() {
    static std::vector<cv::Point2f> r_to_p_vec;  ///< Vector Used for deciding rotation direction.

    if (FindArmorCenterP() && clockwise_ == 0) {
        r_to_p_vec.emplace_back(rtp_vec_);
    }
    if (frame_lost_ >= kMaxFrameLost)
        r_to_p_vec.clear();  // Long time lost frame.

    if (clockwise_ == 0 && static_cast<int>(r_to_p_vec.size()) > 20) {
        cv::Point2f first_rotation = r_to_p_vec[5];  // The fist five frames may be invalid.
        float radius, final_radius = 0;
        for (auto current_rotation = r_to_p_vec.begin() + 6; current_rotation != r_to_p_vec.end(); ++current_rotation) {
            double cross = first_rotation.cross(cv::Point2f(current_rotation->x, current_rotation->y));
            radius = algorithm::SqrtFloat(
                    current_rotation->x * current_rotation->x + current_rotation->y * current_rotation->y);
            final_radius +=
                    std::min(rune_radius_ * (1 + kMaxRatio), std::max(rune_radius_ * (1 - kMaxRatio), radius)) / 15;


            DLOG(INFO) << cross << "  /t" << first_rotation << "  /t"
                       << cv::Point2f(current_rotation->x, current_rotation->y);

            if (cross > 0.0)
                ++clockwise_;
            else if (cross < 0.0)
                --clockwise_;
        }
        if (clockwise_ > 8) {
            DLOG(INFO) << "Power rune's direction is clockwise.";
            clockwise_ = 1;
        } else if (clockwise_ < -8) {
            DLOG(INFO) << "Power rune's direction is anti-clockwise.";
            clockwise_ = -1;
        } else {
            clockwise_ = 0;
            DLOG(WARNING) << "Rotating direction is not decided!";
            r_to_p_vec.clear();
        }
    }
}


