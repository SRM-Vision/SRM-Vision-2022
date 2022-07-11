#include "detector-rune-debug/detector_rune_debug.h"
#include <opencv2/imgproc.hpp>
#include "detector_rune.h"

[[maybe_unused]] RuneDetector::RuneDetector(Entity::Colors color, bool debug) :
        color_(color),
        debug_(debug),
        found_energy_center_r(false),
        found_armor_center_p(false),
        clockwise_(0),
        frame_lost_(0),
        rune_radius_(125),
        rtp_vec_(cv::Point2f(0, 0)),
        rtg_vec_(cv::Point2f(0, 0)),
        energy_center_r_(cv::Point2f(0, 0)),
        armor_center_p_(cv::Point2f(0, 0)),
        last_energy_center_r(cv::Point2f(0, 0)),
        r_offset_(cv::Point2f(0, 0)),
        p_offset_(cv::Point2f(0, 0)),
        fan_center_g_(cv::Point2f(0, 0)),
        send_yaw_pitch_delay_(cv::Point3f(0, 0, 0)) {}

bool RuneDetector::Initialize(const std::string &config_path, const Frame &frame, bool debug_use_trackbar) {
    // Initial center will be center of the image.
    energy_center_r_ = fan_center_g_ = armor_center_p_ = last_energy_center_r = cv::Point2f(0, 0);
    clockwise_ = frame_lost_ = 0;
    rune_radius_ = 125;

    RuneDetectorDebug::Instance().Initialize("../config/infantry/rune-detector-param.yaml", debug_use_trackbar);

    return true;
}

PowerRune RuneDetector::Run(Entity::Colors color, Frame &frame, const cv::MatSize &size) {
    color_ = color;
    color_ = Entity::kRed;
    image_ = frame.image.clone();

    // Split image's channels.
    if (3 == image_.channels())
        ImageSplit(image_);

    cv::threshold(image_, image_, RuneDetectorDebug::Instance().SplitGrayThresh(), 255, cv::THRESH_BINARY);
    ImageMorphologyEx(image_);

    if (debug_) {
        debug::Painter::Instance()->DrawText("G", fan_center_g_, cv::Scalar(125, 145, 0), 3);
        cv::imshow("Test1", image_);
    }

    if (0 == clockwise_)
        FindRotateDirection();
    else
        FindArmorCenterP(image_);

    debug::Painter::Instance()->DrawPoint(armor_center_p_, cv::Scalar(0, 255, 0), 2, 2);
    debug::Painter::Instance()->DrawText("P", armor_center_p_, cv::Scalar(0, 255, 0), 3);
    debug::Painter::Instance()->DrawText("R", energy_center_r_, cv::Scalar(255, 0, 255), 3);
    cv::waitKey(0);
    return {color_,
            clockwise_,
            rtp_vec_,
            rtg_vec_,
            energy_center_r_,
            armor_center_p_,
            fan_center_g_,
            send_yaw_pitch_delay_,
            cv::Point2f(float(size().width >> 1), float(size().height >> 1))};
}

void RuneDetector::ImageSplit(cv::Mat &image) {
    cv::split(image, image_channels_);

    if (Entity::Colors::kRed == color_)
        cv::subtract(image_channels_.at(2), image_channels_.at(0), image);     // Target is red energy
    else if (Entity::Colors::kBlue == color_)
        cv::subtract(image_channels_.at(0), image_channels_.at(2), image);     // Target is blue energy
    else
        LOG(ERROR) << "Input wrong color " << color_ << " of power rune.";
}

void RuneDetector::ImageMorphologyEx(cv::Mat &image) {
    int structElementSize = RuneDetectorDebug::Instance().KernelSize();
    if (structElementSize == 0)
        return;

    cv::Mat element_close = cv::getStructuringElement(cv::MORPH_RECT,
                                                      cv::Size(2 * structElementSize - 1, 2 * structElementSize - 1));
    cv::morphologyEx(image, image, cv::MORPH_CLOSE, element_close);
    cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT,
                                                       cv::Size(2 * structElementSize - 1, 2 * structElementSize - 1));
    cv::dilate(image, image, element_dilate);
}

bool RuneDetector::FindCenterR(cv::Mat &image) {
    std::vector<cv::Point2f> possible_center_r;
    found_energy_center_r = false;

    // Find possible center points by contour's area.
    for (auto &fan_contour: fan_contours_) {
        cv::RotatedRect encircle_r_rect = cv::minAreaRect(fan_contour);
        double encircle_rect_area = encircle_r_rect.size.area();
        if (encircle_rect_area > float(RuneDetectorDebug::Instance().MinRArea())
            && encircle_rect_area < float(RuneDetectorDebug::Instance().MaxRArea())
            && std::abs(encircle_r_rect.size.width - encircle_r_rect.size.height) <
               float(RuneDetectorDebug::Instance().MaxRWHDeviation()))
            possible_center_r.emplace_back(encircle_r_rect.center);
    }

    // Extract the four vertices of the fan page minimum enclosing rectangle.
    fan_encircle_rect_.points(fan_rect_points_);
    cv::Point2f possible_ptr_vec = fan_encircle_rect_.center - armor_center_p_;

    auto dis_width = std::hypot(fan_rect_points_[0].x - fan_rect_points_[1].x,
                                fan_rect_points_[0].y - fan_rect_points_[1].y);
    auto dis_height = std::hypot(fan_rect_points_[1].x - fan_rect_points_[2].x,
                                 fan_rect_points_[1].y - fan_rect_points_[2].y);

    cv::Point2f direction_vec;
    if (dis_width > dis_height) {
        cv::Point2f vec = fan_rect_points_[0] - fan_rect_points_[1];
        // Dot production determines the direction of rotation.
        if (possible_ptr_vec.dot(vec) > 0)
            direction_vec = vec;
        else
            direction_vec = -vec;

        if (debug_) {
            cv::line(image, fan_rect_points_[0], fan_rect_points_[1],
                     cv::Scalar_<double>(255, 0, 0), 3);
        }
    } else {
        cv::Point2f vec = fan_rect_points_[1] - fan_rect_points_[2];
        if (possible_ptr_vec.dot(vec) > 0)
            direction_vec = vec;
        else
            direction_vec = -vec;

        if (debug_) {
            cv::line(image, fan_rect_points_[2], fan_rect_points_[1],
                     cv::Scalar_<double>(255, 0, 0), 3);
        }
    }

    cv::Point2f new_fan_encircle_rect_center = fan_encircle_rect_.center + (direction_vec * .75);
    cv::Size new_rect_size;
    fan_encircle_rect_.size.width > fan_encircle_rect_.size.height ?
            new_rect_size = cv::Size(int(fan_encircle_rect_.size.width) >> 1,
                                     int(fan_encircle_rect_.size.height)) :
            new_rect_size = cv::Size(int(fan_encircle_rect_.size.width),
                                     int(fan_encircle_rect_.size.height) >> 1);
    cv::RotatedRect new_encircle_rect = cv::RotatedRect(new_fan_encircle_rect_center, new_rect_size,
                                                        fan_encircle_rect_.angle);
    cv::Point2f new_rect_points[4];
    new_encircle_rect.points(new_rect_points);

    cv::Rect R_rect = new_encircle_rect.boundingRect();
    if (possible_center_r.empty()) {
        LOG(WARNING) << "No possible center R points found.";
        return false;
    }

    for (const auto &center_r: possible_center_r) {
        if (center_r.inside(R_rect)) {
            // Find R point but it may be wrong
            if (clockwise_ &&
                (center_r.x - energy_center_r_.x > kMaxDeviation || center_r.y - energy_center_r_.y > kMaxDeviation)) {
                cv::Point2f dir_vec = cv::Point2f(center_r.x - energy_center_r_.x, center_r.y - energy_center_r_.y);
                float vec_length = std::sqrt(dir_vec.x * dir_vec.x + dir_vec.y * dir_vec.y);
                r_offset_ = kMaxDeviation * cv::Point2f(dir_vec.x / vec_length, dir_vec.y / vec_length);
                energy_center_r_ = energy_center_r_ / 2 + r_offset_;
            } else {
                r_offset_ = center_r - energy_center_r_;
                energy_center_r_ = center_r / 2 + energy_center_r_ / 2;
            }
            found_energy_center_r = true;
            frame_lost_ = 0;
            // Consider difference between fan center and point R.

            last_energy_center_r = energy_center_r_;
        }
    }

    if (found_energy_center_r)
        return true;

    if (frame_lost_ >= kMaxFrameLost) {
        LOG(WARNING) << "No center R found.";
        return false;
    } else {
        ++frame_lost_;
        energy_center_r_ += r_offset_;
        found_energy_center_r = true;
    }
    return true;
}

bool RuneDetector::FindArmorCenterP(cv::Mat &image) {
    // Find the blade contour in the image and establish the contour hierarchy.
    cv::findContours(image, fan_contours_,
                     fan_hierarchies_,
                     cv::RETR_TREE,
                     cv::CHAIN_APPROX_SIMPLE);

    found_armor_center_p = false;
    if (debug_) {
        armor_encircle_rect_ = cv::RotatedRect();
    }

    if (!fan_hierarchies_.empty()) {
        // Traverse the contour according to the horizontal relationship.
        for (int i = 0; i >= 0; i = fan_hierarchies_[i][0]) {
            // Find minimum enclosing rectangle for each selection.
            fan_encircle_rect_ = cv::minAreaRect(fan_contours_[i]);

            // Record enclosing rectangular area.
            const double &bounding_box_area = fan_encircle_rect_.size.area();
            if (bounding_box_area < RuneDetectorDebug::Instance().MinBoundingBoxArea())
                continue;

            double big_encircle_rect_wh_ratio = fan_encircle_rect_.size.width < fan_encircle_rect_.size.height ?
                                                static_cast<double>(fan_encircle_rect_.size.width) /
                                                static_cast<double>(fan_encircle_rect_.size.height) :
                                                static_cast<double>(fan_encircle_rect_.size.height) /
                                                static_cast<double>(fan_encircle_rect_.size.width);

            // It is not the fan because the profile does not satisfy the aspect ratio.
            if (!(big_encircle_rect_wh_ratio > RuneDetectorDebug::Instance().MinFanWHRatio() &&
                  big_encircle_rect_wh_ratio < RuneDetectorDebug::Instance().MaxFanWHRatio()))
                continue;

            double contour_area = cv::contourArea(fan_contours_[i]);  ///< Contour area.
            // Continue if contour area satisfies limits.
            if (contour_area > RuneDetectorDebug::Instance().MinFanArea() &&
                contour_area < RuneDetectorDebug::Instance().MaxFanArea()) {
                // DLOG(INFO) << "contour_area: " << contour_area;
                // Traverse the sub contour, in case not falling into small cavities.
                for (int next_son = fan_hierarchies_[i][2]; next_son >= 0; next_son = fan_hierarchies_[next_son][0]) {
                    armor_encircle_rect_ = cv::minAreaRect(fan_contours_[next_son]);
                    double armor_rect_area = armor_encircle_rect_.size.area();

                    double small_encircle_rect_wh_ratio =
                            armor_encircle_rect_.size.width < armor_encircle_rect_.size.height ?
                            static_cast<double>(armor_encircle_rect_.size.width) /
                            static_cast<double>(armor_encircle_rect_.size.height) :
                            static_cast<double>(armor_encircle_rect_.size.height) /
                            static_cast<double>(armor_encircle_rect_.size.width);

                    // Limit area and aspect ratio.
                    if (armor_rect_area > RuneDetectorDebug::Instance().MinArmorArea() &&
                        armor_rect_area < RuneDetectorDebug::Instance().MaxArmorArea() &&
                        small_encircle_rect_wh_ratio > RuneDetectorDebug::Instance().MinArmorWHRatio()
                        && small_encircle_rect_wh_ratio < RuneDetectorDebug::Instance().MaxArmorWHRatio()) {
                        if (debug_) {
                            debug::Painter::Instance()->DrawRotatedBox(armor_encircle_rect_, cv::Scalar(255, 0, 255),
                                                                       3);
                        }

                        rtp_vec_ = armor_encircle_rect_.center - energy_center_r_;
                        // Find P Point but it may be wrong
                        float vec_length = std::sqrt(rtp_vec_.x * rtp_vec_.x + rtp_vec_.y * rtp_vec_.y);
                        if (clockwise_ && (vec_length > rune_radius_ * (1 + kMaxRatio) ||
                                           vec_length < rune_radius_ * (1 - kMaxRatio))) {
                            DLOG(INFO) << "Wrong p point: " << armor_encircle_rect_.center;
                            armor_center_p_ += p_offset_;
                            frame_lost_++;
                        } else {
                            p_offset_ = armor_encircle_rect_.center - armor_center_p_;
                            armor_center_p_ = armor_encircle_rect_.center;
                        }
                        found_armor_center_p = true;
                        // Have found the fan blade that meets requirements, exit the son_contour loop
                        break;
                    }
                }
                // Have found armor center, exit loop.
                if (found_armor_center_p)
                    break;
            }
        }

        if (!found_armor_center_p) {
            if (frame_lost_ >= kMaxFrameLost) // Continuous frames lost
                LOG(ERROR) << "No P point found! ";
            else {
                armor_center_p_ += p_offset_;
                found_armor_center_p = true;
                frame_lost_++;
            }
        }
        // Do not change this order.
        if (found_armor_center_p && FindCenterR(image)) {
            if (debug_)
                debug::Painter::Instance()->DrawLine(armor_center_p_, energy_center_r_, cv::Scalar(0, 255, 255), 2);

            FindFanCenterG();
            return true;
        }
    }

    // Have not found R, P and G at the same time.
    LOG(WARNING) << "Have not found R, P and G points at the same time!";
    // Reset center R,P,G
    if (!found_energy_center_r)
        energy_center_r_ = last_energy_center_r;
    if (!found_armor_center_p)
        armor_center_p_ = cv::Point2f(0, 0);
    return false;
}

void RuneDetector::FindFanCenterG() {
    // Imaginary center of mass.
    fan_center_g_ = 0.5 * armor_encircle_rect_.center + 0.5 * energy_center_r_;
    rtg_vec_ = fan_center_g_ - energy_center_r_;
}

void RuneDetector::FindRotateDirection() {
    /// Vector from center R to armor center P used for deciding rotation direction.
    static std::vector<cv::Point2f> r_to_p_vec;

    // When armor centers are found but rotation direction is not decided, armor finding should be done first.
    if (FindArmorCenterP(image_) && clockwise_ == 0)
        r_to_p_vec.emplace_back(fan_center_g_ - energy_center_r_);
    if (frame_lost_ > kMaxFrameLost)
        r_to_p_vec.clear();  // Long time lost frame
    if (debug_)
        DLOG(INFO) << energy_center_r_ << "-------" << fan_center_g_;

    // No certain rotation direction lasting 10 frames or above.
    if (clockwise_ == 0 && static_cast<int>(r_to_p_vec.size()) > 20) {
        cv::Point2f first_rotation = r_to_p_vec[5];  // The fist five frames are invalid
        float radius, final_radius = 0;
        for (auto current_rotation = r_to_p_vec.begin() + 6; current_rotation != r_to_p_vec.end(); ++current_rotation) {
            double cross = first_rotation.cross(cv::Point2f(current_rotation->x, current_rotation->y));
            radius = std::sqrt(current_rotation->x * current_rotation->x + current_rotation->y * current_rotation->y);
            final_radius +=
                    std::min(rune_radius_ * (1 + kMaxRatio), std::max(rune_radius_ * (1 - kMaxRatio), radius)) / 15;
            if (debug_)
                DLOG(INFO) << cross << "****____****" << first_rotation << "****____****"
                           << cv::Point2f(current_rotation->x, current_rotation->y);
            if (cross > 0.0)
                ++clockwise_;
            else if (cross < 0.0)
                --clockwise_;
        }
        if (clockwise_ > 8) {
            LOG(INFO) << "Power rune's direction is clockwise.";
            clockwise_ = 1;
        } else if (clockwise_ < -8) {
            LOG(INFO) << "Power rune's direction is anti-clockwise.";
            clockwise_ = -1;
        } else {
            clockwise_ = 0;
            LOG(WARNING) << "Rotation direction is not decided!";
            r_to_p_vec.clear();
        }
    }
}
