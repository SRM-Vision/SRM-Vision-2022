/**
 * Image provider video header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \warning NEVER include this file except in ./image_provider_video.cpp.
 */

#ifndef IMAGE_PROVIDER_VIDEO_H_
#define IMAGE_PROVIDER_VIDEO_H_

// Include nothing to avoid this file being wrongly included.

/**
 * \brief Video image provider class implementation.
 * \warning NEVER directly use this class to create image provider!  \n
 *   Instead, turn to ImageProviderFactory class and use CREATE_IMAGE_PROVIDER("IPVideo").
 */
class [[maybe_unused]] ImageProviderVideo final : public ImageProvider {
public:
    ImageProviderVideo() : ImageProvider(), time_stamp_(0) {}

    ~ImageProviderVideo() final;

    bool Initialize(const std::string &, bool record) final;

    inline bool GetFrame(Frame &frame) final {
        if (video_.read(frame.image)) {
            time_stamp_ += uint64_t(1e9 / video_.get(cv::CAP_PROP_FPS));
            frame.time_stamp = time_stamp_;
            return true;
        } else
            return false;
    }

private:
    cv::VideoCapture video_;  ///< Video object.

    uint64_t time_stamp_;

    /// Own registry for image provider video.
    [[maybe_unused]] static ImageProviderRegistry<ImageProviderVideo> registry_;
};

#endif  // IMAGE_PROVIDER_VIDEO_H_
