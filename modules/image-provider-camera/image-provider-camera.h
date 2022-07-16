/**
 * Image provider camera header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \warning NEVER include this file except in ./image_provider_camera.cpp.
 */

#ifndef IMAGE_PROVIDER_CAMERA_H_
#define IMAGE_PROVIDER_CAMERA_H_

// Include nothing to avoid this file being wrongly included.

/**
 * \brief Camera image provider class implementation.
 * \warning NEVER directly use this class to create image provider!  \n
 *   Instead, turn to ImageProviderFactory class and use CREATE_IMAGE_PROVIDER("IPCamera").
 */
class [[maybe_unused]] ImageProviderCamera final : public ImageProvider {
public:
    ImageProviderCamera() : camera_(nullptr), rec_(nullptr) {}

    ~ImageProviderCamera() final;

    bool Initialize(const std::string &, bool record) final;

    inline bool GetFrame(Frame &frame) final {
        return camera_->GetFrame(frame);
    }

    ATTR_READER(camera_, GetCameraHandle)

private:
    Camera *camera_;  ///< Camera pointer.

    cv::VideoWriter *rec_;  ///< Video writer pointer.

    /// Own registry for image provider camera.
    [[maybe_unused]] static ImageProviderRegistry<ImageProviderCamera> registry_;
};

#endif  // IMAGE_PROVIDER_CAMERA_H_
