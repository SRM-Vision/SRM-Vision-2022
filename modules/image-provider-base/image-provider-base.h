/**
 * Image provider base class header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \details Include this file only to declare or use pointers of image provider.
 */

#ifndef IMAGE_PROVIDER_BASE_H_
#define IMAGE_PROVIDER_BASE_H_

#include "lang-feature-extension/attr-reader.h"
#include "lang-feature-extension/disable-constructors.h"
#include "data-structure/frame.h"

/**
 * \brief Image provider base class.
 * \note You cannot directly construct objects.  \n
 *   Instead, find camera types in subclass documents,
 *   include image_provider_factory.h and use CREATE_IMAGE_PROVIDER macro.
 */
class ImageProvider : NO_COPY, NO_MOVE {
public:
    ATTR_READER_REF(intrinsic_matrix_, IntrinsicMatrix)

    ATTR_READER_REF(distortion_matrix_, DistortionMatrix)

    ImageProvider() = default;

    /// \details All workings of release will be done here.
    virtual ~ImageProvider() = default;

    /**
     * \brief Initialize by specified configuration file.
     * \param [in] file_path Configuration file path.
     * \param record Record video and save to ../cache.
     * \return Whether initialization succeeded.
     */
    virtual bool Initialize(const std::string &file_path, bool record = false) = 0;

    /**
     * \brief Get a frame.
     * \param [out] frame OpenCV image reference.
     * \return Whether frame is complete.
     */
    virtual bool GetFrame(Frame &frame) = 0;

    virtual bool IsConnected() = 0;

protected:
    cv::Mat intrinsic_matrix_;   ///< Intrinsic matrix for solving PnP.
    cv::Mat distortion_matrix_;  ///< Distortion matrix for solving PnP.
};

#endif  // IMAGE_PROVIDER_BASE_H_
