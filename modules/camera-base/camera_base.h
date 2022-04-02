/**
 * Camera base class header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \details Include this file only to declare or use pointers of camera.
 */

#ifndef CAMERA_BASE_H_
#define CAMERA_BASE_H_

#include "data-structure/frame.h"
#include "data-structure/buffer.h"

/// Buffer size for image provider and camera.
#define CAMERA_BUFFER_SIZE 4

/**
 * \brief Camera base class.
 * \note You cannot directly construct objects.  \n
 *   Instead, find camera types in subclass documents,
 *   include camera_factory.h and use
 *   CameraFactory::Instance()::CreateCamera(camera_type_name).
 */
class Camera : NO_COPY, NO_MOVE {
public:
    Camera() : stream_running_(false),
               daemon_thread_id_(0),
               stop_daemon_thread_flag_(false) {}

    virtual ~Camera() = default;

    /**
     * \brief Open a camera.
     * \param [in] serial_number Serial number of the camera you wanna open.
     * \param [in] config_file Will load config from this file.
     * \return Whether the camera is opened.
     * \note Another try after failures is allowed.
     */
    virtual bool OpenCamera(const std::string &serial_number, const std::string &config_file) = 0;

    /**
     * \brief Close the opened camera.
     * \return Whether the camera is closed normally.
     * \attention No matter what is returned, the camera handle will be unreachable.
     */
    virtual bool CloseCamera() = 0;

    /**
     * \brief Get a frame with image and time stamp from internal image buffer.
     * \param [out] frame Acquired frame will be stored here.
     * \return Whether buffer is not empty, or if you can successfully get an frame.
     */
    virtual bool GetFrame(Frame &frame) = 0;

    /**
     * \brief Run the stream.
     * \return Whether stream is started normally.
     * \attention This function will return false when stream is already started or camera is not opened.
     */
    virtual bool StartStream() = 0;

    /**
     * \brief Stop the stream.
     * \return Whether stream is stopped normally.
     * \attention This function will return false when stream is not started or camera is not opened.
     */
    virtual bool StopStream() = 0;

    /**
     * \brief Check if current device is connected.
     * \return Whether current device is connected.
     */
    virtual bool IsConnected() = 0;

    /**
     * \brief Import current config to specified file.
     * \param [in] file_path File path.
     * \return Whether config file is imported.
     */
    virtual bool ImportConfigurationFile(const std::string &file_path) = 0;

    /**
     * \brief Export current config to specified file.
     * \param [in] file_path File path.
     * \return Whether config file is saved.
     */
    virtual bool ExportConfigurationFile(const std::string &file_path) = 0;

    /**
     * \brief Set exposure time.
     * \param exposure_time Exposure time, automatically converted to corresponding data type.
     * \return Whether exposure time is set.
     */
    [[maybe_unused]] virtual bool SetExposureTime(uint32_t exposure_time) = 0;

    /**
     * \brief Set gain value.
     * \param gain Gain value, automatically converted to corresponding data type.
     * \return Whether gain value is set.
     */
    [[maybe_unused]] virtual bool SetGainValue(float gain) = 0;

protected:
    std::string serial_number_;     ///< Serial number.
    bool stream_running_;           ///< Stream running flag.
    pthread_t daemon_thread_id_;    ///< Daemon thread id.
    bool stop_daemon_thread_flag_;  ///< Flag to stop daemon thread.
    CircularBuffer<Frame, CAMERA_BUFFER_SIZE> buffer_;  ///< A ring buffer to store images.
};

#undef CAMERA_BUFFER_SIZE

#endif  // CAMERA_BASE_H_
