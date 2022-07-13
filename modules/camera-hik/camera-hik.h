/**
 * HikVision camera header.
 * @author trantuan-20048607
 * @date 2022.1.28
 * @warning NEVER include this file except in ./camera_hik.cpp.
 */

#ifndef CAMERA_HIK_H_
#define CAMERA_HIK_H_

/**
 * @brief HikRobot camera class implementation.
 * @warning NEVER directly use this class to create camera!  \n
 *   Turn to CameraFactory class for correct method.
 */
class [[maybe_unused]] HikCamera final : public Camera {
public:
    HikCamera() : device_(nullptr) {}

    ~HikCamera() final = default;

    bool OpenCamera(const std::string &, const std::string &) final;

    bool StartStream() final;

    inline bool GetFrame(Frame &frame) final { return buffer_.Pop(frame); }

    bool StopStream() final;

    bool CloseCamera() final;

    inline bool IsConnected() final {
        if (device_ == nullptr) return false;
        return MV_CC_IsDeviceConnected(device_);
    }

    inline bool ExportConfigurationFile(const std::string &file_path) final {
        auto status_code = MV_CC_FeatureSave(device_, file_path.c_str());
        if (status_code != MV_OK) {
            LOG(INFO) << "Failed to save " << serial_number_ << "'s configuration to "
                      << file_path << " with error 0x" << std::hex << status_code << ".";
            return false;
        }
        LOG(INFO) << "Saved " << serial_number_ << "'s configuration to " << file_path << ".";
        return true;
    }

    inline bool ImportConfigurationFile(const std::string &file_path) final {
        auto status_code = MV_CC_FeatureLoad(device_, file_path.c_str());
        if (status_code != MV_OK) {
            LOG(INFO) << "Failed to import " << serial_number_ << "'s configuration to "
                      << file_path << " with error 0x" << std::hex << status_code << ".";
            return false;
        }
        LOG(INFO) << "Imported " << serial_number_ << "'s configuration to " << file_path << ".";
        return true;
    }

    inline bool SetExposureTime(uint32_t exposure_time) final {
        return SetExposureTimeHikImplementation((float) exposure_time);
    }

    inline bool SetGainValue(float gain) final {
        if (MV_CC_SetFloatValue(device_, "Gain", gain) != MV_OK) {
            LOG(ERROR) << "Failed to set " << serial_number_ << "'s gain to "
                       << std::to_string(gain) << ".";
            return false;
        }
        DLOG(INFO) << "Set " << serial_number_ << "'s gain to "
                   << std::to_string(gain) << ".";
        return true;
    }

private:
    /**
     * @brief Internal image callback function.
     * @param image_data Internal image data.
     * @param frame_info Internal frame info structure.
     * @param obj It should be camera itself.
     */
    static void __stdcall ImageCallbackEx(unsigned char *image_data, MV_FRAME_OUT_INFO_EX *frame_info, void *obj);

    /**
     * @brief Daemon thread main function.
     * @param [in] obj Place camera itself here.
     * @attention Do NOT use this function in other place !!
     */
    static void *DaemonThreadFunction(void *);

    /**
     * @brief HikCamera implementation of exposure time setting.
     * @param exposure_time Float exposure time.
     * @return Whether exposure time is set.
     */
    inline bool SetExposureTimeHikImplementation(float exposure_time) {
        if (MV_CC_SetFloatValue(device_, "ExposureTime", exposure_time) != MV_OK) {
            LOG(ERROR) << "Failed to set " << serial_number_ << "'s exposure time to "
                       << std::to_string(exposure_time) << ".";
            return false;
        }
        DLOG(INFO) << "Set " << serial_number_ << "'s exposure time to "
                   << std::to_string(exposure_time) << ".";
        return true;
    }

    void *device_;  ///< Device handle.

    [[maybe_unused]] static CameraRegistry<HikCamera> hik_camera_registry_;  ///< Own registry in camera factory.
};

#endif  // CAMERA_HIK_H_
