/**
 * HikVision camera config generator source file.
 * @author trantuan-20048607
 * @version 1
 * @date 2022.3.19
 */

#include <iostream>
#include <cstring>
#include <vector>
#include <MvCameraControl.h>

#define ERROR std::cout << "\033[1;31m"
#define WARNING std::cout << "\033[1;33m"
#define INFO std::cout << "\033[0m"

#define ENDL "\033[0m" << std::endl

int main() {
    INFO << "This is a simple HikVision ApiGen config file generator made by Tran Tuan." << ENDL;
    INFO << "Use this tool to generate a config to load through HikVision internal API." << ENDL;
    WARNING << "NOTE: Some configurations will remain if the camera was opened by MVS and has not been powered off."
            << ENDL;

    // Find USB devices.
    // ================================================
    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    auto status_code = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (MV_OK != status_code) {
        ERROR << "Failed to enumerate devices with error code " << "0x" << std::hex << status_code << "." << ENDL;
        return 0;
    }
    std::vector<std::string> sn_list;
    if (device_list.nDeviceNum > 0) {
        for (auto i = 0; i < device_list.nDeviceNum; ++i) {
            MV_CC_DEVICE_INFO *device_info = device_list.pDeviceInfo[i];
            std::string sn;
            if (device_info->nTLayerType == MV_GIGE_DEVICE) {
                sn = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stGigEInfo.chSerialNumber));
                INFO << "GigE device with serial number " << sn << " found, which is not supported." << ENDL;
            } else if (device_info->nTLayerType == MV_USB_DEVICE) {
                sn = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
                INFO << "USB device with serial number " << sn << " found." << ENDL;
                sn_list.push_back(sn);
            } else {
                WARNING << "Device with unsupported device transport layer protocol type " << sn << "found." << ENDL;
            }
        }
    }
    if (sn_list.empty()) {
        WARNING << "No USB device found. Program will exit now." << ENDL;
        return 0;
    }

    // Select a device.
    // ================================================
    INFO << "==========Device List in Serial Number==========" << ENDL;
    for (auto i = 0; i < sn_list.size(); ++i)
        INFO << "[" << (i + 1) << "] SN: " << sn_list[i] << ENDL;
    INFO << "================================================" << ENDL;
    int s = -1;
    while (s < 0 || s > sn_list.size()) {
        INFO << "Select one to open, select 0 to exit: ";
        std::cin >> s;
        std::cin.get();
    }
    if (!s)
        return 0;
    std::string sn = sn_list[s - 1];

    // Open device.
    // ================================================
    void *device = nullptr;
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO *device_info = device_list.pDeviceInfo[i];
        std::string temp_sn;
        if (device_info->nTLayerType == MV_USB_DEVICE)
            temp_sn = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
        if (temp_sn == sn) {
            status_code = MV_CC_CreateHandle(&device, device_info);
            if (MV_OK != status_code) {
                ERROR << "Failed to create device handle with error " << "0x" << std::hex << status_code
                      << ".";
                return 0;
            }
            goto confirmed_sn;  // WARNING: Do NOT change anything when NOT you used GOTO statement.
        }
    }
    ERROR << "Select USB device is not found. Program will exit." << ENDL;
    return 0;
    confirmed_sn:
    status_code = MV_CC_OpenDevice(device, MV_ACCESS_Exclusive, 1);
    if (MV_OK != status_code) {
        ERROR << "Failed to open device with error " << "0x" << std::hex << status_code << "." << ENDL;
        status_code = MV_CC_DestroyHandle(device);
        if (MV_OK != status_code)
            ERROR << "Failed to destroy device handle with error " << "0x" << std::hex << status_code << "." << ENDL;
        return 0;
    }

    // Save configuration.
    // ================================================
    std::string file_path = "../../config/cameras/HV" + sn + ".txt";
    status_code = MV_CC_FeatureSave(device, file_path.c_str());
    if (status_code != MV_OK) {
        INFO << "Failed to save " << sn << "'s configuration to "
             << file_path << " with error 0x" << std::hex << status_code << "." << ENDL;
        return 0;
    }
    INFO << "Saved " << sn << "'s configuration to " << file_path << "." << ENDL;

    // Close device.
    // ================================================
    status_code = MV_CC_CloseDevice(device);
    if (MV_OK != status_code)
        ERROR << "Failed to close camera with error " << "0x" << std::hex << status_code << "." << ENDL;

    // Destroy device handle.
    // ================================================
    status_code = MV_CC_DestroyHandle(device);
    if (MV_OK != status_code)
        ERROR << "Failed to destroy handle with error " << "0x" << std::hex << status_code << "." << ENDL;
    INFO << "Closed camera " << sn << "." << ENDL;

    return 0;
}
