/**
 * HikVision camera config generator source file.
 * @details The formats of config file are different between MVS client and MvCameraControl API. \n
 *   Connect the HikVision camera (change parameters in MVS client), compile and run this tool,
 *   select the camera and the corresponding MvCameraControl API config file will be generated
 *   at ../../config/cameras. \n
 *   This program supports standard ASCII color output and requires MVS API to compile and run.
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
    std::vector<std::string> serial_number_list;
    if (device_list.nDeviceNum > 0)
        for (auto i = 0; i < device_list.nDeviceNum; ++i) {
            MV_CC_DEVICE_INFO *device_info = device_list.pDeviceInfo[i];
            std::string serial_number;
            if (device_info->nTLayerType == MV_GIGE_DEVICE) {
                serial_number = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stGigEInfo.chSerialNumber));
                INFO << "GigE device with serial number " << serial_number << " found, which is not supported." << ENDL;
            } else if (device_info->nTLayerType == MV_USB_DEVICE) {
                serial_number = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
                INFO << "USB device with serial number " << serial_number << " found." << ENDL;
                serial_number_list.push_back(serial_number);
            } else
                WARNING << "Device with unsupported device transport layer protocol type " << serial_number << "found." << ENDL;
        }

    if (serial_number_list.empty()) {
        WARNING << "No USB device found. Program will exit now." << ENDL;
        return 0;
    }

    // Select a device.
    // ================================================
    INFO << "==========Device List in Serial Number==========" << ENDL;
    for (auto i = 0; i < serial_number_list.size(); ++i)
        INFO << "[" << (i + 1) << "] SN: " << serial_number_list[i] << ENDL;
    INFO << "================================================" << ENDL;
    int selected_id = -1;
    while (selected_id < 0 || selected_id > serial_number_list.size()) {
        INFO << "Select one to open, select 0 to exit: ";
        std::cin >> selected_id;
        std::cin.get();
    }
    if (!selected_id)
        return 0;
    std::string selected_serial_number = serial_number_list[selected_id - 1];

    // Open device.
    // ================================================
    void *device = nullptr;
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO *device_info = device_list.pDeviceInfo[i];
        std::string temp_serial_number;
        if (device_info->nTLayerType == MV_USB_DEVICE)
            temp_serial_number = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
        if (temp_serial_number == selected_serial_number) {
            status_code = MV_CC_CreateHandle(&device, device_info);
            if (MV_OK != status_code) {
                ERROR << "Failed to create device handle with error " << "0x" << std::hex << status_code
                      << ".";
                return 0;
            }
            goto confirmed;  // WARNING: Do NOT change anything when NOT YOU used GOTO statement.
        }
    }
    ERROR << "Select USB device is not found. Program will exit." << ENDL;
    return 0;
    confirmed:
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
    std::string file_path = "../../config/cameras/HV" + selected_serial_number + ".txt";
    status_code = MV_CC_FeatureSave(device, file_path.c_str());
    if (status_code != MV_OK) {
        INFO << "Failed to save " << selected_serial_number << "'selected_id configuration to "
             << file_path << " with error 0x" << std::hex << status_code << "." << ENDL;
        return 0;
    }
    INFO << "Saved " << selected_serial_number << "'selected_id configuration to " << file_path << "." << ENDL;

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
    INFO << "Closed camera " << selected_serial_number << "." << ENDL;

    return 0;
}
