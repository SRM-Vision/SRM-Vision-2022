/**
 * Serial class header.
 * \author trantuan-20048607, anonymity
 * \date 2022.1.28
 * \details Include this file to use serial communication.
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <mutex>
#include <thread>
#include <glog/logging.h>
#include "lang-feature-extension/disable_constructors.h"
#include "packet.h"

/// \brief Serial manager class.
class Serial : NO_COPY, NO_MOVE {
public:
    Serial() :
            communication_flag_(false),
            serial_fd_(0) {};

    ~Serial();

    /**
     * \brief Is serial port connected?
     * \return Whether serial port is opened.
     */
    [[maybe_unused]] [[nodiscard]] inline bool IsOpened() const { return !serial_port_.empty(); }

    /**
     * \brief Start serial communication.
     * \return Whether communication is successfully established.
     */
    bool StartCommunication();

    /**
     * \brief Stop serial communication.
     * \return Whether communication is successfully stopped.
     */
    bool StopCommunication();

    /**
     * \brief Send a data packet.
     * \param [in] data A data packet to send.
     * \param [in] duration Time duration.
     * \return Whether data packet is successfully sent.
     */
    template<typename Rep, typename Period>
    [[maybe_unused]] bool SendData(SerialSendPacket &data, const std::chrono::duration<Rep, Period> &duration) {
        if (!communication_flag_)
            return false;

        std::unique_lock<std::timed_mutex> lock(send_data_lock_, duration);
        if (lock.owns_lock()) {
            send_data_ = data;
            Send();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return true;
        } else {
            LOG(WARNING) << "Failed to send data to serial port " << serial_port_ << " for time out.";
            return false;
        }
    }

    /**
     * \brief Get a data packet.
     * \param [out] data A data packet to receive.
     * \param [in] duration Time duration.
     * \return Whether data packet is successfully read.
     */
    template<typename Rep, typename Period>
    [[maybe_unused]] bool GetData(SerialReceivePacket &data, const std::chrono::duration<Rep, Period> &duration) {
        if (!communication_flag_)
            return false;

        std::unique_lock<std::timed_mutex> lock(receive_data_lock_, duration);
        if (lock.owns_lock()) {
            Receive();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            data = receive_data_;
            return true;
        } else {
            LOG(WARNING) << "Data packet is discarded for reading data timed out.";
            return false;
        }
    }

private:
    /**
     * \brief Open specified serial port.
     * \param port Serial port.
     * \return Whether specified serial port is successfully opened.
     */
    [[nodiscard]] bool OpenSerialPort(const std::string &port);

    /**
     * \brief Close serial port.
     * \return Whether specified serial port is successfully closed.
     */
    [[nodiscard]] bool CloseSerialPort() const;

    bool Send();

    bool Receive();

    std::string serial_port_;  ///< Serial port filename, for logging.
    int serial_fd_;            ///< Serial FD.

    std::timed_mutex send_data_lock_;     ///< Timed mutex lock for sending data.
    std::timed_mutex receive_data_lock_;  ///< Timed mutex lock for reading data.
    SerialSendPacket send_data_{};        ///< Sent data packet.
    SerialReceivePacket receive_data_{};  ///< Read data packet.

    bool communication_flag_;  ///< Flag to control daemon thread.
};

#endif  // SERIAL_H_
