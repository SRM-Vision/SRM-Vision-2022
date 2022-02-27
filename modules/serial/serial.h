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
#include "lang-feature-extension/disable_constructor.h"
#include "data-structure/serial_data.h"

#define SERIAL_RECEIVING_TIMEOUT_MS 10  ///< Receiving thread time out. (millisecond)
#define SERIAL_SENDING_TIMEOUT_MS 10    ///< Sending thread time out. (millisecond)

/// \brief Serial manager class.
class Serial : NO_COPY, NO_MOVE {
public:
    Serial() :
            communication_flag_(false),
            receive_thread_status_(false),
            send_thread_status_(false),
            serial_fd_(0),
            daemon_thread_id_(0),
            receive_thread_id_(0),
            send_thread_id_(0) {};

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
    [[maybe_unused]] inline bool SendData(SendPacket &data, const std::chrono::duration<Rep, Period> &duration) {
        if (!communication_flag_)
            return false;

        if (send_data_lock_.try_lock_for(duration)) {
            send_data_ = data;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            send_data_lock_.unlock();
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
    [[maybe_unused]] inline bool
    GetData(ReceivePacket &data, const std::chrono::duration<Rep, Period> &duration) {
        if (!communication_flag_)
            return false;

        if (receive_data_lock_.try_lock_for(duration)) {
            data = ReceivePacket(receive_data_);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            receive_data_lock_.unlock();
            return true;
        } else {
            LOG(WARNING) << "Data packet is discarded for reading data timing out.";
            return false;
        }
    }

private:
    /**
     * \brief Function for receiving thread.
     * \param obj Serial_ object itself.
     */
    static void *ReceiveThreadFunction(void *obj);

    /**
     * \brief Function for sending thread.
     * \param obj Serial_ object itself.
     */
    static void *SendThreadFunction(void *obj);

    /**
     * \brief Function for daemon thread.
     * \param obj Serial_ object itself.
     */
    static void *DaemonThreadFunction(void *obj);

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

    std::string serial_port_;  ///< Serial port filename, for logging.
    int serial_fd_;            ///< Serial FD.

    std::timed_mutex send_data_lock_;     ///< Timed mutex lock for sending data.
    std::timed_mutex receive_data_lock_;  ///< Timed mutex lock for reading data.
    SendPacket send_data_{};              ///< Sent data packet.
    SerialReceivePacket receive_data_{};  ///< Read data packet.

    bool communication_flag_;      ///< Flag to control daemon thread.
    bool receive_thread_status_;   ///< Flag to control receiving thread.
    bool send_thread_status_;      ///< Flag to control sending thread.
    pthread_t daemon_thread_id_;   ///< Daemon thread id.
    pthread_t receive_thread_id_;  ///< Receiving thread id.
    pthread_t send_thread_id_;     ///< Sending thread id.
};

#endif  // SERIAL_H_
