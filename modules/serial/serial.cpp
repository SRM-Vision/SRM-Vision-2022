#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "serial.h"

/// \brief Automatically acquire UART device connected to the system.
inline std::string GetUartDeviceName() {
    FILE *ls = popen("ls /dev/ttyACM* --color=never", "r");
    static char name[127];
    if (fscanf(ls, "%s", name) == -1) {
        pclose(ls);
        return "";
    }
    pclose(ls);
    return name;
}

/// \brief Convert uint type baud-rate to termios type.
inline unsigned int ConvertBaudRate(unsigned int baud_rate) {
    switch (baud_rate) {
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 1152000:
            return B1152000;
        case 4000000:
            return B4000000;
        default:
            LOG(WARNING) << "Baud rate " << baud_rate << " is unsupported, use 115200 as default.";
            return B115200;
    }
}

bool Serial::StartCommunication() {
    if (communication_flag_)
        return false;

    // Find an UART device.
    serial_port_ = GetUartDeviceName();
    if (serial_port_.empty()) {
        LOG(ERROR) << "No UART device found.";
        return false;
    }

    // Open serial port.
    if (!OpenSerialPort(serial_port_)) {
        serial_port_ = "";
        return false;
    }

    // Start daemon thread.
    communication_flag_ = true;
    pthread_create(&daemon_thread_id_, nullptr, DaemonThreadFunction, this);
    DLOG(INFO) << "Serial port " << serial_port_ << "'s daemon thread "
               << std::to_string(daemon_thread_id_) << " started.";

    LOG(INFO) << "Serial communication started.";
    return true;
}

bool Serial::StopCommunication() {
    if (!communication_flag_)
        return false;

    // Stop daemon thread.
    communication_flag_ = false;
    pthread_join(daemon_thread_id_, nullptr);
    DLOG(INFO) << "Serial port " << serial_port_ << "'s daemon thread "
               << std::to_string(daemon_thread_id_) << " stopped.";
    daemon_thread_id_ = 0;

    // Close serial port.
    if (!CloseSerialPort())
        LOG(ERROR) << "Failed to close serial port " << serial_port_ << ".";
    serial_port_ = "";
    serial_fd_ = 0;

    LOG(INFO) << "Serial communication stopped.";
    return true;
}

bool Serial::OpenSerialPort(const std::string &port) {
    if (chmod(port.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == EACCES) {
        LOG(WARNING) << "Running in user mode, manually setting permission is required.";
        LOG(WARNING) << "To set permission of current serial port, run this command as root:";
        LOG(WARNING) << "  chmod 777 " << port;
    }

    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ == -1) {
        LOG(ERROR) << "Failed to open serial port " << port << ".";
        return false;
    }

    unsigned int baud_rate = ConvertBaudRate(4000000);
    termios termios_option{};
    tcgetattr(serial_fd_, &termios_option);

    cfmakeraw(&termios_option);

    cfsetispeed(&termios_option, baud_rate);
    cfsetospeed(&termios_option, baud_rate);

    tcsetattr(serial_fd_, TCSANOW, &termios_option);

    termios_option.c_cflag &= ~PARENB;
    termios_option.c_cflag &= ~CSTOPB;
    termios_option.c_cflag &= ~CSIZE;
    termios_option.c_cflag |= CS8;
    termios_option.c_cflag &= ~INPCK;
    termios_option.c_cflag |= (baud_rate | CLOCAL | CREAD);
    termios_option.c_cflag &= ~(INLCR | ICRNL);
    termios_option.c_cflag &= ~(IXON);
    termios_option.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termios_option.c_oflag &= ~OPOST;
    termios_option.c_oflag &= ~(ONLCR | OCRNL);
    termios_option.c_iflag &= ~(ICRNL | INLCR);
    termios_option.c_iflag &= ~(IXON | IXOFF | IXANY);
    termios_option.c_cc[VTIME] = 1;
    termios_option.c_cc[VMIN] = 1;

    tcflush(serial_fd_, TCIOFLUSH);

    DLOG(INFO) << "Opened serial port " << serial_fd_ << ".";
    LOG(INFO) << "Serial ready.";
    return true;
}

bool Serial::CloseSerialPort() const {
    tcflush(serial_fd_, TCIOFLUSH);
    if (close(serial_fd_) == -1) {
        LOG(ERROR) << "Failed to close serial port " << serial_fd_ << ".";
        return false;
    } else {
        DLOG(INFO) << "Closed serial port " << serial_fd_ << ".";
        return true;
    }
}

void *Serial::DaemonThreadFunction(void *obj) {
    auto self = (Serial *) obj;

    // Initialize data memory.
    memset(&self->receive_data_, 0, sizeof(self->receive_data_));
    memset(&self->send_data_, 0, sizeof(self->send_data_));

    // Start receiving and sending threads.
    self->receive_thread_status_ = true;
    self->send_thread_status_ = true;

    pthread_create(&self->receive_thread_id_, nullptr, ReceiveThreadFunction, self);
    DLOG(INFO) << "Serial port " << self->serial_port_ << "'s receiving thread "
               << std::to_string(self->receive_thread_id_) << " started.";
    pthread_create(&self->send_thread_id_, nullptr, SendThreadFunction, self);
    DLOG(INFO) << "Serial port " << self->serial_port_ << "'s sending thread "
               << std::to_string(self->send_thread_id_) << " started.";

    while (self->communication_flag_) {
        if (self->receive_thread_status_ && self->send_thread_status_)
            usleep(100);
        else {
            LOG(ERROR) << "Serial port " << self->serial_port_ << " works abnormal, try to reconnect.";

            // Pause another working threads.
            self->receive_thread_status_ = false;
            self->send_thread_status_ = false;

            // Clear memory.
            memset(&self->receive_data_, 0, sizeof(self->receive_data_));
            memset(&self->send_data_, 0, sizeof(self->send_data_));

            // Close serial port.
            if (!self->CloseSerialPort()) {
                LOG(WARNING) << "Serial port " << self->serial_port_
                             << " may be disconnected. Please check connection.";
            } else {
                LOG(ERROR) << "Serial port " << self->serial_port_ << " is still connected. What has happened?";
            }
            self->serial_fd_ = 0;

            // Reopen and find another serial port.
            for (; !self->OpenSerialPort(self->serial_port_); self->serial_port_ = GetUartDeviceName())
                usleep(200);

            LOG(INFO) << "Successfully reopened serial port " << self->serial_port_ << ".";

            // Resume all working threads.
            self->receive_thread_status_ = true;
            self->send_thread_status_ = true;
        }
    }

    // Stop all working threads.
    self->receive_thread_status_ = false;
    self->send_thread_status_ = false;

    pthread_join(self->receive_thread_id_, nullptr);
    DLOG(INFO) << "Serial port " << self->serial_port_ << "'s receiving thread "
               << std::to_string(self->receive_thread_id_) << " stopped.";
    self->receive_thread_id_ = 0;

    pthread_join(self->send_thread_id_, nullptr);
    DLOG(INFO) << "Serial port " << self->serial_port_ << "'s sending thread "
               << std::to_string(self->send_thread_id_) << " stopped.";
    self->send_thread_id_ = 0;

    memset(&self->receive_data_, 0, sizeof(self->receive_data_));
    memset(&self->send_data_, 0, sizeof(self->send_data_));

    return nullptr;
}

void *Serial::ReceiveThreadFunction(void *obj) {
    auto self = (Serial *) obj;

    while (self->communication_flag_) {
        if (!self->receive_thread_status_ || !self->send_thread_status_)
            usleep(100);
        else {
            if (!self->receive_data_lock_.try_lock_for(
                    std::chrono::duration<uint32_t, std::ratio<1, 1000>>(SERIAL_RECEIVING_TIMEOUT_MS)))
                LOG(WARNING) << "Serial port " << self->serial_port_
                             << "'s receiving thread is delayed for reading data.";
            else {
                auto t_1 = std::chrono::high_resolution_clock::now(),
                        t_2 = std::chrono::high_resolution_clock::now();
                unsigned int read_count = 0;
                for (unsigned int once_read_count;
                     read_count < int(sizeof(SerialReceivePacket))
                     && self->receive_thread_status_
                     && self->send_thread_status_;
                     t_2 = std::chrono::high_resolution_clock::now()) {
                    // Once reading time out.
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(t_2 - t_1).count()
                        > SERIAL_RECEIVING_TIMEOUT_MS) {
                        LOG(ERROR) << "Failed to read " << sizeof(SerialReceivePacket) - read_count
                                   << " bytes of data from serial port "
                                   << self->serial_port_ << " for time out.";
                        self->receive_thread_status_ = false;
                        break;
                    }

                    // Try to read data, tag status to abnormal when error occurs.
                    try {
                        once_read_count = read(self->serial_fd_,
                                               ((unsigned char *) (&self->receive_data_)) + read_count,
                                               sizeof(SerialReceivePacket) - read_count);
                    }
                    catch (std::exception &e) {
                        LOG(ERROR) << "Failed to communicate with serial port " << self->serial_port_ <<
                                   " with exception " << e.what() << ".";
                        self->receive_thread_status_ = false;
                        break;
                    }

                    // For ErrnoEAGAIN retry to read data.
                    if (once_read_count == -1) {
                        if (errno == EAGAIN) {
                            read_count = 0;
                            continue;
                        }
                        LOG(ERROR) << "Failed to receive data from serial port " << self->serial_port_ << ".";
                        self->receive_thread_status_ = false;
                        break;
                    }
                    read_count += once_read_count;

                    tcflush(self->serial_fd_, TCIFLUSH);
                    DLOG(INFO) << "Serial port " << self->serial_port_ << " OK.";
                }
                self->receive_data_lock_.unlock();
            }
        }
    }
    return nullptr;
}

void *Serial::SendThreadFunction(void *obj) {
    auto self = (Serial *) obj;

    while (self->communication_flag_) {
        if (!self->receive_thread_status_ || !self->send_thread_status_)
            usleep(100);
        else {
            if (!self->send_data_lock_.try_lock_for(
                    std::chrono::duration<uint32_t, std::ratio<1, 1000>>(SERIAL_SENDING_TIMEOUT_MS)))
                LOG(WARNING) << "Serial port " << self->serial_port_
                             << "'s sending thread is delayed for reading data.";
            else {
                for (unsigned int send_count;
                     self->receive_thread_status_ && self->send_thread_status_;
                     std::this_thread::sleep_for(std::chrono::milliseconds(1))) {
                    tcflush(self->serial_fd_, TCOFLUSH);

                    // Try to send data, tag status to abnormal when error occurs.
                    try {
                        send_count = write(self->serial_fd_, &self->send_data_, sizeof(send_data_));
                    }
                    catch (std::exception &e) {
                        LOG(ERROR) << "Failed to communicate with serial port " << self->serial_port_ <<
                                   " with exception " << e.what() << ".";
                        self->send_thread_status_ = false;
                        break;
                    }

                    // Send failed without exception.
                    if (send_count == -1) {
                        LOG(ERROR) << "Failed to send data to serial port " << self->serial_port_ << ".";
                        self->send_thread_status_ = false;
                        break;
                    } else if (send_count < static_cast<int>(sizeof(send_data_))) {
                        LOG(ERROR) << "Failed to send " << sizeof(send_data_) - send_count
                                   << " bytes of data to serial port " << self->serial_port_ << ".";
                        self->send_thread_status_ = false;
                        break;
                    } else {
                        DLOG(INFO) << "Sent " << sizeof(send_data_) - send_count
                                   << " bytes of data to serial port " << self->serial_port_ << ".";
                    }
                }
                self->send_data_lock_.unlock();
            }
        }
    }
    return nullptr;
}

Serial::~Serial() {
    if (communication_flag_) {
        StopCommunication();
        LOG(WARNING) << "Serial communication is not stopped manually though it works well.";
        LOG(WARNING) << "Please check your code to prevent potential logical errors.";
    }
}
