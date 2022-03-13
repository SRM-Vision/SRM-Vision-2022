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

    communication_flag_ = true;
    LOG(INFO) << "Serial communication started.";
    return true;
}

bool Serial::Send() {
    tcflush(serial_fd_, TCOFLUSH);

    ssize_t send_count;
    try {
        send_count = write(serial_fd_, &send_data_, sizeof(SerialSendPacket));
    }
    catch (...) {
        LOG(ERROR) << "Failed to send data to serial port " << serial_port_ << ".";
        return false;
    }

    if (send_count < sizeof(SerialSendPacket)) {
        LOG(ERROR) << "Failed to send data to serial port " << serial_port_ << ".";
        return false;
    } else {
        DLOG(INFO) << "Sent " << sizeof(SerialSendPacket) << " bytes of data to serial port " << serial_port_;
        DLOG(INFO) << "Data: " << send_data_;
        return true;
    }
}

bool Serial::Receive() {
    memset(&receive_data_, 0, sizeof(receive_data_));

    ssize_t read_count = 0;
    const auto t1 = std::chrono::high_resolution_clock::now();
    while (read_count < int(sizeof(SerialReceivePacket))) {
        auto t2 = std::chrono::high_resolution_clock::now();

        // 10ms time limit.
        if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() > 10) {
            DLOG(ERROR) << "Failed to receive " << sizeof(SerialReceivePacket) - read_count
                        << " bytes of data for time out.";
            return false;
        }

        ssize_t once_read_count;
        try {
            once_read_count = read(serial_fd_, ((unsigned char *) (&receive_data_)) + read_count,
                                   sizeof(SerialReceivePacket) - read_count);
        }
        catch (...) {
            LOG(ERROR) << "Failed to receive data from serial port " << serial_port_ << ".";
            return false;
        }

        if (once_read_count == -1) {
            if (errno == EAGAIN) {
                DLOG(WARNING) << "Delayed receiving data from serial port " << serial_port_ << ".";
                continue;
            }

            LOG(ERROR) << "Failed to receive data from serial port " << serial_port_ << ".";
            return false;
        }

        read_count += once_read_count;
    }

    tcflush(serial_fd_, TCIFLUSH);
    DLOG(INFO) << "Received " << sizeof(SerialReceivePacket) << " bytes of data.";
    DLOG(INFO) << "Data: " << receive_data_;
    return true;
}

bool Serial::StopCommunication() {
    if (!communication_flag_)
        return false;

    // Close serial port.
    if (!CloseSerialPort())
        LOG(ERROR) << "Failed to close serial port " << serial_port_ << ".";
    serial_port_ = "";
    serial_fd_ = -1;

    communication_flag_ = false;
    LOG(INFO) << "Serial communication stopped.";
    return true;
}

bool Serial::OpenSerialPort(const std::string &port) {
    if (chmod(port.c_str(), S_IRWXU | S_IRWXG | S_IRWXO)) {
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

Serial::~Serial() {
    if (communication_flag_) {
        StopCommunication();
        LOG(WARNING) << "Serial communication is not stopped manually though it works well.";
        LOG(WARNING) << "Please check your code to prevent potential logical errors.";
    }
}
