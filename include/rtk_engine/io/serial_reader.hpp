/**
 * @file serial_reader.hpp
 * @brief Serial port reader for hardware GNSS receiver ingestion.
 */

#ifndef RTK_ENGINE_IO_SERIAL_READER_HPP
#define RTK_ENGINE_IO_SERIAL_READER_HPP

#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

namespace rtk {

class SerialReader {
public:
    SerialReader(const std::string& port, int baud_rate = B115200) : port_(port), baud_rate_(baud_rate), fd_(-1) {}
    ~SerialReader() { disconnect(); }

    bool connect() {
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            std::cerr << "[IO] Error opening serial port " << port_ << "\n";
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) return false;

        cfsetospeed(&tty, baud_rate_);
        cfsetispeed(&tty, baud_rate_);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag |= (CLOCAL | CREAD);

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ISIG;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;

        tcsetattr(fd_, TCSANOW, &tty);
        
        std::cout << "[IO] Connected to serial port " << port_ << "\n";
        return true;
    }

    void disconnect() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    int read(uint8_t* buffer, size_t max_len) {
        if (fd_ < 0) return -1;
        return ::read(fd_, buffer, max_len);
    }

private:
    std::string port_;
    int baud_rate_;
    int fd_;
};

} // namespace rtk

#endif // RTK_ENGINE_IO_SERIAL_READER_HPP
