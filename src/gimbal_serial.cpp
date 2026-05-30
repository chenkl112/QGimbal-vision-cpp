#include "gimbal_serial.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <iostream>
#include <utility>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace {
bool serial_disabled_name(const std::string& port) {
    std::string normalized = port;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return normalized.empty() || normalized == "none" || normalized == "null" || normalized == "disabled";
}

void append_float_le(std::vector<std::uint8_t>& data, float value) {
    std::uint8_t bytes[sizeof(float)];
    std::memcpy(bytes, &value, sizeof(float));

#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    for (int i = static_cast<int>(sizeof(float)) - 1; i >= 0; --i) {
        data.push_back(bytes[i]);
    }
#else
    for (std::uint8_t byte : bytes) {
        data.push_back(byte);
    }
#endif
}

#ifndef _WIN32
speed_t baud_to_termios(int baudrate) {
    switch (baudrate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
#ifdef B230400
        case 230400:
            return B230400;
#endif
#ifdef B460800
        case 460800:
            return B460800;
#endif
#ifdef B921600
        case 921600:
            return B921600;
#endif
        default:
            return B115200;
    }
}
#endif
}

std::uint8_t crc8(const std::vector<std::uint8_t>& data) {
    std::uint8_t crc = 0x00;
    for (std::uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if ((crc & 0x80U) != 0U) {
                crc = static_cast<std::uint8_t>((crc << 1U) ^ 0x07U);
            } else {
                crc = static_cast<std::uint8_t>(crc << 1U);
            }
        }
    }
    return crc;
}

std::vector<std::uint8_t> build_packet(GimbalCommand cmd, float yaw, float pitch) {
    std::vector<std::uint8_t> packet;
    packet.reserve(10);
    packet.push_back(static_cast<std::uint8_t>(cmd));
    append_float_le(packet, yaw);
    append_float_le(packet, pitch);
    packet.push_back(crc8(packet));
    return packet;
}

GimbalSerial::GimbalSerial(std::string port, int baudrate) : port_(std::move(port)), baudrate_(baudrate) {}

GimbalSerial::~GimbalSerial() {
    close();
}

bool GimbalSerial::open() {
    if (serial_disabled_name(port_)) {
        return false;
    }

#ifdef _WIN32
    std::string device = port_;
    if (device.rfind("\\\\.\\", 0) != 0 && device.rfind("COM", 0) == 0) {
        device = "\\\\.\\" + device;
    }

    HANDLE h = CreateFileA(
        device.c_str(),
        GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        std::cerr << "warning: failed to open serial port " << port_ << "\n";
        handle_ = nullptr;
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        CloseHandle(h);
        std::cerr << "warning: failed to read serial settings for " << port_ << "\n";
        return false;
    }

    dcb.BaudRate = static_cast<DWORD>(baudrate_);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        std::cerr << "warning: failed to apply serial settings for " << port_ << "\n";
        return false;
    }

    handle_ = h;
#else
    fd_ = ::open(port_.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "warning: failed to open serial port " << port_ << ": " << std::strerror(errno) << "\n";
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "warning: failed to read serial settings for " << port_ << ": " << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    cfmakeraw(&tty);
    const speed_t baud = baud_to_termios(baudrate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "warning: failed to apply serial settings for " << port_ << ": " << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }
#endif

    apply_startup_states();
    return true;
}

void GimbalSerial::close() {
    if (is_open() && enabled == 1) {
        send_command(GimbalCommand::Disable);
    }

#ifdef _WIN32
    if (handle_ != nullptr) {
        CloseHandle(static_cast<HANDLE>(handle_));
        handle_ = nullptr;
    }
#else
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#endif
}

bool GimbalSerial::is_open() const {
#ifdef _WIN32
    return handle_ != nullptr;
#else
    return fd_ >= 0;
#endif
}

void GimbalSerial::send_command(GimbalCommand cmd, float yaw, float pitch) {
    write_bytes(build_packet(cmd, yaw, pitch));
}

void GimbalSerial::send_rpm(double yaw_rpm, double pitch_rpm) {
    send_command(GimbalCommand::SpeedCtrl, static_cast<float>(yaw_rpm), static_cast<float>(pitch_rpm));
}

void GimbalSerial::start() {
    send_command(GimbalCommand::Enable);
}

void GimbalSerial::stop() {
    send_command(GimbalCommand::Disable);
}

void GimbalSerial::enable_stability() {
    send_command(GimbalCommand::EnableStability);
}

void GimbalSerial::disable_stability() {
    send_command(GimbalCommand::DisableStability);
}

void GimbalSerial::enable_laser() {
    send_command(GimbalCommand::EnableLaser);
}

void GimbalSerial::disable_laser() {
    send_command(GimbalCommand::DisableLaser);
}

void GimbalSerial::reset_imu() {
    send_command(GimbalCommand::ResetImu);
}

bool GimbalSerial::write_bytes(const std::vector<std::uint8_t>& bytes) {
    if (!is_open()) {
        return false;
    }

#ifdef _WIN32
    DWORD written = 0;
    return WriteFile(static_cast<HANDLE>(handle_), bytes.data(), static_cast<DWORD>(bytes.size()), &written, nullptr) != 0 &&
           written == bytes.size();
#else
    const ssize_t written = ::write(fd_, bytes.data(), bytes.size());
    return written == static_cast<ssize_t>(bytes.size());
#endif
}

void GimbalSerial::apply_startup_states() {
    if (enabled == 1) {
        start();
    } else if (enabled == 0) {
        stop();
    }

    if (stability_enabled == 1) {
        enable_stability();
    } else if (stability_enabled == 0) {
        disable_stability();
    }

    if (laser_enabled == 1) {
        enable_laser();
    } else if (laser_enabled == 0) {
        disable_laser();
    }
}
