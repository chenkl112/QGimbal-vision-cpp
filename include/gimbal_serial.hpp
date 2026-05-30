#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

enum class GimbalCommand : std::uint8_t {
    Nop = 0x00,
    Enable = 0x01,
    Disable = 0x02,
    CurrentCtrl = 0x03,
    SpeedCtrl = 0x04,
    AngleCtrl = 0x05,
    LowSpeedCtrl = 0x06,
    StepAngleCtrl = 0x07,
    EnableStability = 0xFF,
    DisableStability = 0xFE,
    EnableLaser = 0xFD,
    DisableLaser = 0xFC,
    ResetImu = 0xFB,
};

std::uint8_t crc8(const std::vector<std::uint8_t>& data);

std::vector<std::uint8_t> build_packet(GimbalCommand cmd, float yaw = 0.0F, float pitch = 0.0F);

class GimbalSerial {
public:
    GimbalSerial(std::string port, int baudrate);
    ~GimbalSerial();

    GimbalSerial(const GimbalSerial&) = delete;
    GimbalSerial& operator=(const GimbalSerial&) = delete;

    bool open();
    void close();
    bool is_open() const;

    void send_command(GimbalCommand cmd, float yaw = 0.0F, float pitch = 0.0F);
    void send_rpm(double yaw_rpm, double pitch_rpm);
    void start();
    void stop();
    void enable_stability();
    void disable_stability();
    void enable_laser();
    void disable_laser();
    void reset_imu();

    int laser_enabled = 2;
    int enabled = 1;
    int stability_enabled = 2;

private:
    bool write_bytes(const std::vector<std::uint8_t>& bytes);
    void apply_startup_states();

    std::string port_;
    int baudrate_;

#ifdef _WIN32
    void* handle_ = nullptr;
#else
    int fd_ = -1;
#endif
};
