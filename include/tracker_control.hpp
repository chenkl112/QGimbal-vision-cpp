#pragma once

#include "pid.hpp"

#include <chrono>
#include <optional>
#include <utility>

struct PIDConfig {
    double kp = 0.8;
    double ki = 0.0;
    double kd = 0.08;
    double integral_limit = 0.8;
    double output_limit = 1.0;
};

struct ControlConfig {
    bool enabled = true;
    double deadband_px = 6.0;
    double lost_timeout_s = 0.25;
    double max_rpm_yaw = 120.0;
    double max_rpm_pitch = 120.0;
    bool invert_yaw = true;
    bool invert_pitch = false;

    PIDConfig yaw_pid{4.0, 0.80, 0.08, 0.2, 1.0};
    PIDConfig pitch_pid{3.0, 0.60, 0.06, 0.2, 1.0};
};

struct ControlOutput {
    double yaw_rpm = 0.0;
    double pitch_rpm = 0.0;
    double err_x_px = 0.0;
    double err_y_px = 0.0;
};

class GimbalTracker {
public:
    explicit GimbalTracker(const ControlConfig& cfg);

    void reset();

    std::pair<bool, ControlOutput> update(
        int frame_w,
        int frame_h,
        const std::optional<std::pair<double, double>>& target_center,
        double dt,
        double now_s);

private:
    ControlConfig cfg_;
    PID yaw_pid_;
    PID pitch_pid_;
    double last_seen_ts_ = 0.0;
};
