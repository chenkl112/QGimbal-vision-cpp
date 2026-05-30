#include "tracker_control.hpp"

#include <algorithm>
#include <cmath>

namespace {
double apply_deadband(double err_px, double deadband_px) {
    if (std::abs(err_px) <= deadband_px) {
        return 0.0;
    }
    return err_px;
}

PID pid_from_cfg(const PIDConfig& cfg) {
    return PID(cfg.kp, cfg.ki, cfg.kd, cfg.integral_limit, cfg.output_limit);
}
}

GimbalTracker::GimbalTracker(const ControlConfig& cfg)
    : cfg_(cfg), yaw_pid_(pid_from_cfg(cfg.yaw_pid)), pitch_pid_(pid_from_cfg(cfg.pitch_pid)) {}

void GimbalTracker::reset() {
    yaw_pid_.reset();
    pitch_pid_.reset();
    last_seen_ts_ = 0.0;
}

std::pair<bool, ControlOutput> GimbalTracker::update(
    int frame_w,
    int frame_h,
    const std::optional<std::pair<double, double>>& target_center,
    double dt,
    double now_s) {
    if (!cfg_.enabled) {
        return {false, ControlOutput{}};
    }

    if (!target_center.has_value()) {
        if (last_seen_ts_ > 0.0 && (now_s - last_seen_ts_) <= cfg_.lost_timeout_s) {
            return {false, ControlOutput{}};
        }
        reset();
        return {true, ControlOutput{}};
    }

    last_seen_ts_ = now_s;

    const double center_x = static_cast<double>(frame_w) * 0.5;
    const double center_y = static_cast<double>(frame_h) * 0.5;
    const double cx = target_center->first;
    const double cy = target_center->second;

    const double err_x_px = apply_deadband(cx - center_x, cfg_.deadband_px);
    const double err_y_px = apply_deadband(cy - center_y, cfg_.deadband_px);

    const double half_w = std::max(1.0, center_x);
    const double half_h = std::max(1.0, center_y);
    const double err_x = err_x_px / half_w;
    const double err_y = err_y_px / half_h;

    const double yaw_u = yaw_pid_.update(err_x, dt);
    const double pitch_u = pitch_pid_.update(err_y, dt);

    ControlOutput out;
    out.yaw_rpm = yaw_u * cfg_.max_rpm_yaw;
    out.pitch_rpm = pitch_u * cfg_.max_rpm_pitch;
    out.err_x_px = err_x_px;
    out.err_y_px = err_y_px;

    if (cfg_.invert_yaw) {
        out.yaw_rpm = -out.yaw_rpm;
    }
    if (cfg_.invert_pitch) {
        out.pitch_rpm = -out.pitch_rpm;
    }

    return {true, out};
}
