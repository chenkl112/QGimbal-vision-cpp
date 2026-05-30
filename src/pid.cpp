#include "pid.hpp"

#include <algorithm>

namespace {
double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}
}

PID::PID(double kp, double ki, double kd, double integral_limit, double output_limit)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      integral_limit_(integral_limit),
      output_limit_(output_limit) {}

void PID::reset() {
    integral_ = 0.0;
    has_prev_error_ = false;
    prev_error_ = 0.0;
}

double PID::update(double error, double dt) {
    if (dt <= 0.0) {
        return 0.0;
    }

    const double p = kp_ * error;

    integral_ += error * dt;
    integral_ = clamp(integral_, -integral_limit_, integral_limit_);
    const double i = ki_ * integral_;

    double d = 0.0;
    if (has_prev_error_) {
        d = kd_ * ((error - prev_error_) / dt);
    }
    prev_error_ = error;
    has_prev_error_ = true;

    return clamp(p + i + d, -output_limit_, output_limit_);
}
