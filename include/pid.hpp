#pragma once

class PID {
public:
    PID(double kp, double ki, double kd, double integral_limit, double output_limit);

    void reset();
    double update(double error, double dt);

private:
    double kp_;
    double ki_;
    double kd_;
    double integral_limit_;
    double output_limit_;
    double integral_ = 0.0;
    bool has_prev_error_ = false;
    double prev_error_ = 0.0;
};
