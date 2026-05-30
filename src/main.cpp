#include "gimbal_serial.hpp"
#include "rectangle_detector.hpp"
#include "tracker_control.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace {
constexpr int DEFAULT_CAMERA = 0;
constexpr int DEFAULT_WIDTH = 640;
constexpr int DEFAULT_HEIGHT = 480;
constexpr int DEFAULT_FPS = 120;
constexpr const char* DEFAULT_FOURCC = "MJPG";
constexpr int DEFAULT_DISPLAY = 1;
constexpr double DEFAULT_PRINT_INTERVAL = 0.05;
constexpr int DEFAULT_CONTROL_ENABLED = 1;
constexpr double DEFAULT_MAX_RPM = 20.0;
constexpr double DEFAULT_DEADBAND_PX = 0.0;
constexpr double DEFAULT_LOST_TIMEOUT_S = 0.4;

struct Args {
    int camera = DEFAULT_CAMERA;
    int display = DEFAULT_DISPLAY;
    double print_interval = DEFAULT_PRINT_INTERVAL;
    int control = DEFAULT_CONTROL_ENABLED;
    double max_rpm = DEFAULT_MAX_RPM;
    double deadband_px = DEFAULT_DEADBAND_PX;
    double lost_timeout = DEFAULT_LOST_TIMEOUT_S;
    std::string fourcc = DEFAULT_FOURCC;
    std::string serial_port =
#ifdef _WIN32
        "COM1";
#else
        "/dev/ttyS1";
#endif
    int serial_baud = 115200;

    RectangleDetectorConfig detector;
};

void print_usage(const char* exe) {
    std::cout
        << "Usage: " << exe << " [options]\n"
        << "\n"
        << "Camera:\n"
        << "  --camera N                Camera index, default " << DEFAULT_CAMERA << "\n"
        << "  --display 0|1             Show OpenCV window, default " << DEFAULT_DISPLAY << "\n"
        << "  --print-interval SEC      Console output interval for --display 0, default "
        << DEFAULT_PRINT_INTERVAL << "\n"
        << "  --fourcc CODE             Camera pixel format, default " << DEFAULT_FOURCC << "\n"
        << "\n"
        << "Control:\n"
        << "  --control 0|1             Enable PID serial output, default " << DEFAULT_CONTROL_ENABLED << "\n"
        << "  --max-rpm RPM             Max yaw/pitch RPM, default " << DEFAULT_MAX_RPM << "\n"
        << "  --deadband-px PX          Pixel deadband, default " << DEFAULT_DEADBAND_PX << "\n"
        << "  --lost-timeout SEC        Reset controller after target loss, default " << DEFAULT_LOST_TIMEOUT_S
        << "\n"
        << "  --serial-port PORT        Serial port, default platform value; use none to disable serial\n"
        << "  --serial-baud BAUD        Serial baudrate, default 115200\n"
        << "\n"
        << "Rectangle detector (from Rectangle-recognition):\n"
        << "  --min-area-ratio R        Default 0.05\n"
        << "  --max-area-ratio R        Default 0.80\n"
        << "  --aspect-ratio R          Default 26/17\n"
        << "  --aspect-tolerance R      Default 0.30\n"
        << "  --canny-low V             Default 50\n"
        << "  --canny-high V            Default 150\n"
        << "  --help                    Show this help\n";
}

bool parse_int(const std::string& value, int& out) {
    try {
        size_t pos = 0;
        const int parsed = std::stoi(value, &pos);
        if (pos != value.size()) {
            return false;
        }
        out = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_double(const std::string& value, double& out) {
    try {
        size_t pos = 0;
        const double parsed = std::stod(value, &pos);
        if (pos != value.size()) {
            return false;
        }
        out = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool needs_value(int i, int argc, const std::string& name) {
    if (i + 1 < argc) {
        return true;
    }
    std::cerr << "missing value for " << name << "\n";
    return false;
}

bool parse_args(int argc, char** argv, Args& args) {
    for (int i = 1; i < argc; ++i) {
        const std::string name = argv[i];
        if (name == "--help" || name == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        }

        auto value = [&]() -> std::string {
            if (!needs_value(i, argc, name)) {
                std::exit(2);
            }
            return argv[++i];
        };

        bool ok = true;
        if (name == "--camera") {
            ok = parse_int(value(), args.camera);
        } else if (name == "--display") {
            ok = parse_int(value(), args.display);
        } else if (name == "--print-interval") {
            ok = parse_double(value(), args.print_interval);
        } else if (name == "--control") {
            ok = parse_int(value(), args.control);
        } else if (name == "--max-rpm") {
            ok = parse_double(value(), args.max_rpm);
        } else if (name == "--deadband-px") {
            ok = parse_double(value(), args.deadband_px);
        } else if (name == "--lost-timeout") {
            ok = parse_double(value(), args.lost_timeout);
        } else if (name == "--fourcc") {
            args.fourcc = value();
        } else if (name == "--serial-port") {
            args.serial_port = value();
        } else if (name == "--serial-baud") {
            ok = parse_int(value(), args.serial_baud);
        } else if (name == "--min-area-ratio") {
            ok = parse_double(value(), args.detector.min_area_ratio);
        } else if (name == "--max-area-ratio") {
            ok = parse_double(value(), args.detector.max_area_ratio);
        } else if (name == "--aspect-ratio") {
            ok = parse_double(value(), args.detector.target_aspect_ratio);
        } else if (name == "--aspect-tolerance") {
            ok = parse_double(value(), args.detector.aspect_tolerance);
        } else if (name == "--canny-low") {
            ok = parse_double(value(), args.detector.canny_low);
        } else if (name == "--canny-high") {
            ok = parse_double(value(), args.detector.canny_high);
        } else {
            std::cerr << "unknown option: " << name << "\n";
            return false;
        }

        if (!ok) {
            std::cerr << "invalid value for " << name << "\n";
            return false;
        }
    }

    if (args.display != 0 && args.display != 1) {
        std::cerr << "--display must be 0 or 1\n";
        return false;
    }
    if (args.control != 0 && args.control != 1) {
        std::cerr << "--control must be 0 or 1\n";
        return false;
    }
    return true;
}

std::string fourcc_to_string(double raw) {
    const auto v = static_cast<int>(raw);
    std::string s;
    for (int i = 0; i < 4; ++i) {
        const char c = static_cast<char>((v >> (8 * i)) & 0xFF);
        if (c != '\0') {
            s.push_back(c);
        }
    }
    return s.empty() ? "unknown" : s;
}

int fourcc_from_string(const std::string& s) {
    std::string code = s;
    while (code.size() < 4) {
        code.push_back(' ');
    }
    return cv::VideoWriter::fourcc(code[0], code[1], code[2], code[3]);
}

double seconds_since_start() {
    static const auto start = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - start).count();
}
}

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) {
        print_usage(argv[0]);
        return 2;
    }

    cv::VideoCapture cap(args.camera);
    if (!cap.isOpened()) {
        std::cerr << "failed to open camera index " << args.camera << "\n";
        return 2;
    }

    cap.set(cv::CAP_PROP_FOURCC, fourcc_from_string(args.fourcc));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, DEFAULT_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, DEFAULT_HEIGHT);
    cap.set(cv::CAP_PROP_FPS, DEFAULT_FPS);

    std::cout << "camera=" << args.camera
              << " requested=" << DEFAULT_WIDTH << "x" << DEFAULT_HEIGHT << "@" << DEFAULT_FPS
              << " fourcc=" << args.fourcc
              << " actual=" << static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)) << "x"
              << static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)) << "@"
              << std::fixed << std::setprecision(1) << cap.get(cv::CAP_PROP_FPS)
              << " fourcc=" << fourcc_to_string(cap.get(cv::CAP_PROP_FOURCC)) << "\n";

    ControlConfig ctrl_cfg;
    ctrl_cfg.enabled = args.control != 0;
    ctrl_cfg.deadband_px = args.deadband_px;
    ctrl_cfg.lost_timeout_s = args.lost_timeout;
    ctrl_cfg.max_rpm_yaw = args.max_rpm;
    ctrl_cfg.max_rpm_pitch = args.max_rpm;

    GimbalTracker tracker(ctrl_cfg);
    GimbalSerial serial(args.serial_port, args.serial_baud);
    const bool serial_opened = serial.open();
    if (!serial_opened) {
        std::cout << "serial output disabled or unavailable: " << args.serial_port << "\n";
    }

    const bool display = args.display != 0;
    const std::string win_name = "Camera " + std::to_string(args.camera);
    if (display) {
        cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    }

    double last_print = 0.0;
    auto prev_tp = std::chrono::steady_clock::now();
    double fps = 0.0;
    bool laser_on = false;

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "failed to read frame, retrying...\n";
            cv::waitKey(100);
            continue;
        }

        const auto now_tp = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now_tp - prev_tp).count();
        prev_tp = now_tp;
        const double now_s = seconds_since_start();

        if (dt > 0.0) {
            constexpr double alpha = 0.98;
            const double inst_fps = 1.0 / dt;
            fps = fps > 0.0 ? (alpha * fps + (1.0 - alpha) * inst_fps) : inst_fps;
        }

        const auto rects = detect_rectangles(frame, args.detector);
        const DetectedRect* best = rects.empty() ? nullptr : &rects.front();
        const bool target_visible = best != nullptr;

        if (target_visible && !laser_on) {
            serial.enable_laser();
            laser_on = true;
        } else if (!target_visible && laser_on) {
            serial.disable_laser();
            laser_on = false;
        }

        std::optional<std::pair<double, double>> target_center;
        if (best != nullptr) {
            target_center = {best->center.x, best->center.y};
        }

        auto [should_send, ctrl_out] =
            tracker.update(frame.cols, frame.rows, target_center, std::max(dt, 1e-6), now_s);
        if (should_send) {
            serial.send_rpm(ctrl_out.yaw_rpm, ctrl_out.pitch_rpm);
        }

        if (display) {
            if (best != nullptr) {
                draw_detected_rect(frame, *best);
            }

            cv::drawMarker(
                frame,
                cv::Point(frame.cols / 2, frame.rows / 2),
                cv::Scalar(255, 0, 0),
                cv::MARKER_CROSS,
                18,
                2);

            cv::putText(
                frame,
                "FPS: " + std::to_string(static_cast<int>(fps)),
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 255, 0),
                2);

            std::ostringstream status;
            status << std::fixed << std::setprecision(0)
                   << "err(px)=(" << ctrl_out.err_x_px << "," << ctrl_out.err_y_px << ") rpm=("
                   << std::setprecision(1) << ctrl_out.yaw_rpm << "," << ctrl_out.pitch_rpm << ")";
            cv::putText(
                frame,
                status.str(),
                cv::Point(10, 65),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 255),
                2);

            cv::imshow(win_name, frame);
            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) {
                break;
            }
        } else {
            if (args.print_interval <= 0.0 || (now_s - last_print) >= args.print_interval) {
                last_print = now_s;
                std::cout << std::fixed << std::setprecision(1);
                if (best == nullptr) {
                    std::cout << "fps=" << fps << " rect=none rpm=(" << ctrl_out.yaw_rpm << ","
                              << ctrl_out.pitch_rpm << ")\n";
                } else {
                    std::cout << "fps=" << fps << " cx=" << best->center.x << " cy=" << best->center.y
                              << " area=" << best->area << " err=(" << ctrl_out.err_x_px << ","
                              << ctrl_out.err_y_px << ") rpm=(" << ctrl_out.yaw_rpm << ","
                              << ctrl_out.pitch_rpm << ")\n";
                }
            }
        }
    }

    if (laser_on) {
        serial.disable_laser();
    }
    cap.release();
    serial.close();
    if (display) {
        cv::destroyAllWindows();
    }

    return 0;
}
