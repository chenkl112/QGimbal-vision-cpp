#include "rectangle_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace {
void draw_cross(cv::Mat& img, cv::Point center, cv::Scalar color, int size = 10, int thickness = 2) {
    cv::line(img, cv::Point(center.x - size, center.y), cv::Point(center.x + size, center.y), color, thickness);
    cv::line(img, cv::Point(center.x, center.y - size), cv::Point(center.x, center.y + size), color, thickness);
}

cv::Point2f line_intersection(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4) {
    const double x1 = p1.x;
    const double y1 = p1.y;
    const double x2 = p2.x;
    const double y2 = p2.y;
    const double x3 = p3.x;
    const double y3 = p3.y;
    const double x4 = p4.x;
    const double y4 = p4.y;

    const double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::abs(denom) < 1e-10) {
        return cv::Point2f(
            static_cast<float>((x1 + x2 + x3 + x4) / 4.0),
            static_cast<float>((y1 + y2 + y3 + y4) / 4.0));
    }

    const double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    return cv::Point2f(
        static_cast<float>(x1 + t * (x2 - x1)),
        static_cast<float>(y1 + t * (y2 - y1)));
}

bool check_aspect_ratio(const std::vector<cv::Point>& approx, double target_ratio, double tolerance) {
    const double side1 = cv::norm(cv::Point2f(approx[0]) - cv::Point2f(approx[1]));
    const double side2 = cv::norm(cv::Point2f(approx[1]) - cv::Point2f(approx[2]));
    const double side3 = cv::norm(cv::Point2f(approx[2]) - cv::Point2f(approx[3]));
    const double side4 = cv::norm(cv::Point2f(approx[3]) - cv::Point2f(approx[0]));

    const double max_side = std::max(std::max(side1, side2), std::max(side3, side4));
    const double min_side = std::min(std::min(side1, side2), std::min(side3, side4));
    if (min_side <= 0.0) {
        return false;
    }

    const double actual_ratio = max_side / min_side;
    const double min_ratio = target_ratio * (1.0 - tolerance);
    const double max_ratio = target_ratio * (1.0 + tolerance);
    return actual_ratio >= min_ratio && actual_ratio <= max_ratio;
}

int normalized_kernel_size(int requested) {
    int k = std::max(1, requested);
    if (k % 2 == 0) {
        ++k;
    }
    return k;
}
}

std::vector<DetectedRect> detect_rectangles(const cv::Mat& frame, const RectangleDetectorConfig& cfg) {
    std::vector<DetectedRect> rects;
    if (frame.empty()) {
        return rects;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    const int blur_size = normalized_kernel_size(cfg.blur_kernel_size);
    cv::GaussianBlur(gray, gray, cv::Size(blur_size, blur_size), 0);

    cv::Mat edges;
    cv::Canny(gray, edges, cfg.canny_low, cfg.canny_high, cfg.canny_aperture);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double img_area = static_cast<double>(frame.cols) * static_cast<double>(frame.rows);
    const double min_area = img_area * cfg.min_area_ratio;
    const double max_area = img_area * cfg.max_area_ratio;

    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < min_area || area > max_area) {
            continue;
        }

        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, cfg.approx_epsilon_ratio * cv::arcLength(contour, true), true);

        if (approx.size() != 4) {
            continue;
        }

        if (!check_aspect_ratio(approx, cfg.target_aspect_ratio, cfg.aspect_tolerance)) {
            continue;
        }

        DetectedRect rect;
        rect.area = area;
        rect.corners = approx;
        rect.center = line_intersection(approx[0], approx[2], approx[1], approx[3]);
        rects.push_back(rect);
    }

    std::sort(rects.begin(), rects.end(), [](const DetectedRect& a, const DetectedRect& b) {
        return a.area > b.area;
    });

    return rects;
}

void draw_detected_rect(cv::Mat& frame, const DetectedRect& rect) {
    if (rect.corners.size() != 4) {
        return;
    }

    for (size_t i = 0; i < rect.corners.size(); ++i) {
        cv::line(frame, rect.corners[i], rect.corners[(i + 1) % rect.corners.size()], cv::Scalar(0, 255, 0), 2);
        cv::circle(frame, rect.corners[i], 5, cv::Scalar(255, 0, 0), -1);
    }

    const cv::Point center(cvRound(rect.center.x), cvRound(rect.center.y));
    draw_cross(frame, center, cv::Scalar(0, 0, 255), 15, 2);

    cv::putText(
        frame,
        "A:" + std::to_string(static_cast<int>(rect.area)),
        cv::Point(center.x - 80, center.y - 20),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(0, 255, 0),
        2);
    cv::putText(
        frame,
        "X:" + std::to_string(center.x) + " Y:" + std::to_string(center.y),
        cv::Point(center.x - 80, center.y + 20),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(0, 255, 0),
        2);
}
