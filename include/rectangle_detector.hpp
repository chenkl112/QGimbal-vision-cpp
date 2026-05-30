#pragma once

#include <opencv2/core.hpp>

#include <vector>

struct DetectedRect {
    cv::Point2f center;
    std::vector<cv::Point> corners;
    double area = 0.0;
};

struct RectangleDetectorConfig {
    double min_area_ratio = 0.05;
    double max_area_ratio = 0.80;
    double target_aspect_ratio = 26.0 / 17.0;
    double aspect_tolerance = 0.30;
    double approx_epsilon_ratio = 0.02;
    int blur_kernel_size = 5;
    double canny_low = 50.0;
    double canny_high = 150.0;
    int canny_aperture = 3;
};

std::vector<DetectedRect> detect_rectangles(
    const cv::Mat& frame,
    const RectangleDetectorConfig& cfg = RectangleDetectorConfig{});

void draw_detected_rect(cv::Mat& frame, const DetectedRect& rect);
