#include "Deep/MaskPostProcessor.h"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
namespace ORB_SLAM3 {
MaskPostProcessor::MaskPostProcessor(bool enabled, int radius)
    : enabled_(enabled), radius_(radius) {
    if (radius < 0 || radius > 1024)
        throw std::runtime_error("MaskPostProcess.DilateRadius must be in [0,1024]");
}
cv::Mat MaskPostProcessor::Apply(const cv::Mat& raw) const {
    if (raw.empty()) return cv::Mat();
    if (raw.type() != CV_8UC1)
        throw std::runtime_error("MaskPostProcess expects CV_8UC1");
    if (!enabled_ || radius_ == 0) return raw;
    cv::Mat result;
    cv::dilate(raw, result, cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * radius_ + 1, 2 * radius_ + 1)),
        cv::Point(-1,-1), 1);
    return result;
}
}
