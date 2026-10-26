#ifndef ORB_SLAM3_MASK_POST_PROCESSOR_H
#define ORB_SLAM3_MASK_POST_PROCESSOR_H
#include <opencv2/core.hpp>
namespace ORB_SLAM3 {
class MaskPostProcessor {
public:
    MaskPostProcessor(bool enabled = false, int radius = 3);
    cv::Mat Apply(const cv::Mat& raw) const;
    bool Enabled() const { return enabled_; }
    int Radius() const { return radius_; }
private:
    bool enabled_;
    int radius_;
};
}
#endif
