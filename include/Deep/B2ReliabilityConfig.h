#ifndef ORB_SLAM3_B2_RELIABILITY_CONFIG_H
#define ORB_SLAM3_B2_RELIABILITY_CONFIG_H
#include <cmath>
#include <stdexcept>
namespace ORB_SLAM3 {
struct B2ReliabilityConfig {
    bool enable = false;
    int depthRadius = 1, minDepthSamples = 5;
    float minDepthValidRatio = .60f;
    float depthEdgeAbsolute = .05f, depthEdgeRelative = .02f;
    int minBackgroundInliers = 30;
    float minBackgroundInlierRatio = .30f;
    float maxBackgroundReprojection = 2.5f;
    int gridCols = 4, gridRows = 3;
    float minBackgroundCoverage = .25f;
};
inline void ValidateB2Reliability(const B2ReliabilityConfig& c) {
    if (c.depthRadius<1 || c.depthRadius>10 || c.minDepthSamples<1 ||
        c.minDepthSamples>(2*c.depthRadius+1)*(2*c.depthRadius+1) ||
        !std::isfinite(c.minDepthValidRatio) || c.minDepthValidRatio<=0 || c.minDepthValidRatio>1 ||
        !std::isfinite(c.depthEdgeAbsolute) || c.depthEdgeAbsolute<0 ||
        !std::isfinite(c.depthEdgeRelative) || c.depthEdgeRelative<0 ||
        c.depthEdgeAbsolute+c.depthEdgeRelative<=0 || c.minBackgroundInliers<1 ||
        !std::isfinite(c.minBackgroundInlierRatio) || c.minBackgroundInlierRatio<=0 || c.minBackgroundInlierRatio>1 ||
        !std::isfinite(c.maxBackgroundReprojection) || c.maxBackgroundReprojection<=0 ||
        c.gridCols<1 || c.gridCols>32 || c.gridRows<1 || c.gridRows>32 ||
        !std::isfinite(c.minBackgroundCoverage) || c.minBackgroundCoverage<=0 || c.minBackgroundCoverage>1)
        throw std::runtime_error("Invalid B2.Reliability configuration");
}
}
#endif
