#ifndef ORB_SLAM3_STATIC_QUALITY_CONFIG_H
#define ORB_SLAM3_STATIC_QUALITY_CONFIG_H
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace ORB_SLAM3 {
struct StaticQualityConfig {
    bool enable=false;
    double weightMin=0.10;
    double depthSigmaA=0.01; // metres
    double depthSigmaB=0.01; // inverse metres; experimental, not calibrated
    bool saveSummaryCsv=false;
};
inline void ValidateStaticQuality(const StaticQualityConfig& c) {
    // Nonnegative B and strictly positive A guarantee positivity for ALL z>0,
    // not only the B2 depth interval (main Frame depths are not B2 candidates).
    if(!std::isfinite(c.weightMin) || c.weightMin<=0 || c.weightMin>1 ||
       !std::isfinite(c.depthSigmaA) || c.depthSigmaA<=0 ||
       !std::isfinite(c.depthSigmaB) || c.depthSigmaB<0)
        throw std::runtime_error("StaticQuality requires 0<WeightMin<=1, DepthSigmaA>0, DepthSigmaB>=0, all finite");
}
inline double StaticDepthWeight(double measured,double predicted,const StaticQualityConfig& c) {
    const double sigma=c.depthSigmaA+c.depthSigmaB*measured*measured;
    const double u=std::abs(measured-predicted)/sigma;
    return std::max(c.weightMin,1.0/(1.0+u*u));
}
}
#endif
