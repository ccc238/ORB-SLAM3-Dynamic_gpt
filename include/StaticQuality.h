#ifndef ORB_SLAM3_STATIC_QUALITY_H
#define ORB_SLAM3_STATIC_QUALITY_H
#include "Deep/StaticQualityConfig.h"
#include <sophus/se3.hpp>
#include <string>
namespace ORB_SLAM3 {
class Frame;
struct StaticQualitySummary {
    bool applied=false;
    std::string skipReason;
    int allowed=0,valid=0,neutral=0;
    double minimum=1,mean=0,sum=0,effective=0,computeMs=0;
};
StaticQualitySummary PrepareStaticQuality(Frame& frame,const Sophus::SE3f& coarse,
    bool reliable,const std::string& skipReason,const StaticQualityConfig& config);
}
#endif
