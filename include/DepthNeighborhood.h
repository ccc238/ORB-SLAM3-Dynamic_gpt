#ifndef ORB_SLAM3_DEPTH_NEIGHBORHOOD_H
#define ORB_SLAM3_DEPTH_NEIGHBORHOOD_H
#include "Deep/B2ReliabilityConfig.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include <vector>
namespace ORB_SLAM3 {
struct DepthNeighborhoodResult {
    float depth=-1, validRatio=0, spread=-1;
    int validSamples=0, eligibleSamples=0;
    bool valid=false, edge=false;
    const char* reason="DEPTH_MISSING";
};
// Same threshold convention as existing Frame annotation: max(abs,rel*z).
inline float DepthEdgeLimit(float depth, float absolute, float relative) {
    return std::max(absolute,relative*depth);
}
inline DepthNeighborhoodResult InspectDepthNeighborhood(const cv::Mat& depth,
    const cv::Point2f& pixel, const cv::Mat& mask, const B2ReliabilityConfig& config,
    float minDepth, float maxDepth) {
    DepthNeighborhoodResult out;
    if(depth.empty() || depth.type()!=CV_32FC1 || !std::isfinite(pixel.x) || !std::isfinite(pixel.y)) return out;
    if(!mask.empty() && (mask.type()!=CV_8UC1 || mask.size()!=depth.size()))
        throw std::runtime_error("B2 depth mask coordinate mismatch");
    const int x=cvRound(pixel.x), y=cvRound(pixel.y);
    if(x<0 || y<0 || x>=depth.cols || y>=depth.rows) return out;
    std::vector<float> values;
    for(int v=std::max(0,y-config.depthRadius);v<=std::min(depth.rows-1,y+config.depthRadius);++v)
        for(int u=std::max(0,x-config.depthRadius);u<=std::min(depth.cols-1,x+config.depthRadius);++u) {
            if(!mask.empty() && !mask.at<uchar>(v,u)) continue;
            ++out.eligibleSamples;
            const float z=depth.at<float>(v,u);
            if(std::isfinite(z) && z>0) values.push_back(z);
        }
    out.validSamples=values.size();
    out.validRatio=out.eligibleSamples ? float(out.validSamples)/out.eligibleSamples : 0;
    if(values.empty()) return out;
    if(out.validSamples<config.minDepthSamples || out.validRatio<config.minDepthValidRatio) {
        out.reason="DEPTH_NEIGHBORHOOD_INSUFFICIENT";return out;
    }
    std::sort(values.begin(),values.end());
    out.depth=values[values.size()/2]; // preserve upper-median convention
    if(out.depth<minDepth || out.depth>maxDepth) {out.reason="DEPTH_OUT_OF_RANGE";return out;}
    out.spread=values.back()-values.front();
    out.edge=out.spread>DepthEdgeLimit(out.depth,config.depthEdgeAbsolute,config.depthEdgeRelative);
    if(out.edge) {out.reason="DEPTH_EDGE";return out;}
    out.valid=true;out.reason="OK";return out;
}
}
#endif
