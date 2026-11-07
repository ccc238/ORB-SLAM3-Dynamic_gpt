#ifndef ORB_SLAM3_COARSE_POSE_QUALITY_H
#define ORB_SLAM3_COARSE_POSE_QUALITY_H
#include "Deep/B2ReliabilityConfig.h"
#include <cmath>
#include <string>
#include <opencv2/core.hpp>
#include <vector>
#include <algorithm>
namespace ORB_SLAM3 {
struct CoarsePoseQuality {
    int match_count = 0;
    int inlier_count = 0;
    float inlier_ratio = 0.0f;
    float median_reprojection_error = -1.0f;
    bool just_relocalized = false;
    bool reliable = false;
    std::string reason;
    int background_matches=0, background_samples=0, background_invalid=0, background_cells=0;
    float background_inlier_ratio=0, background_reprojection=-1, background_coverage=0;
    double reliability_ms=0;
};
inline CoarsePoseQuality EvaluateBackgroundQuality(CoarsePoseQuality q, const B2ReliabilityConfig& c) {
    if(!c.enable || !q.reliable) return q;
    const char* failure=nullptr;
    if(q.background_invalid || !std::isfinite(q.background_reprojection) ||
       !std::isfinite(q.background_coverage) || !std::isfinite(q.background_inlier_ratio))
        failure="BACKGROUND_NUMERIC_INVALID";
    else if(q.background_samples<c.minBackgroundInliers)
        failure="INSUFFICIENT_BACKGROUND_POSE_SAMPLES";
    else if(q.background_inlier_ratio<c.minBackgroundInlierRatio)
        failure="LOW_BACKGROUND_INLIER_RATIO";
    else if(q.background_reprojection<0 || q.background_reprojection>c.maxBackgroundReprojection)
        failure="BACKGROUND_REPROJECTION_HIGH";
    else if(q.background_coverage<c.minBackgroundCoverage)
        failure="BACKGROUND_COVERAGE_LOW";
    if(failure) {q.reliable=false;q.reason=failure;}
    return q;
}
struct CoarseBackgroundSample { cv::Point2f rawPixel; float reprojection; };
inline void MeasureBackgroundQuality(CoarsePoseQuality& q,
    const std::vector<CoarseBackgroundSample>& samples, int matches, int invalid,
    cv::Size size, const B2ReliabilityConfig& c) {
    q.background_matches=matches;q.background_samples=samples.size();q.background_invalid=invalid;
    q.background_reprojection=-1;q.background_coverage=0;q.background_cells=0;
    q.background_inlier_ratio=matches ? float(samples.size())/matches : 0;
    std::vector<float> errors;std::vector<bool> occupied(c.gridCols*c.gridRows,false);
    if(size.width<=0 || size.height<=0) {++q.background_invalid;return;}
    for(const auto& sample:samples) {
        if(!std::isfinite(sample.reprojection) || sample.reprojection<0 ||
           !std::isfinite(sample.rawPixel.x) || !std::isfinite(sample.rawPixel.y) ||
           sample.rawPixel.x<0 || sample.rawPixel.y<0 || sample.rawPixel.x>=size.width || sample.rawPixel.y>=size.height) {
            ++q.background_invalid;continue;
        }
        errors.push_back(sample.reprojection);
        const int col=std::min(c.gridCols-1,int(sample.rawPixel.x*c.gridCols/size.width));
        const int row=std::min(c.gridRows-1,int(sample.rawPixel.y*c.gridRows/size.height));
        occupied[row*c.gridCols+col]=true;
    }
    if(!errors.empty()) {
        std::sort(errors.begin(),errors.end());const size_t mid=errors.size()/2;
        q.background_reprojection=errors.size()%2 ? errors[mid] : .5f*(errors[mid-1]+errors[mid]);
    }
    q.background_cells=std::count(occupied.begin(),occupied.end(),true);
    q.background_coverage=float(q.background_cells)/occupied.size();
}
struct CoarsePoseContext {
    bool initialized = false;
    bool new_map = false;
    bool relocalizing = false;
    bool just_recovered = false;
    bool initial_success = false;
    bool finite_pose = false;
    bool pure_vo = false;
    int background_features = 0;
    int background_inliers = 0;
};
// Pure decision function: never receives a SLAM Frame/MapPoint or changes SLAM.
inline CoarsePoseQuality EvaluateCoarsePoseQuality(CoarsePoseQuality quality,
    const CoarsePoseContext& context, int minInliers, float minRatio)
{
    quality.inlier_ratio = quality.match_count > 0 ?
        float(quality.inlier_count) / quality.match_count : 0.0f;
    quality.reliable = false;
    if (context.new_map) quality.reason = "NEW_MAP";
    else if (!context.initialized) quality.reason = "NOT_INITIALIZED";
    else if (context.relocalizing) quality.reason = "RELOCALIZATION";
    else if (context.just_recovered) quality.reason = "JUST_RECOVERED";
    else if (!context.initial_success) quality.reason = "COARSE_TRACK_FAILED";
    else if (context.background_features < minInliers)
        quality.reason = "INSUFFICIENT_COARSE_FEATURES";
    else if (quality.inlier_count < minInliers)
        quality.reason = "INSUFFICIENT_COARSE_INLIERS";
    else if (context.background_inliers < minInliers)
        quality.reason = "INSUFFICIENT_COARSE_FEATURES";
    else if (!context.finite_pose || context.pure_vo ||
             quality.inlier_count > quality.match_count ||
             !std::isfinite(quality.inlier_ratio) || quality.inlier_ratio < minRatio)
        quality.reason = "POSE_NOT_RELIABLE";
    else {
        quality.reliable = true;
        quality.reason = "COARSE_POSE_RELIABLE";
    }
    return quality;
}
} // namespace ORB_SLAM3
#endif
