#ifndef ORB_SLAM3_COARSE_POSE_QUALITY_H
#define ORB_SLAM3_COARSE_POSE_QUALITY_H
#include <cmath>
#include <string>
namespace ORB_SLAM3 {
struct CoarsePoseQuality {
    int match_count = 0;
    int inlier_count = 0;
    float inlier_ratio = 0.0f;
    float median_reprojection_error = -1.0f;
    bool just_relocalized = false;
    bool reliable = false;
    std::string reason;
};
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
