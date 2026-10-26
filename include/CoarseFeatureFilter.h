#ifndef ORB_SLAM3_COARSE_FEATURE_FILTER_H
#define ORB_SLAM3_COARSE_FEATURE_FILTER_H
#include "FeaturePolicyEvaluator.h"
#include <algorithm>
#include <functional>
#include <stdexcept>

namespace ORB_SLAM3 {
struct CoarseFilterReport {
    int before = 0, rejected = 0, after = 0, inliers = 0;
    int restoredInliers = -1;
    bool applied = false, fallback = false;
    std::string reason = "NA";
};

// Only association vectors, pose and policy metadata are touched. MapPoint
// objects are never mutated. The callback is the existing pose optimizer.
// On failure the ORIGINAL inputs are restored BEFORE invoking it again.
inline CoarseFilterReport OptimizeWithCoarseFeatureFilter(Frame& frame,
    const std::vector<FeatureUsePolicy>& policy, int minMatches, int minInliers,
    float minRatio, int minMappedInliers, const std::function<int()>& optimize,
    const std::function<int()>& mappedInliers,
    const std::function<std::string()>& extraFailure = [](){return std::string();})
{
    if (!frame.mbSemanticAnnotated || !frame.SemanticMetadataAligned() ||
        policy.size() != static_cast<size_t>(frame.N) ||
        frame.mvpMapPoints.size() != policy.size() || frame.mvbOutlier.size() != policy.size() ||
        !frame.HasPose())
        throw std::runtime_error("Coarse filtering requires aligned annotated features and an initial pose");
    const auto backup_map_points = frame.mvpMapPoints;
    const auto backup_outliers = frame.mvbOutlier;
    const auto backup_excluded = frame.mvDynamicExcluded;
    const auto backup_policy = frame.mvFeaturePolicy;
    const Sophus::SE3f backup_pose = frame.GetPose();
    const auto restore = [&]() {
        frame.mvpMapPoints = backup_map_points;
        frame.mvbOutlier = backup_outliers;
        frame.mvDynamicExcluded = backup_excluded;
        frame.mvFeaturePolicy = backup_policy;
        frame.SetPose(backup_pose);
    };
    CoarseFilterReport report;
    for (int i=0; i<frame.N; ++i) {
        report.before += frame.mvpMapPoints[i] != nullptr;
        const bool reject = !FeaturePolicyAllows(policy[i],FeaturePolicyPhase::COARSE);
        frame.mvFeaturePolicy[i] = policy[i];
        frame.mvDynamicExcluded[i] = reject;
        if (reject) {
            report.rejected += frame.mvpMapPoints[i] != nullptr;
            frame.mvpMapPoints[i] = nullptr;
            frame.mvbOutlier[i] = false; // Policy exclusion is NOT an optimization outlier.
        }
    }
    report.after = report.before - report.rejected;
    try {
        if (report.after < minMatches) report.reason = "COARSE_FILTER_INSUFFICIENT_MATCHES";
        else {
            report.inliers = optimize();
            if (report.inliers < minInliers)
                report.reason = "COARSE_FILTER_INSUFFICIENT_INLIERS";
            else if (report.inliers > report.after ||
                     float(report.inliers)/report.after < minRatio ||
                     !frame.GetPose().matrix().allFinite())
                report.reason = "COARSE_FILTER_POSE_NOT_RELIABLE";
            else if (mappedInliers() < minMappedInliers)
                report.reason = "COARSE_FILTER_INSUFFICIENT_MAP_INLIERS";
            else {
                const std::string failure = extraFailure();
                if (failure.empty()) { report.applied = true; return report; }
                report.reason = failure;
            }
        }
        report.fallback = true;
        restore();
        report.restoredInliers = optimize();
    } catch (...) { restore(); throw; }
    return report;
}
} // namespace ORB_SLAM3
#endif
