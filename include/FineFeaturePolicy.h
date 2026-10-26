#ifndef ORB_SLAM3_FINE_FEATURE_POLICY_H
#define ORB_SLAM3_FINE_FEATURE_POLICY_H
namespace ORB_SLAM3 {
class Frame;
struct FineFeaturePolicyReport {
    int before=0, rejectPending=0, rejectDynamic=0, rejectBoundary=0, rejectUnknown=0;
    int after=0, excludedFeatures=0;
    int pendingAtOptimizer=0, dynamicAtOptimizer=0, forbiddenAtOptimizer=0;
};
// Uses CURRENT B4 state; never changes keypoints, descriptor rows or MapPoint objects.
FineFeaturePolicyReport ApplyCurrentFineFeaturePolicy(Frame& frame);
void VerifyFineOptimizerInputs(const Frame& frame);
// Original non-inertial RGB-D TrackLocalMap acceptance thresholds, unchanged.
int FineStaticSupportRequired(unsigned long frameId, unsigned long lastRelocFrame,
    int maxFrames, bool recentlyLost);
bool FinePoseSupported(int inliers,int requiredSupport,bool finitePose);
bool FinePolicyAllowsKeyFrame(bool optimizationExecuted,bool fineFailed);
}
#endif
