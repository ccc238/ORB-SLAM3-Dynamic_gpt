#include "FineFeaturePolicy.h"
#include "FeaturePolicyEvaluator.h"
#include <stdexcept>
namespace ORB_SLAM3 {
void VerifyFineOptimizerInputs(const Frame& frame)
{
    const auto policy=ComputeFeaturePolicies(frame,FeaturePolicyPhase::FINE,true);
    if(frame.mvpMapPoints.size()!=policy.size() || frame.mvbOutlier.size()!=policy.size())
        throw std::runtime_error("Fine optimizer input size mismatch");
    for(int i=0;i<frame.N;++i) {
        const bool reject=!FeaturePolicyAllows(policy[i],FeaturePolicyPhase::FINE);
        if(frame.mvFeaturePolicy[i]!=policy[i] || frame.mvDynamicExcluded[i]!=reject ||
            (reject && (frame.mvpMapPoints[i] || frame.mvbOutlier[i])))
            throw std::runtime_error("Fine optimizer input contains a forbidden feature or stale policy");
    }
}
FineFeaturePolicyReport ApplyCurrentFineFeaturePolicy(Frame& frame)
{
    if (!frame.mbSemanticAnnotated || !frame.SemanticMetadataAligned() ||
        frame.mvpMapPoints.size()!=static_cast<size_t>(frame.N) ||
        frame.mvbOutlier.size()!=static_cast<size_t>(frame.N))
        throw std::runtime_error("Fine policy requires aligned annotated Frame features");
    // Explicit action opt-in: the metadata may currently describe the coarse pass.
    const auto policy=ComputeFeaturePolicies(frame,FeaturePolicyPhase::FINE,true);
    FineFeaturePolicyReport result;
    for (int i=0;i<frame.N;++i) {
        const bool hadMatch=frame.mvpMapPoints[i]!=nullptr;
        result.before+=hadMatch;
        const bool reject=!FeaturePolicyAllows(policy[i],FeaturePolicyPhase::FINE);
        frame.mvFeaturePolicy[i]=policy[i];
        frame.mvDynamicExcluded[i]=reject;
        result.excludedFeatures+=reject;
        if (reject) {
            if (hadMatch) {
                switch(policy[i]) {
                case FeatureUsePolicy::EXCLUDE_PENDING: ++result.rejectPending; break;
                case FeatureUsePolicy::EXCLUDE_DYNAMIC: ++result.rejectDynamic; break;
                case FeatureUsePolicy::EXCLUDE_BOUNDARY: ++result.rejectBoundary; break;
                default: ++result.rejectUnknown; break;
                }
            }
            frame.mvpMapPoints[i]=nullptr;
            frame.mvbOutlier[i]=false;
        }
        if (frame.mvpMapPoints[i]) {
            ++result.after;
            result.pendingAtOptimizer+=policy[i]==FeatureUsePolicy::EXCLUDE_PENDING;
            result.dynamicAtOptimizer+=policy[i]==FeatureUsePolicy::EXCLUDE_DYNAMIC;
            result.forbiddenAtOptimizer+=reject;
        }
    }
    if (result.before-result.after!=result.rejectPending+result.rejectDynamic+
        result.rejectBoundary+result.rejectUnknown || result.forbiddenAtOptimizer ||
        result.pendingAtOptimizer || result.dynamicAtOptimizer)
        throw std::runtime_error("Forbidden feature association survived fine filtering");
    VerifyFineOptimizerInputs(frame);
    return result;
}
int FineStaticSupportRequired(unsigned long frameId,unsigned long lastRelocFrame,
    int maxFrames,bool recentlyLost)
{
    if(frameId<lastRelocFrame+maxFrames) return 50;
    return recentlyLost ? 11 : 30;
}
bool FinePoseSupported(int inliers,int requiredSupport,bool finitePose)
{
    return finitePose && inliers>=requiredSupport;
}
bool FinePolicyAllowsKeyFrame(bool optimizationExecuted,bool fineFailed)
{
    return optimizationExecuted && !fineFailed;
}
}
