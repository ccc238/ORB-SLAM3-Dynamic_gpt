#ifndef ORB_SLAM3_KEYFRAME_DECISION_GUARD_H
#define ORB_SLAM3_KEYFRAME_DECISION_GUARD_H
#include <string>
namespace ORB_SLAM3 {
// Final-stage provenance, not a new keyframe-insertion heuristic or threshold.
struct KeyFrameDecisionContext {
    bool active=false;
    long frameId=-1, semanticFrameId=-1, finalFrameId=-1;
    bool coarseFallback=false, coarseReliable=false, evidenceComplete=false;
    bool fineCompleted=false, optimizerExecuted=false, fineSucceeded=false, fineFailed=false;
    bool semanticPolicyAvailable=false, finitePose=false, finalPoseUnchanged=false;
    int coarseMatches=0, coarseInliers=0, finalInliers=-1, currentInliers=-1;
    int requiredStaticSupport=0, decisionInliers=-1;
    bool needEvaluated=false, needed=false, createAttempted=false, created=false, bootstrap=false;
};
struct KeyFrameGateResult { bool allowed=false; std::string reason; };
inline KeyFrameGateResult EvaluateKeyFrameDecision(const KeyFrameDecisionContext& c,
                                                  bool requireNeedDecision=false) {
    if(!c.active) return {true,"LEGACY_PATH"};
    if(c.coarseFallback) return {false,"COARSE_POSE_FALLBACK"};
    if(!c.coarseReliable) return {false,"COARSE_POSE_UNRELIABLE"};
    if(c.semanticFrameId!=c.frameId || !c.semanticPolicyAvailable)
        return {false,"SEMANTIC_POLICY_UNAVAILABLE"};
    if(!c.evidenceComplete) return {false,"EVIDENCE_NOT_COMPLETE"};
    if(!c.fineCompleted) return {false,"FINE_TRACKING_NOT_COMPLETED"};
    if(c.fineFailed || !c.fineSucceeded || !c.optimizerExecuted)
        return {false,"FINE_TRACKING_FAILED"};
    if(c.finalFrameId!=c.frameId) return {false,"FINAL_SNAPSHOT_STALE"};
    if(!c.finitePose) return {false,"FINAL_POSE_INVALID"};
    if(!c.finalPoseUnchanged) return {false,"FINAL_POSE_CHANGED"};
    if(c.requiredStaticSupport<=0 || c.finalInliers<c.requiredStaticSupport)
        return {false,"INSUFFICIENT_STATIC_SUPPORT"};
    if(c.currentInliers!=c.finalInliers) return {false,"FINAL_INLIERS_CHANGED"};
    if(requireNeedDecision && (!c.needEvaluated || !c.needed || c.decisionInliers!=c.finalInliers))
        return {false,"NEED_KEYFRAME_NOT_APPROVED"};
    return {true,"READY_FINAL_POSE_AND_INLIERS"};
}
}
#endif
