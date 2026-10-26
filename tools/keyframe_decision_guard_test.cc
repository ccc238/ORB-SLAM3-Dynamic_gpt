#include "KeyFrameDecisionGuard.h"
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3;
static void Require(bool good,const char* message) {if(!good) throw std::runtime_error(message);}
static KeyFrameDecisionContext Ready() {
    KeyFrameDecisionContext c;
    c.active=true;c.frameId=c.semanticFrameId=c.finalFrameId=42;
    c.coarseReliable=c.evidenceComplete=c.fineCompleted=c.optimizerExecuted=c.fineSucceeded=true;
    c.semanticPolicyAvailable=c.finitePose=c.finalPoseUnchanged=true;
    c.coarseMatches=1000;c.coarseInliers=900;
    c.finalInliers=c.currentInliers=35;c.requiredStaticSupport=30;
    c.needEvaluated=c.needed=true;c.decisionInliers=35;
    return c;
}
static void Denied(const KeyFrameDecisionContext& c,const char* reason) {
    const auto decision=EvaluateKeyFrameDecision(c);
    Require(!decision.allowed && decision.reason==reason,reason);
    Require(!EvaluateKeyFrameDecision(c,true).allowed,"Create guard bypassed denied need guard");
}
int main() {
    try {
        auto c=Ready();Require(EvaluateKeyFrameDecision(c,true).allowed,"Final-stage supported frame denied");
        c=Ready();c.coarseFallback=true;Denied(c,"COARSE_POSE_FALLBACK");
        c.bootstrap=true;Denied(c,"COARSE_POSE_FALLBACK"); // logging flag is not a guard bypass
        c=Ready();c.coarseReliable=false;Denied(c,"COARSE_POSE_UNRELIABLE");
        c=Ready();c.fineFailed=true;Denied(c,"FINE_TRACKING_FAILED");
        c=Ready();c.fineSucceeded=false;Denied(c,"FINE_TRACKING_FAILED");
        c=Ready();c.optimizerExecuted=false;Denied(c,"FINE_TRACKING_FAILED");
        c=Ready();c.fineCompleted=false;Denied(c,"FINE_TRACKING_NOT_COMPLETED");
        c=Ready();c.evidenceComplete=false;Denied(c,"EVIDENCE_NOT_COMPLETE");
        c=Ready();c.semanticPolicyAvailable=false;Denied(c,"SEMANTIC_POLICY_UNAVAILABLE");
        c=Ready();--c.semanticFrameId;Denied(c,"SEMANTIC_POLICY_UNAVAILABLE");
        c=Ready();--c.finalFrameId;Denied(c,"FINAL_SNAPSHOT_STALE");
        c=Ready();c.finalPoseUnchanged=false;Denied(c,"FINAL_POSE_CHANGED");
        c=Ready();c.finitePose=false;Denied(c,"FINAL_POSE_INVALID");
        // Huge coarse matches/inliers must NEVER compensate for weak fine inliers.
        c=Ready();c.finalInliers=c.currentInliers=29;Denied(c,"INSUFFICIENT_STATIC_SUPPORT");
        c=Ready();c.currentInliers=c.coarseInliers;Denied(c,"FINAL_INLIERS_CHANGED");
        c=Ready();c.finalInliers=c.currentInliers=30;c.decisionInliers=30;
        Require(EvaluateKeyFrameDecision(c,true).allowed,"Exact original support threshold denied");
        c=Ready();c.needed=false;Require(!EvaluateKeyFrameDecision(c,true).allowed,"Unapproved creation allowed");
        c=Ready();c.needEvaluated=false;Require(!EvaluateKeyFrameDecision(c,true).allowed,"Missing NeedNewKeyFrame allowed");
        c=Ready();c.decisionInliers=c.coarseMatches;
        Require(!EvaluateKeyFrameDecision(c,true).allowed,"Creation used coarse-stage decision input");
        // Reset cannot reuse a previous frame's final pose/inliers/semantic policy.
        c=KeyFrameDecisionContext();c.active=true;c.frameId=43;
        Require(!EvaluateKeyFrameDecision(c,true).allowed,"Default/new frame reused stale provenance");
        c=KeyFrameDecisionContext();Require(EvaluateKeyFrameDecision(c,true).allowed,"Disabled legacy path changed");
        std::cout << "PASS coarse fallback/unreliable, fine failure, stale/invalid semantic policy, static support, "
            "final pose/inlier provenance, Need-before-Create, threshold boundary, per-frame reset, legacy bypass\n";
    } catch(const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
}
