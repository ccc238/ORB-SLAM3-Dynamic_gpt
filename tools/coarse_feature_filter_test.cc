#include "CoarseFeatureFilter.h"
#include <iostream>
#include <cstdint>
#include <stdexcept>
using namespace ORB_SLAM3;
static void Require(bool value,const char* message) {
    if (!value) throw std::runtime_error(message);
}
struct Fixture {
    Frame frame;
    std::vector<FeatureUsePolicy> policy;
    Fixture() {
        frame.N=8; frame.mnId=17; frame.mTimeStamp=1.25;
        frame.ResetSemanticMetadata(); frame.mbSemanticAnnotated=true;
        frame.mvSemanticRegion={SemanticRegion::BACKGROUND,SemanticRegion::PERSON_INTERIOR,
            SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_INTERIOR,
            SemanticRegion::PERSON_BOUNDARY,SemanticRegion::UNKNOWN,
            SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_INTERIOR};
        frame.mvInstanceId={-1,0,1,2,3,-1,4,-1};
        frame.mvTrackId={-1,1,2,3,4,-1,5,-1};
        frame.mvPreviousB4State={TemporalState::UNKNOWN,TemporalState::STATIC,TemporalState::PENDING,
            TemporalState::DYNAMIC,TemporalState::STATIC,TemporalState::STATIC,
            TemporalState::PENDING,TemporalState::STATIC};
        // Contradictory CURRENT states must not influence the COARSE policy.
        frame.mvCurrentB4State.assign(8,TemporalState::STATIC);
        policy=ComputeFeaturePolicies(frame,FeaturePolicyPhase::COARSE);
        frame.SetPose(Sophus::SE3f());
        frame.mvpMapPoints.resize(8);
        // Tokens only: no MapPoint object is accessed or owned by this test.
        for (int i=0;i<8;++i)
            frame.mvpMapPoints[i]=reinterpret_cast<MapPoint*>(uintptr_t(0x1000+16*i));
        frame.mvbOutlier.assign(8,true);
    }
};
int main() {
    try {
        {
            Fixture f; int calls=0;
            const auto report=OptimizeWithCoarseFeatureFilter(f.frame,f.policy,2,2,.30f,2,[&](){
                ++calls;
                for (int i=0;i<8;++i) {
                    Require(bool(f.frame.mvpMapPoints[i])==(i<2),"Only BG and previous S may reach optimizer");
                    Require(f.frame.mvDynamicExcluded[i]==(i>=2),"Exclusion metadata mismatch");
                    if(i>=2) Require(!f.frame.mvbOutlier[i],"Policy exclusion is not an outlier");
                }
                return 2;
            },[](){return 2;});
            Require(calls==1 && report.applied && !report.fallback && report.before==8 &&
                report.rejected==6 && report.after==2 && report.inliers==2,"Successful filtering report");
            bool caught=false;
            try { ComputeFeaturePolicies(f.frame,FeaturePolicyPhase::FINE); }
            catch(const std::runtime_error&) {caught=true;}
            Require(caught,"Evidence-only guard must remain strict");
            Require(ComputeFeaturePolicies(f.frame,FeaturePolicyPhase::COARSE,true)==f.policy,"Action summary opt-in");
        }
        for (int failure=0;failure<5;++failure) {
            Fixture f;
            if(failure==2) {
                f.frame.mvSemanticRegion[2]=SemanticRegion::BACKGROUND;
                f.policy=ComputeFeaturePolicies(f.frame,FeaturePolicyPhase::COARSE);
            }
            const auto original=f.frame.mvpMapPoints;
            const auto outliers=f.frame.mvbOutlier;
            const auto policies=f.frame.mvFeaturePolicy;
            const auto excluded=f.frame.mvDynamicExcluded;
            const auto pose=f.frame.GetPose().matrix();
            int calls=0;
            const auto report=OptimizeWithCoarseFeatureFilter(f.frame,f.policy,failure==0?3:2,2,
                failure==2?.90f:.30f,2,[&](){
                    ++calls;
                    const bool restored=failure==0 || calls==2;
                    if (restored) {
                        Require(f.frame.mvpMapPoints==original && f.frame.mvbOutlier==outliers,
                            "Original associations/outliers MUST be restored BEFORE fallback optimizer");
                        Require(f.frame.mvFeaturePolicy==policies && f.frame.mvDynamicExcluded==excluded,
                            "Fallback must restore policy metadata");
                        Require(f.frame.GetPose().matrix().isApprox(pose),"Fallback must restore initial pose");
                        return 8;
                    }
                    f.frame.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),Eigen::Vector3f(1,2,3)));
                    f.frame.mvbOutlier[0]=false;
                    if(failure==1) return 1;
                    if(failure==2) return 2; // 2/3 < 0.90: inadequate inlier ratio.
                    return 2;
                },[&](){return failure==3?1:2;},[&](){return failure==4?
                    std::string("COARSE_FILTER_INSUFFICIENT_BACKGROUND_INLIERS"):std::string();});
            Require(report.fallback && !report.applied && report.restoredInliers==8 &&
                calls==(failure==0?1:2),"Failure must actually rerun original optimizer");
        }
        {
            Fixture f; const auto original=f.frame.mvpMapPoints;
            bool caught=false;
            try { OptimizeWithCoarseFeatureFilter(f.frame,f.policy,2,2,.3f,2,
                []()->int{throw std::runtime_error("optimizer error");},[](){return 2;}); }
            catch(const std::runtime_error&) {caught=true;}
            Require(caught && f.frame.mvpMapPoints==original &&
                std::count(f.frame.mvDynamicExcluded.begin(),f.frame.mvDynamicExcluded.end(),true)==0,
                "Exceptions must restore state and propagate");
        }
        std::cout << "PASS previous-state filtering, BG/S allow, P/D/new/boundary/unknown reject, "
            "outlier separation, pre/post-optimization rollback, pose rollback, mapped/background guards, exception safety\n";
    } catch(const std::exception& error) {std::cerr << "FAIL " << error.what() << '\n';return 1;}
}
