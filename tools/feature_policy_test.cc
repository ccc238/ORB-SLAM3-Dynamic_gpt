#include "FeaturePolicyEvaluator.h"
#include <sstream>
#include <stdexcept>
#include <iostream>

using namespace ORB_SLAM3;
static void Require(bool ok,const char* message) { if (!ok) throw std::runtime_error(message); }
static bool Allowed(const Frame& frame,FeaturePolicyPhase phase) {
    return FeaturePolicyAllows(ComputeFeaturePolicies(frame,phase).at(0),phase);
}

int main() {
    try {
        Frame frame;
        frame.mnId=42; frame.mTimeStamp=1.25; frame.N=1; frame.ResetSemanticMetadata();
        frame.mbSemanticAnnotated=true;
        frame.mvSemanticRegion[0]=SemanticRegion::PERSON_INTERIOR;
        frame.mvInstanceId[0]=0; frame.mvTrackId[0]=7;
        const TemporalState states[]={TemporalState::STATIC,TemporalState::PENDING,TemporalState::DYNAMIC,TemporalState::UNKNOWN};
        for (auto previous:states) for (auto current:states) {
            frame.mvPreviousB4State[0]=previous; frame.mvCurrentB4State[0]=current;
            const auto coarse=ComputeFeaturePolicies(frame,FeaturePolicyPhase::COARSE);
            Require(Allowed(frame,FeaturePolicyPhase::COARSE)==(previous==TemporalState::STATIC),"Coarse must use previous state only");
            Require(Allowed(frame,FeaturePolicyPhase::FINE)==(current==TemporalState::STATIC),"Fine must use current state only");
            Require(Allowed(frame,FeaturePolicyPhase::MAPPING)==(current==TemporalState::STATIC),"Map must use current state only");
            SummarizeFeaturePolicies(frame,coarse);
            Require(frame.mvFeaturePolicy[0]==FeatureUsePolicy::USE_NORMAL && !frame.mvDynamicExcluded[0],"Proposal was applied");
        }
        for (auto region:{SemanticRegion::BACKGROUND,SemanticRegion::PERSON_BOUNDARY,SemanticRegion::UNKNOWN}) {
            frame.mvSemanticRegion[0]=region;
            frame.mvPreviousB4State[0]=frame.mvCurrentB4State[0]=TemporalState::STATIC;
            for (auto phase:{FeaturePolicyPhase::COARSE,FeaturePolicyPhase::FINE,FeaturePolicyPhase::MAPPING})
                Require(Allowed(frame,phase)==(region==SemanticRegion::BACKGROUND),"Boundary/unknown priority");
        }
        frame.mvSemanticRegion[0]=SemanticRegion::PERSON_INTERIOR;
        frame.mvTrackId[0]=-1;
        Require(!Allowed(frame,FeaturePolicyPhase::COARSE) && !Allowed(frame,FeaturePolicyPhase::FINE) &&
                !Allowed(frame,FeaturePolicyPhase::MAPPING),"Unknown instance must never be allowed");

        // BG, S->P, P->S, NEW P->P, D->D, boundary, unknown, invalid owner.
        frame.N=8; frame.ResetSemanticMetadata(); frame.mbSemanticAnnotated=true;
        frame.mvSemanticRegion={SemanticRegion::BACKGROUND,SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_INTERIOR,
            SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_BOUNDARY,
            SemanticRegion::UNKNOWN,SemanticRegion::PERSON_INTERIOR};
        frame.mvInstanceId={-1,0,1,2,3,4,-1,-1}; frame.mvTrackId={-1,7,8,9,10,11,-1,-1};
        frame.mvPreviousB4State={TemporalState::UNKNOWN,TemporalState::STATIC,TemporalState::PENDING,TemporalState::PENDING,
            TemporalState::DYNAMIC,TemporalState::STATIC,TemporalState::STATIC,TemporalState::STATIC};
        frame.mvCurrentB4State={TemporalState::UNKNOWN,TemporalState::PENDING,TemporalState::STATIC,TemporalState::PENDING,
            TemporalState::DYNAMIC,TemporalState::STATIC,TemporalState::STATIC,TemporalState::STATIC};
        const auto regions=frame.mvSemanticRegion;
        const auto policies=frame.mvFeaturePolicy;
        const auto previous=frame.mvPreviousB4State, current=frame.mvCurrentB4State;
        const auto ids=frame.mvTrackId, owners=frame.mvInstanceId;
        const auto excluded=frame.mvDynamicExcluded;
        auto coarse=ComputeFeaturePolicies(frame,FeaturePolicyPhase::COARSE);
        const auto s=SummarizeFeaturePolicies(frame,coarse);
        Require(s.coarseAllowed==2 && s.fineAllowed==2 && s.mapAllowed==2 && s.background==1 &&
                s.stationary==1 && s.pending==2 && s.dynamic==1 && s.boundary==1 && s.unknown==2,"Nine CSV count definitions");
        Require(coarse[3]==FeatureUsePolicy::EXCLUDE_PENDING,"NEW P must not enter coarse");
        const auto fine=ComputeFeaturePolicies(frame,FeaturePolicyPhase::FINE);
        Require(coarse[1]==FeatureUsePolicy::USE_NORMAL && fine[1]==FeatureUsePolicy::EXCLUDE_PENDING &&
                coarse[2]==FeatureUsePolicy::EXCLUDE_PENDING && fine[2]==FeatureUsePolicy::USE_NORMAL,"Transition timing");
        Require(frame.mvSemanticRegion==regions && frame.mvFeaturePolicy==policies && frame.mvPreviousB4State==previous &&
            frame.mvCurrentB4State==current && frame.mvTrackId==ids && frame.mvInstanceId==owners &&
            frame.mvDynamicExcluded==excluded,"Evaluation must not mutate Frame metadata");
        Require(FeaturePolicyAllows(FeatureUsePolicy::USE_POSE_ONLY,FeaturePolicyPhase::COARSE) &&
            FeaturePolicyAllows(FeatureUsePolicy::USE_POSE_ONLY,FeaturePolicyPhase::FINE) &&
            !FeaturePolicyAllows(FeatureUsePolicy::USE_POSE_ONLY,FeaturePolicyPhase::MAPPING),"USE_POSE_ONLY semantics");
        std::ostringstream csv; WriteFeaturePolicyCsvHeader(csv); WriteFeaturePolicyCsv(csv,frame,s);
        Require(csv.str().find("coarse_allowed_count,fine_allowed_count,map_allowed_count")!=std::string::npos &&
            csv.str().find("42,1.25,8,2,2,2,1,1,2,1,1,2,")!=std::string::npos,"CSV serialization");
        bool failed=false; coarse[0]=FeatureUsePolicy::EXCLUDE_UNKNOWN;
        try { SummarizeFeaturePolicies(frame,coarse); } catch(const std::runtime_error&) { failed=true; }
        Require(failed,"Stale/invalid coarse snapshot must fail");
        frame.N=0; frame.ResetSemanticMetadata(); frame.mbSemanticAnnotated=true;
        Require(SummarizeFeaturePolicies(frame,{}).coarseAllowed==0,"Empty frame");
        std::cout<<"PASS 16 state pairs, S->P/P->S timing, NEW P, boundary/unknown, counts/CSV, no application, empty frames\n";
        return 0;
    } catch(const std::exception& error) { std::cerr<<"FAIL: "<<error.what()<<'\n'; return 1; }
}
