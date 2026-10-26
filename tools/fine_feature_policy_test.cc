#include "FineFeaturePolicy.h"
#include "Frame.h"
#include <iostream>
#include <cstdint>
#include <stdexcept>
using namespace ORB_SLAM3;
static void Require(bool good,const char* message) {if(!good) throw std::runtime_error(message);}
int main() {
    try {
        Frame frame;
        frame.N=10;frame.ResetSemanticMetadata();frame.mbSemanticAnnotated=true;
        frame.mvpMapPoints.resize(10);frame.mvbOutlier.assign(10,true);
        frame.mvSemanticRegion={SemanticRegion::BACKGROUND,SemanticRegion::PERSON_INTERIOR,
            SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_INTERIOR,
            SemanticRegion::PERSON_BOUNDARY,SemanticRegion::UNKNOWN,
            SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_INTERIOR,
            SemanticRegion::BACKGROUND,SemanticRegion::PERSON_INTERIOR};
        frame.mvInstanceId={-1,0,1,2,3,-1,-1,4,-1,5};
        frame.mvTrackId={-1,0,1,2,3,-1,-1,4,-1,5};
        frame.mvPreviousB4State.assign(10,TemporalState::STATIC);
        frame.mvPreviousB4State[2]=TemporalState::PENDING;
        frame.mvCurrentB4State.assign(10,TemporalState::STATIC);
        frame.mvCurrentB4State[1]=frame.mvCurrentB4State[7]=TemporalState::PENDING;
        frame.mvCurrentB4State[3]=TemporalState::DYNAMIC;
        frame.mvKeys.resize(10);frame.mvKeysUn.resize(10);
        frame.mDescriptors=cv::Mat::ones(10,32,CV_8U);
        const auto descriptors=frame.mDescriptors.clone();
        for(int i=0;i<10;++i) {
            frame.mvKeys[i]=frame.mvKeysUn[i]=cv::KeyPoint(float(i),float(i+10),1.f);
            frame.mvpMapPoints[i]=reinterpret_cast<MapPoint*>(uintptr_t(0x2000+16*i));
        }
        const auto rawKeys=frame.mvKeys;
        // P->S is newly eligible but its coarse association was already removed.
        frame.mvpMapPoints[2]=nullptr;frame.mvDynamicExcluded[2]=true;
        frame.mvFeaturePolicy[2]=FeatureUsePolicy::EXCLUDE_PENDING;
        frame.mvpMapPoints[8]=nullptr;
        auto first=ApplyCurrentFineFeaturePolicy(frame);
        Require(first.before==8 && first.after==2 && first.rejectPending==2 &&
            first.rejectDynamic==1 && first.rejectBoundary==1 && first.rejectUnknown==2,
            "First pass must use CURRENT states, not previous states");
        Require(!frame.mvpMapPoints[1] && !frame.mvbOutlier[1] && frame.mvDynamicExcluded[1],
            "S->P must be cleared, without being called an optimization outlier");
        Require(!frame.mvpMapPoints[2] && !frame.mvDynamicExcluded[2] &&
            frame.mvFeaturePolicy[2]==FeatureUsePolicy::USE_NORMAL,"P->S must be eligible for local search");
        // Simulate SearchLocalPoints reintroducing all matches, including P/D.
        for(int i=0;i<10;++i) frame.mvpMapPoints[i]=reinterpret_cast<MapPoint*>(uintptr_t(0x3000+16*i));
        auto second=ApplyCurrentFineFeaturePolicy(frame);
        Require(second.before==10 && second.after==4 && second.excludedFeatures==6 &&
            second.pendingAtOptimizer==0 && second.dynamicAtOptimizer==0 && second.forbiddenAtOptimizer==0,
            "Search reintroduction must not reach fine optimization");
        for(int i=0;i<10;++i) {
            const bool allowed=i==0 || i==2 || i==8 || i==9;
            Require(bool(frame.mvpMapPoints[i])==allowed && frame.mvDynamicExcluded[i]!=allowed,"Fine survivor set");
            Require(frame.mvKeys[i].pt==rawKeys[i].pt,"Keypoint index changed");
        }
        Require(frame.N==10 && frame.mDescriptors.rows==10 &&
            cv::norm(frame.mDescriptors,descriptors,cv::NORM_INF)==0,"Descriptors or N changed");
        auto third=ApplyCurrentFineFeaturePolicy(frame);
        Require(third.before==4 && third.after==4 && third.rejectPending==0 && third.rejectDynamic==0,
            "Fine filtering must be idempotent");
        frame.mvpMapPoints[3]=reinterpret_cast<MapPoint*>(uintptr_t(0x4000));
        bool leaked=false;
        try {VerifyFineOptimizerInputs(frame);} catch(const std::runtime_error&){leaked=true;}
        Require(leaked,"The final optimizer guard must catch any reintroduced D match");
        frame.mvpMapPoints[3]=nullptr;
        // Insufficient allowed support never restores P/D, regardless of how
        // many forbidden features exist. Both pre/post optimization failures block KFs.
        Require(!FinePoseSupported(second.after,30,true) && !FinePolicyAllowsKeyFrame(false,true) &&
            !FinePolicyAllowsKeyFrame(true,true) && !FinePolicyAllowsKeyFrame(false,false),"Failure must block keyframes");
        Require(FinePoseSupported(30,30,true) && !FinePoseSupported(29,30,true) &&
            !FinePoseSupported(100,30,false) && FinePolicyAllowsKeyFrame(true,false),"Fine success gates");
        Require(FineStaticSupportRequired(29,0,30,false)==50 && FineStaticSupportRequired(30,0,30,false)==30 &&
            FineStaticSupportRequired(31,0,30,true)==11 && FineStaticSupportRequired(29,0,30,true)==50,
            "Original RGB-D support thresholds/priority must be preserved");
        for(int i:{1,3,4,5,6,7}) Require(!frame.mvpMapPoints[i],"Failure silently restored forbidden associations");
        frame.N=0;frame.ResetSemanticMetadata();frame.mbSemanticAnnotated=true;
        frame.mvpMapPoints.clear();frame.mvbOutlier.clear();
        Require(ApplyCurrentFineFeaturePolicy(frame).after==0,"Empty feature frame");
        frame.N=1;bool caught=false;
        try {ApplyCurrentFineFeaturePolicy(frame);} catch(const std::runtime_error&){caught=true;}
        Require(caught,"Misaligned metadata must fail explicitly");
        std::cout << "PASS current S/P/D, S->P/P->S timing, new P, boundary/unknown, two-pass reintroduction, "
            "P/D optimizer input zero, unchanged ORB indices/descriptors, idempotence, insufficient support, KF guards, empty/misaligned frames\n";
    } catch(const std::exception& error) {std::cerr << "FAIL " << error.what() << '\n';return 1;}
}
