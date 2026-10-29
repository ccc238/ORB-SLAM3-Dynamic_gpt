#include "B4TemporalStateMachine.h"
#include "FineFeaturePolicy.h"
#include "FeaturePolicyEvaluator.h"
#include "MapAdmission.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapPoint.h"
#include "Map.h"
#include "Pinhole.h"
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3;
namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template<class F> void Reject(F action, const char* message) {
    bool rejected = false;
    try { action(); } catch (const std::runtime_error&) { rejected = true; }
    Check(rejected, message);
}
B3FusionResult Evidence(int kind, int id=0) {
    B3FusionResult result;
    result.track_id=id; result.instance_id=id;
    result.fusion_reliable=kind != 0;
    result.fusion_category=kind == 1 ? B3_SEM_DYNAMIC_GEOM_STATIC :
        kind == 2 ? B3_SEM_DYNAMIC_GEOM_MOVING : B3_FUSION_UNRELIABLE;
    return result;
}
B4Config Config() { B4Config c; c.evidenceExpiryEnable=true; return c; }
B4StateResult Step(B4TemporalStateMachine& m, long id, double time, int kind) {
    return m.Process(id,time,{Evidence(kind)},800).at(0);
}
double MakeStatic(B4TemporalStateMachine& m) {
    for (int i=0;i<6;++i) Check(Step(m,i,i/30.0,1).nStatic==i+1,"static accumulation");
    return 5/30.0;
}
double MakeDynamic(B4TemporalStateMachine& m) {
    for (int i=0;i<3;++i) Step(m,i,i/30.0,2);
    return 2/30.0;
}
void LegacyAndShortFreeze() {
    B4TemporalStateMachine legacy(B4Config{});
    Step(legacy,0,0,2); Step(legacy,1,1/30.0,2);
    auto row=Step(legacy,2,10,0);
    Check(row.nDynamic==2 && !row.counterExpired,"disabled legacy freeze");
    Check(Step(legacy,3,20,2).state==B4MotionState::DYNAMIC,"disabled old counters survive");
    B4TemporalStateMachine m(Config());
    auto initial=Step(m,0,0,0);
    Check(!initial.hasReliableHistory && std::isnan(initial.evidenceAgeSeconds) &&
        initial.expiryReason=="NO_RELIABLE_HISTORY","no fabricated timestamp/history");
    Step(m,1,1/30.0,2);
    auto frozen=Step(m,2,2/30.0,0);
    Check(frozen.nDynamic==1 && !frozen.counterExpired,"short freeze retains counters");
    auto expired=Step(m,3,0.25,2);
    Check(expired.state==B4MotionState::PENDING && expired.nDynamic==1 &&
        expired.counterExpired && expired.countersReset,"unreliable must not refresh reliable time");
}
void CounterAndRecoveryExpiry() {
    B4TemporalStateMachine m(Config());
    Step(m,0,0,2); Step(m,1,1/30.0,2);
    auto row=Step(m,2,0.3,2);
    Check(row.counterExpired && row.nDynamic==1 && row.state==B4MotionState::PENDING,
        "two old moving observations cannot complete P->D");
    Check(row.expiryReason=="COUNTER_GAP_EXPIRED" && row.reasonCode=="P_ACCUMULATE_MOVING",
        "expiry and current update reasons must coexist");
    B4TemporalStateMachine d(Config());
    MakeDynamic(d);
    for(int i=3;i<7;++i) Step(d,i,i/30.0,1);
    row=Step(d,7,0.24,0);
    Check(row.state==B4MotionState::DYNAMIC && row.nRecovery==4 && !row.counterExpired,"short D freeze retains recovery");
    row=Step(d,8,0.5,0);
    Check(row.state==B4MotionState::DYNAMIC && row.nRecovery==0 && row.counterExpired,
        "D expiry clears recovery, never permits recovery by timeout");
    row=Step(d,9,0.6,1);
    Check(row.state==B4MotionState::DYNAMIC && row.nRecovery==1,"D recovery restarts at one");
}
void StaticExpiryAndReadOnly() {
    B4TemporalStateMachine m(Config());
    const double last=MakeStatic(m);
    B4MotionState state;
    Check(m.GetUsableState(0,last+0.31,state) && state==B4MotionState::PENDING,"coarse rejects stale S");
    Check(m.GetState(0,state) && state==B4MotionState::STATIC,"query must not mutate live state");
    auto row=Step(m,6,last+0.1,0);
    Check(row.nStatic==6 && row.ageFrames==7 && !row.counterExpired,"future query must not advance counters/clock/age");
    row=Step(m,7,last+0.31,1);
    Check(row.previousState==B4MotionState::STATIC && row.stateAfterExpiry==B4MotionState::PENDING &&
        row.state==B4MotionState::PENDING && row.staticStateExpired && row.counterExpired &&
        row.nStatic==1 && row.reasonCode=="P_ACCUMULATE_STATIC","expired S cannot recover with one static observation");
    Check(std::abs(row.evidenceAgeSeconds-0.31)<1e-12,"age is measured before current reliable refresh");
    row=Step(m,8,last+0.32,0);
    Check(row.nStatic==1 && std::abs(row.evidenceAgeSeconds-0.01)<1e-12,"reliable evidence refreshes acquisition time");
}
void MissingAndTTL() {
    B4TemporalStateMachine m(Config()); const double last=MakeStatic(m);
    B4MotionState query;
    for(int i=0;i<3;++i) Check(m.GetUsableState(0,last+1,query) && query==B4MotionState::PENDING,"future query");
    for(int i=1;i<=16;++i) {
        auto row=m.Process(5+i,last+i/30.0,{},800).at(0);
        Check(row.missedFrames==i,"expiry must not advance TTL twice");
        if(i==10) Check(row.staticStateExpired && row.previousState==B4MotionState::STATIC &&
            row.state==B4MotionState::PENDING,"missing S expires within identity TTL");
        Check((row.reasonCode=="TRACK_TTL_EXPIRED")== (i==16),"identity expires only at miss 16");
    }
    Check(m.Process(22,last+17/30.0,{},800).empty(),"terminated track removed");
    B4TemporalStateMachine restore(Config()); const double t=MakeStatic(restore);
    for(int i=1;i<=10;++i) restore.Process(5+i,t+i/30.0,{},800);
    auto row=Step(restore,16,t+11/30.0,1);
    Check(row.trackId==0 && row.missedFrames==0 && row.state==B4MotionState::PENDING && row.nStatic==1,
        "same identity within TTL must not inherit expired S/counters");
}
void BoundariesAndInvalidTime() {
    B4TemporalStateMachine m(Config()); Step(m,0,0,2);
    auto row=Step(m,1,0.20,0);
    Check(!row.counterExpired && row.nDynamic==1,"equal gap is valid");
    row=Step(m,2,std::nextafter(0.20,1.0),0);
    Check(row.counterExpired && row.nDynamic==0,"strictly greater gap expires");
    B4TemporalStateMachine s(Config()); const double t=MakeStatic(s);
    row=Step(s,6,t+0.30,0);
    Check(!row.staticStateExpired && row.state==B4MotionState::STATIC,"equal S age remains valid");
    B4MotionState state;
    Check(s.GetUsableState(0,std::nextafter(t+0.30,1.0),state) && state==B4MotionState::PENDING,
        "strictly greater S age expires in coarse query");
    Reject([&]{Step(m,3,0.1,2);},"rollback must be rejected");
    Reject([&]{Step(m,3,std::numeric_limits<double>::quiet_NaN(),2);},"NaN time");
    Reject([&]{Step(m,3,std::numeric_limits<double>::infinity(),2);},"infinite time");
    Reject([&]{Step(m,2,std::nextafter(0.20,1.0),2);},"duplicate frame guard");
    Reject([&]{Step(m,3,std::nextafter(0.20,1.0),2);},"same acquisition time cannot add evidence");
    Reject([&]{m.Process(3,0.4,{Evidence(2),Evidence(2)},800);},"duplicate track guard");
    row=Step(m,3,0.4,2);
    Check(row.nDynamic==1 && row.ageFrames==4,"invalid inputs must be atomic");
    m.Reset(); row=Step(m,0,0,2);
    Check(row.nDynamic==1 && !row.hasReliableHistory && row.ageFrames==1,"explicit reset starts fresh sequence");
    Reject([&]{Step(m,-1,0.1,2);},"frame ID reset requires explicit Reset");
    for(double invalid:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        auto c=Config();c.maxEvidenceGapSeconds=invalid;
        Reject([&]{B4TemporalStateMachine bad(c);},"invalid gap");
        c=Config();c.maxStaticAgeSeconds=invalid;
        Reject([&]{B4TemporalStateMachine bad(c);},"invalid S age");
    }
    auto c=Config();c.maxEvidenceGapSeconds=0.4;
    Reject([&]{B4TemporalStateMachine bad(c);},"gap must not exceed S age");
    c.maxEvidenceGapSeconds=c.maxStaticAgeSeconds=0.25; B4TemporalStateMachine equal(c);
    // A real-world-sized acquisition epoch must use the same deadline rule.
    B4TemporalStateMachine epoch(Config()); const double stamp=1341846226.81792;
    Step(epoch,0,stamp,2);
    Check(!Step(epoch,1,stamp+0.20,0).counterExpired,"epoch equality");
    Check(Step(epoch,2,std::nextafter(stamp+0.20,stamp+1),0).counterExpired,"epoch strict greater");
}
void FineAndMapAdmission() {
    B4TemporalStateMachine m(Config());const double t=MakeStatic(m);
    auto expired=Step(m,6,t+0.31,0);
    cv::Mat gray(240,320,CV_8UC1),depth(240,320,CV_32F,cv::Scalar(2));cv::randu(gray,0,255);
    ORBextractor extractor(100,1.2f,4,20,7);Pinhole camera({250,250,160,120});
    cv::Mat K=(cv::Mat_<float>(3,3)<<250,0,160,0,250,120,0,0,1),dist=cv::Mat::zeros(4,1,CV_32F);
    Frame frame(gray,depth,0,&extractor,nullptr,K,dist,25,4,&camera);frame.SetPose(Sophus::SE3f());
    Check(frame.N>0,"synthetic Frame features");frame.mbSemanticAnnotated=true;
    frame.mvSemanticRegion.assign(frame.N,SemanticRegion::BACKGROUND);
    frame.mvSemanticRegion[0]=SemanticRegion::PERSON_INTERIOR;frame.mvInstanceId[0]=0;frame.mvTrackId[0]=0;
    Check(expired.state==B4MotionState::PENDING,"expiry result");
    frame.mvPreviousB4State[0]=frame.mvCurrentB4State[0]=TemporalState::PENDING;
    const auto coarse=ComputeFeaturePolicies(frame,FeaturePolicyPhase::COARSE);
    Check(coarse[0]==FeatureUsePolicy::EXCLUDE_PENDING &&
        !FeaturePolicyAllows(coarse[0],FeaturePolicyPhase::COARSE),"expired P coarse proposal must reject");
    MapPoint sentinel;frame.mvpMapPoints[0]=&sentinel;
    auto fine=ApplyCurrentFineFeaturePolicy(frame);
    Check(!frame.mvpMapPoints[0] && fine.rejectPending==1 && frame.mvDynamicExcluded[0],"expired P cannot reach fine optimizer");
    PrepareFrameMapAdmission(frame,"");Check(!frame.mvAllowMapCreation[0],"expired P cannot create maps");
    Map map(0);KeyFrameDatabase db;KeyFrame keyframe(frame,&map,&db);
    Check(!keyframe.AllowsMapCreation(0),"KeyFrame snapshot preserves expired P denial");
    Reject([&]{keyframe.AddMapPoint(&sentinel,0);},"expired P cannot add indexed observation");
}
}
int main() {
    try {
        LegacyAndShortFreeze();CounterAndRecoveryExpiry();StaticExpiryAndReadOnly();
        MissingAndTTL();BoundariesAndInvalidTime();FineAndMapAdmission();
        std::cout<<"PASS B4 expiry: legacy, gaps, S/P/D, readonly coarse query, TTL, time/reset, reasons, fine/map admission\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
