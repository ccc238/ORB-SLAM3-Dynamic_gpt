#include "Deep/InstanceTrackManager.h"
#include "B2MotionEvidence.h"
#include "B4TemporalStateMachine.h"
#include "Frame.h"
#include "ORBextractor.h"
#include "FrameSemanticAnnotator.h"
#include "FineFeaturePolicy.h"
#include "FeaturePolicyEvaluator.h"
#include "MapAdmission.h"
#include "Map.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Pinhole.h"
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3;
void Check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
YOLOSegInstance Person() {YOLOSegInstance p;p.classId=0;p.confidence=.9;p.boundingBox=cv::Rect(30,30,100,160);
    p.mask=cv::Mat::zeros(240,320,CV_8UC1);p.mask(p.boundingBox).setTo(255);return p;}
B3FusionResult Static(int id){B3FusionResult b;b.track_id=id;b.instance_id=0;b.fusion_reliable=true;b.fusion_category=B3_SEM_DYNAMIC_GEOM_STATIC;return b;}
B2CandidateFrame Measurement(long frame,const InstanceAssociationResult& association) {
    B2CandidateFrame f;f.frameId=frame;f.timestamp=frame/30.;f.poseValid=true;f.poseSource="coarse";f.imageSize={320,240};f.Tcw=Sophus::SE3f();
    B2PersonInstance p;p.trackId=association.trackId;p.instanceId=0;p.association=association;p.mask=Person().mask;
    for(int i=0;i<2;++i){B2CandidateFeature k;k.candidateId=i;k.keypoint=cv::KeyPoint(70+i*20,90,8);
        k.descriptor=cv::Mat(1,32,CV_8U,cv::Scalar(i?255:0)).clone();k.depthMeters=2;k.depthValid=true;p.features.push_back(k);}
    f.persons.push_back(p);return f;
}
int main(){try {
    InstanceAssociationConfig config;config.mode=3;B4Config b4;
    InstanceTrackManager manager(b4,config);B4TemporalStateMachine states(b4);
    std::vector<YOLOSegInstance> persons;
    for(int i=0;i<6;++i) {
        persons={Person()};manager.Associate(persons,{320,240},i,i/30.);
        // Set up a known old S independently of the first-frame measurement gate.
        states.Process(i,i/30.,{Static(persons[0].trackId)},400);
    }
    const int id=persons[0].trackId;B4MotionState state;
    Check(states.GetState(id,state)&&state==B4MotionState::STATIC,"old S setup");
    persons.clear();manager.Associate(persons,{320,240},6,6/30.);states.Process(6,6/30.,{},400);
    persons={Person()};manager.Associate(persons,{320,240},7,7/30.);
    const auto reacquired=persons[0].association;
    Check(reacquired.trackId==id&&reacquired.resetEvidence&&!reacquired.allowStateInheritance,"reacquisition contract");
    B4MotionState coarse=B4MotionState::PENDING;
    if(reacquired.allowStateInheritance)states.GetUsableState(id,7/30.,coarse);
    Check(coarse==B4MotionState::PENDING,"old S authorized coarse use");
    std::unordered_map<int,InstanceAssociationResult> identities{{id,reacquired}};
    auto result=states.Process(7,7/30.,{Static(id)},400,&identities).at(0);
    Check(result.state==B4MotionState::PENDING&&result.nStatic==0&&result.nDynamic==0&&!result.hasReliableHistory,
        "untrusted fabricated reliable B3 evidence advanced B4");
    Check(result.reasonCode=="IDENTITY_UNTRUSTED_FREEZE","identity reason lost");
    B2Config cfg;cfg.enable=true;cfg.saveCsv=false;cfg.debugEnable=false;cfg.geometryDecisionEnable=true;
    B2MotionEvidence evidence(cfg);cv::Mat K=(cv::Mat_<float>(3,3)<<250,0,160,0,250,120,0,0,1);evidence.SetCameraCalibration(K);
    auto previous=Measurement(5,reacquired),current=Measurement(7,reacquired);
    auto b2=evidence.Process(&previous,current,Sophus::SE3f(),true);
    Check(b2.persons[0].matchedCount==0&&b2.persons[0].geometryDecision==GEOM_UNRELIABLE&&
        b2.persons[0].geometryFailureReason==IDENTITY_ASSOCIATION_UNTRUSTED,"B2 paired a reacquisition gap");
    persons={Person()};manager.Associate(persons,{320,240},8,8/30.);
    current=Measurement(8,persons[0].association);
    b2=evidence.Process(&previous,current,Sophus::SE3f(),true);
    Check(b2.persons[0].matchedCount==0&&b2.persons[0].geometryFailureReason==IDENTITY_HISTORY_MISMATCH,"B2 mixed historical measurements and adjacent pose");
    auto baseline=Measurement(7,reacquired);
    b2=evidence.Process(&baseline,current,Sophus::SE3f(),true);
    Check(b2.persons[0].matchedCount==2,"new measurement baseline cannot pair with next confirmed frame");
    cv::Mat gray(240,320,CV_8UC1),depth(240,320,CV_32F,cv::Scalar(2));cv::randu(gray,0,255);
    ORBextractor extractor(100,1.2f,4,20,7);Pinhole camera({250,250,160,120});cv::Mat dist=cv::Mat::zeros(4,1,CV_32F);
    Frame frame(gray,depth,0,&extractor,nullptr,K,dist,25,4,&camera);frame.SetPose(Sophus::SE3f());
    Check(frame.N>0,"synthetic Frame missing");frame.mbSemanticAnnotated=true;
    frame.mvSemanticRegion.assign(frame.N,SemanticRegion::BACKGROUND);frame.mvSemanticRegion[0]=SemanticRegion::PERSON_INTERIOR;
    frame.mvInstanceId[0]=0;frame.mvTrackId[0]=id;frame.mvPreviousB4State[0]=frame.mvCurrentB4State[0]=TemporalState::PENDING;
    auto coarsePolicy=ComputeFeaturePolicies(frame,FeaturePolicyPhase::COARSE);
    Check(!FeaturePolicyAllows(coarsePolicy[0],FeaturePolicyPhase::COARSE),"identity P coarse policy");
    MapPoint sentinel;frame.mvpMapPoints[0]=&sentinel;ApplyCurrentFineFeaturePolicy(frame);
    Check(!frame.mvpMapPoints[0],"identity P reached fine optimizer");PrepareFrameMapAdmission(frame,"");
    Map map(0);KeyFrameDatabase db;KeyFrame keyframe(frame,&map,&db);
    Check(!keyframe.AllowsMapCreation(0),"identity P reached map admission");
    evidence.Reset();states.Reset();manager.Reset();persons={Person()};manager.Associate(persons,{320,240},0,0);
    Check(!evidence.HasPreviousFrame()&&!states.GetState(id,state)&&persons[0].trackId>id,"sequence reset leaked history/reused ID");
    std::cout<<"PASS identity -> B2/B4/coarse/fine/KeyFrame admission and sequence reset\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
