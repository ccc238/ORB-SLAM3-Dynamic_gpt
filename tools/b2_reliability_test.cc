#include "B2MotionEvidence.h"
#include "B3SemanticGeometryFusion.h"
#include "B4TemporalStateMachine.h"
#include "CoarsePoseQuality.h"
#include "DepthNeighborhood.h"
#include "ORBextractor.h"
#include <iostream>
#include <set>
#include <limits>
using namespace ORB_SLAM3;
static void Check(bool v,const char* msg){if(!v)throw std::runtime_error(msg);}

static B2CandidateFrame FrameInput(int id,bool moving) {
    B2CandidateFrame f;f.frameId=id;f.timestamp=id/30.;f.imageSize={320,240};
    f.poseValid=true;f.poseSource="coarse";f.Tcw=Sophus::SE3f();
    B2PersonInstance p;p.instanceId=0;p.trackId=10;p.classId=0;
    p.boundingBox={80,60,160,120};
    p.mask=cv::Mat::zeros(f.imageSize,CV_8U);p.mask(p.boundingBox).setTo(255);
    p.association.evaluated=true;p.association.allowStateInheritance=true;p.association.allowGeometryPair=true;
    p.association.associationStatus=AssociationStatus::CONTINUOUS_CONFIRMED;
    p.association.trackId=10;p.association.instanceId=0;
    p.association.previousMatchedFrame=id-1;p.association.previousMatchedTimestamp=(id-1)/30.;
    cv::RNG random(1234);
    for(int i=0;i<12;++i) {
        B2CandidateFeature feature;feature.candidateId=i;feature.instanceId=0;
        feature.keypoint=cv::KeyPoint(100+i*8+(moving?5:0),100,7);
        feature.depthValid=true;feature.depthStatus=DEPTH_VALID;feature.depthMeters=2;
        feature.depthQualityChecked=true;feature.depthFailureReason="OK";
        feature.descriptor=cv::Mat(1,32,CV_8U);random.fill(feature.descriptor,cv::RNG::UNIFORM,0,256);
        p.features.push_back(feature);
    }
    f.persons.push_back(p);
    for(int i=0;i<60;++i) {
        B2BackgroundObservation o;o.mapPointId=i;o.depthMeters=2;
        o.pixel={float(5+(i%10)*4),float(5+(i/10)*4)};f.backgroundObservations.push_back(o);
    }
    return f;
}
int main() {
    try {
        B2Config config;config.saveCsv=false;config.debugEnable=false;
        config.geometryDecisionEnable=true;config.minMatchedFeatures=10;
        config.reliability.enable=true;ValidateB2Reliability(config.reliability);
        cv::Mat depth(20,20,CV_32F,cv::Scalar(2)),mask(20,20,CV_8U,cv::Scalar(255));
        auto sample=[&](){return InspectDepthNeighborhood(depth,{10,10},mask,config.reliability,.3,5);};
        Check(sample().valid,"Flat depth rejected");
        depth.colRange(11,20).setTo(3);
        Check(sample().edge && !sample().valid,"Static depth discontinuity not rejected");
        depth.setTo(0);Check(std::string(sample().reason)=="DEPTH_MISSING","Missing depth reason");
        depth.at<float>(10,10)=2;
        Check(std::string(sample().reason)=="DEPTH_NEIGHBORHOOD_INSUFFICIENT","Sparse depth accepted");
        depth.setTo(2);depth.at<float>(10,10)=std::numeric_limits<float>::quiet_NaN();
        Check(sample().valid,"Single hole should allow sufficient valid neighbors");
        depth.setTo(2);mask.colRange(11,20).setTo(0);depth.colRange(11,20).setTo(4);
        Check(sample().valid,"Outside-instance depths contaminated sampling");
        B2ReliabilityConfig invalid=config.reliability;invalid.minDepthSamples=10;
        bool threw=false;try{ValidateB2Reliability(invalid);}catch(const std::runtime_error&){threw=true;}
        Check(threw,"Invalid quality parameter accepted");

        B2MotionEvidence processor(config);
        cv::Mat K=(cv::Mat_<float>(3,3)<<100,0,160,0,100,120,0,0,1);
        processor.SetCameraCalibration(K);
        const auto previous=FrameInput(0,false);auto current=FrameInput(1,true);
        auto process=[&](const B2CandidateFrame& c,bool reliable=true){
            return processor.Process(&previous,c,Sophus::SE3f(),reliable).persons.front();};
        auto result=process(current);
        Check(result.geometryDecision==GEOM_MOVING_CANDIDATE && result.validGeometryCount==12 &&
            result.rawR3DMedian>.09,"Trustworthy large motion was filtered out");
        const auto staticResult=process(FrameInput(1,false));
        Check(staticResult.geometryDecision==GEOM_STATIC_CANDIDATE,"Valid static evidence rejected");
        auto edge=current;edge.persons[0].features[0].nearDepthEdge=true;
        auto edgeResult=process(edge);
        Check(edgeResult.validGeometryCount==11 && edgeResult.depthEdgeCount==1,"Depth edge entered geometry");
        auto sparse=current;
        for(auto& f:sparse.persons[0].features){f.depthValid=false;f.depthFailureReason="DEPTH_NEIGHBORHOOD_INSUFFICIENT";}
        auto sparseResult=process(sparse);
        Check(sparseResult.geometryDecision==GEOM_UNRELIABLE && sparseResult.depthInvalidCount==12 &&
            sparseResult.geometryFailureReason==INSUFFICIENT_VALID_DEPTH,"Invalid depth advanced evidence");
        auto bad=current;bad.persons[0].association.allowGeometryPair=false;
        Check(process(bad).geometryDecision==GEOM_UNRELIABLE,"Untrusted identity accepted");
        bad=current;bad.persons[0].association.previousMatchedTimestamp=123;
        Check(process(bad).geometryFailureReason==IDENTITY_HISTORY_MISMATCH,"Historical timestamp mismatch accepted");
        bad=current;bad.persons[0].association.associationStatus=AssociationStatus::REACQUIRED;
        Check(process(bad).geometryFailureReason==IDENTITY_HISTORY_MISMATCH,"Reacquisition used an adjacent pose pair");
        bad=current;bad.timestamp=previous.timestamp;
        Check(process(bad).geometryDecision==GEOM_UNRELIABLE,"Repeated acquisition timestamp accepted");
        bad=current;bad.persons[0].classId=2;
        Check(process(bad).geometryFailureReason==IDENTITY_HISTORY_MISMATCH,"Different class inherited geometry");
        bad=current;bad.persons[0].features[1].candidateId=0;
        Check(process(bad).geometryFailureReason==DUPLICATE_CANDIDATE_ID,"Duplicate feature ID accepted");
        // MatExpr assignment may reuse shared cv::Mat storage. Allocate a
        // separate matrix so this corruption fixture cannot alter `current`.
        bad=current;for(auto& f:bad.persons[0].features)f.descriptor=cv::Mat(1,32,CV_8U,cv::Scalar(0)).clone();
        Check(process(bad).matchedCount==0,"Bad descriptor matches accepted");
        bad=current;bad.persons[0].features.resize(5);
        Check(process(bad).geometryDecision==GEOM_UNRELIABLE,"Insufficient valid samples accepted");
        bad=current;bad.persons[0].features[0].depthMeters=std::numeric_limits<float>::quiet_NaN();
        const auto numeric=process(bad);
        if(numeric.geometryFailureReason!=NUMERIC_INVALID)
            std::cerr<<"numeric diagnostic decision="<<numeric.geometryDecision<<" reason="<<numeric.geometryFailureReason
                     <<" numeric_count="<<numeric.numericInvalidCount<<" valid="<<numeric.validGeometryCount<<'\n';
        Check(numeric.geometryFailureReason==NUMERIC_INVALID,"NaN numeric rejection reason missing");
        Check(process(current,false).geometryFailureReason==POSE_WEAK,"Weak pose advanced evidence");
        auto nonMutual=config;nonMutual.useMutualMatch=false;
        B2MotionEvidence unique(nonMutual);unique.SetCameraCalibration(K);
        auto dupPrevious=previous;
        dupPrevious.persons[0].features[1].descriptor=dupPrevious.persons[0].features[0].descriptor;
        auto uniqueResult=unique.Process(&dupPrevious,current,Sophus::SE3f(),true).persons.front();
        std::set<int> targets;
        for(const auto& match:uniqueResult.matches)if(match.descriptorMatched)
            Check(targets.insert(match.currentCandidateId).second,"One current feature reused");
        Check(uniqueResult.duplicateMatchCount==1,"Nonmutual duplicate target not diagnosed");

        auto off=config;off.reliability.enable=false;
        B2MotionEvidence legacy(off);legacy.SetCameraCalibration(K);
        Check(legacy.Process(&previous,edge,Sophus::SE3f(),true).persons[0].validGeometryCount==12,
            "Disabled enhancement changed legacy depth-edge path");
        cv::Mat gray(240,320,CV_8U),holes(240,320,CV_32F,cv::Scalar(0));cv::randu(gray,0,255);
        YOLOSegInstance instance;instance.classId=0;instance.mask=previous.persons[0].mask;
        instance.boundingBox=previous.persons[0].boundingBox;instance.trackId=10;
        ORBextractor extractor(300,1.2f,4,20,7);
        const auto extracted=processor.ExtractCandidates(gray,holes,{instance},0,extractor);
        Check(!extracted.persons[0].features.empty(),"Extraction test has no features");
        for(const auto& f:extracted.persons[0].features)
            Check(f.depthQualityChecked && !f.depthValid && f.depthFailureReason=="DEPTH_MISSING",
                "Depth quality not connected to candidate extraction");

        std::vector<CoarseBackgroundSample> samples;
        for(int i=0;i<36;++i)samples.push_back({{float(20+(i%4)*80),float(20+((i/4)%3)*80)},.5f});
        auto measure=[&](std::vector<CoarseBackgroundSample> s){
            CoarsePoseQuality q;q.reliable=true;MeasureBackgroundQuality(q,s,40,0,{320,240},config.reliability);
            return EvaluateBackgroundQuality(q,config.reliability);};
        Check(measure(samples).reliable,"Good coarse background rejected");
        auto poor=samples;for(auto& s:poor)s.reprojection=4;
        Check(measure(poor).reason=="BACKGROUND_REPROJECTION_HIGH","High background reprojection accepted");
        poor=samples;for(auto& s:poor)s.rawPixel={10,10};
        Check(measure(poor).reason=="BACKGROUND_COVERAGE_LOW","Concentrated background accepted");
        poor.resize(5);Check(measure(poor).reason=="INSUFFICIENT_BACKGROUND_POSE_SAMPLES","Sparse pose reference accepted");
        poor=samples;poor[0].reprojection=std::numeric_limits<float>::infinity();
        Check(measure(poor).reason=="BACKGROUND_NUMERIC_INVALID","Numeric pose error accepted");

        B4Config b4config;B4TemporalStateMachine b4(b4config);B3Fusion fusion(true,true);
        SemanticInstance sem;sem.classId=0;sem.className="person";sem.instanceId=0;sem.trackId=10;
        auto reliableStatic=fusion.Fuse(sem,&staticResult);
        for(int i=0;i<6;++i)b4.Process(i,i/30.,{reliableStatic},400);
        auto unreliable=fusion.Fuse(sem,&sparseResult);
        auto frozen=b4.Process(6,.18,{unreliable},400).front();
        Check(frozen.state==B4MotionState::STATIC && frozen.nStatic==6,"Weak evidence changed B4 count");
        auto expired=b4.Process(7,.6,{unreliable},400).front();
        Check(expired.state==B4MotionState::PENDING && expired.nStatic==0 && expired.staticStateExpired,
            "Unreliable evidence bypassed static expiry");
        std::cout<<"PASS depth edges/holes/insufficient/masked sampling, matching/duplicates/identity/history, "
            "coarse reprojection/coverage/numerics, reliable large motion, B4 freeze+expiry, legacy off path\n";
    }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
