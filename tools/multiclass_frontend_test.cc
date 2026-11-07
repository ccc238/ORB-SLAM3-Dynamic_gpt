#include "B3SemanticGeometryFusion.h"
#include "B4TemporalStateMachine.h"
#include "Deep/InstanceTrackManager.h"
#include "FrameSemanticAnnotator.h"
#include "FeaturePolicyEvaluator.h"
#include "FineFeaturePolicy.h"
#include "CoarseFeatureFilter.h"
#include "MapAdmission.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Pinhole.h"
#include <iostream>
#include <stdexcept>

using namespace ORB_SLAM3;
static void Check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }

int main() {
    try {
        SemanticClassConfig classes;
        classes.multiClassEnable=true; classes.classIds={0,2,16};
        classes.classNames={"person","car","dog"};
        B3Fusion legacy(true,true), fusion(true,true,classes);
        B2PersonInstance geometry;
        geometry.classId=2;geometry.trackId=10;geometry.candidateCount=100;
        geometry.matchedCount=50;geometry.validGeometryCount=40;
        geometry.geometryDecision=GEOM_MOVING_CANDIDATE;
        SemanticInstance semantic;
        semantic.classId=2;semantic.className="car";semantic.trackId=10;semantic.instanceId=0;
        semantic.boundingBox=cv::Rect(40,40,80,100);
        Check(legacy.Fuse(semantic,&geometry).fusion_category==B3_SEM_STATIC_GEOM_MOVING,
              "Legacy non-person behavior changed");
        auto moving=fusion.Fuse(semantic,&geometry);
        Check(moving.semantic_dynamic_prior && moving.fusion_category==B3_SEM_DYNAMIC_GEOM_MOVING &&
              moving.applied_action=="NONE_EVIDENCE_ONLY", "Car does not drive semantic fusion");
        Check(!fusion.Fuse(semantic,nullptr).fusion_reliable,"Missing B2 became reliable");
        semantic.classId=16;semantic.className="dog";
        Check(fusion.Fuse(semantic,&geometry).semantic_dynamic_prior,"Dog prior missing");
        semantic.classId=56;semantic.className="chair";
        Check(!fusion.Fuse(semantic,&geometry).semantic_dynamic_prior,"Unselected class gained permission");

        B4Config config;config.classes=classes;config.evidenceOnly=false;
        config.evidenceExpiryEnable=false;
        B4TemporalStateMachine b4(config);
        B3FusionResult stationary=moving;
        stationary.instance_id=1;stationary.track_id=11;stationary.semantic_class_id=16;
        stationary.semantic_class_name="dog";stationary.fusion_category=B3_SEM_DYNAMIC_GEOM_STATIC;
        stationary.geometry_state=GEOMETRY_STATE_STATIC_CANDIDATE;
        for(int i=0;i<6;++i) {
            const auto results=b4.Process(i,i/30.0,{moving,stationary},400);
            Check(results.size()==2,"Instance aggregation mixed classes");
            if(i==2) Check(results[0].state==B4MotionState::DYNAMIC,"Car P->D failed");
            if(i==5) Check(results[1].state==B4MotionState::STATIC,"Dog P->S failed");
        }
        auto forged=moving;forged.track_id=12;forged.instance_id=2;forged.semantic_class_id=56;
        const auto rejected=b4.Process(6,6/30.0,{forged},400);
        Check(rejected[0].state==B4MotionState::PENDING && rejected[0].nDynamic==0,
              "Unselected class advanced B4");

        cv::Mat gray(240,320,CV_8UC1),depth(240,320,CV_32F,cv::Scalar(2.f));
        cv::randu(gray,0,255);
        Pinhole camera({250,250,160,120});
        cv::Mat K=(cv::Mat_<float>(3,3)<<250,0,160,0,250,120,0,0,1);
        cv::Mat distortion=cv::Mat::zeros(4,1,CV_32F);
        ORBextractor extractor(300,1.2f,4,20,7);
        Frame frame(gray,depth,0,&extractor,nullptr,K,distortion,25,4,&camera);
        frame.SetPose(Sophus::SE3f());Check(frame.N>6,"No synthetic ORB features");
        std::vector<YOLOSegInstance> instances(2);
        for(int i=0;i<2;++i) {
            auto& in=instances[i];in.classId=i?16:2;in.trackId=10+i;in.confidence=.9f;
            in.boundingBox=cv::Rect(40+140*i,40,80,100);
            in.mask=cv::Mat::zeros(gray.size(),CV_8U);in.mask(in.boundingBox).setTo(255);
        }
        const cv::Point2f pixels[]={{10,10},{70,70},{210,70},{40,70},{180,70},{100,180}};
        for(int i=0;i<frame.N;++i) frame.mvKeys[i].pt=frame.mvKeysUn[i].pt=pixels[i%6];
        const auto keys=frame.mvKeys;const auto descriptors=frame.mDescriptors.clone();
        FeatureSemanticConfig annotation;annotation.enable=true;annotation.classes=classes;
        AnnotateRGBDFeatures(frame,gray.size(),depth,instances,
            {{10,TemporalState::PENDING},{11,TemporalState::STATIC}},annotation);
        Check(frame.mvSemanticRegion[1]==SemanticRegion::PERSON_INTERIOR && frame.mvTrackId[1]==10 &&
              frame.mvSemanticRegion[2]==SemanticRegion::PERSON_INTERIOR && frame.mvTrackId[2]==11,
              "Non-person interior remained background");
        const auto coarse=ComputeFeaturePolicies(frame,FeaturePolicyPhase::COARSE);
        Check(coarse[1]==FeatureUsePolicy::EXCLUDE_PENDING && coarse[2]==FeatureUsePolicy::USE_NORMAL,
              "Coarse permissions ignore managed class state");
        MapPoint sentinel;frame.mvpMapPoints.assign(frame.N,&sentinel);
        const auto coarseReport=OptimizeWithCoarseFeatureFilter(frame,coarse,1,1,.1f,1,[&](){
            Check(!frame.mvpMapPoints[1] && frame.mvpMapPoints[2] && !frame.mvpMapPoints[3],
                  "Managed P/boundary reaches coarse optimization");
            int count=0;for(auto* point:frame.mvpMapPoints) count+=point!=nullptr;
            return count;
        },[](){return 1;});
        Check(coarseReport.applied && !coarseReport.fallback,"Coarse multi-class action not applied");
        UpdateFeatureTemporalStates(frame,{{10,TemporalState::DYNAMIC},{11,TemporalState::STATIC}});
        frame.mvpMapPoints.assign(frame.N,&sentinel); // Local search may reintroduce associations.
        const auto fine=ApplyCurrentFineFeaturePolicy(frame);
        Check(fine.dynamicAtOptimizer==0 && !frame.mvpMapPoints[1] && frame.mvpMapPoints[2] &&
              !frame.mvpMapPoints[3],"Managed D/boundary reaches fine pose optimization");
        PrepareFrameMapAdmission(frame,"");
        Check(!frame.mvAllowMapCreation[1] && frame.mvAllowMapCreation[2] && !frame.mvAllowMapCreation[3],
              "Non-person P/D map admission leaked");
        Map map(0);KeyFrameDatabase db;KeyFrame keyframe(frame,&map,&db);
        Check(!keyframe.AllowsMapCreation(1) && keyframe.AllowsMapCreation(2) &&
              keyframe.MapAdmissionMetadataAligned(),"KeyFrame lost multi-class gate");
        Frame pending(frame);pending.mvCurrentB4State[2]=TemporalState::PENDING;
        PrepareFrameMapAdmission(pending,"");
        Check(!pending.mvAllowMapCreation[2],"Managed P admitted to new map observations");
        for(int i=0;i<frame.N;++i) Check(frame.mvKeys[i].pt==keys[i].pt,"Feature index changed");
        Check(cv::norm(frame.mDescriptors,descriptors,cv::NORM_INF)==0,"Descriptors changed");

        annotation.classes.multiClassEnable=false;
        AnnotateRGBDFeatures(frame,gray.size(),depth,instances,{},annotation);
        Check(frame.mvSemanticRegion[1]==SemanticRegion::BACKGROUND,"Legacy annotation changed");
        B2Config b2Config;b2Config.debugEnable=false;b2Config.saveCsv=false;
        B2MotionEvidence b2(b2Config);
        const auto candidates=b2.ExtractCandidates(gray,depth,instances,0,extractor);
        Check(candidates.persons.size()==2 && candidates.persons[0].classId==2 &&
              candidates.persons[1].classId==16 && !candidates.persons[0].features.empty(),
              "B2 dropped selected non-person instances");
        B4Config trackingConfig;InstanceAssociationConfig identity;identity.mode=3;
        InstanceTrackManager manager(trackingConfig,identity);
        manager.Associate(instances,gray.size(),0,0,gray,depth);
        const int carId=instances[0].trackId,dogId=instances[1].trackId;
        manager.Associate(instances,gray.size(),1,1/30.0,gray,depth);
        Check(instances[0].trackId==carId && instances[1].trackId==dogId && carId!=dogId,
              "Multi-class IDs unstable");
        instances.resize(1);instances[0].classId=16;
        manager.Associate(instances,gray.size(),2,2/30.0,gray,depth);
        Check(instances[0].trackId!=carId,"Identity inherited across classes");
        std::cout<<"PASS multi-class B2/B3/B4, per-instance IDs, legacy defaults, coarse policy, "
            "fine optimizer exclusion, indexed KeyFrame map admission and descriptor alignment\n";
    } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
