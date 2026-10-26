#include "FrameSemanticAnnotator.h"
#include "FrameDrawer.h"
#include "Deep/DynamicDetector.h"
#include "Deep/InstanceTrackManager.h"
#include "Pinhole.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace ORB_SLAM3;
static void Require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }

static void Synthetic() {
    cv::Mat gray(64,64,CV_8UC1), depth(64,64,CV_32F,cv::Scalar(1.f));
    cv::randu(gray,0,255);
    depth.colRange(32,64).setTo(2.f);
    ORBextractor extractor(200,1.2f,4,20,7);
    Pinhole camera({100,100,32,32});
    cv::Mat K=(cv::Mat_<float>(3,3)<<100,0,32,0,100,32,0,0,1), dist=cv::Mat::zeros(4,1,CV_32F);
    Frame frame(gray,depth,0,&extractor,nullptr,K,dist,10,4,&camera);
    frame.SetPose(Sophus::SE3f());
    frame.mvKeys.clear();
    for (const cv::Point2f pixel : std::vector<cv::Point2f>{{5,5},{16,20},{30,30},{31,35},{40,40},{64,10},{52,52},{18,25}})
        frame.mvKeys.emplace_back(pixel,7.f);
    frame.N=frame.mvKeys.size();
    frame.mvKeysUn=frame.mvKeys;
    for (auto& key : frame.mvKeysUn) key.pt += cv::Point2f(10,10); // Must NOT be used for mask lookup.
    frame.mDescriptors=cv::Mat(frame.N,32,CV_8UC1);
    cv::randu(frame.mDescriptors,0,255);
    MapPoint sentinel;
    frame.mvpMapPoints.assign(frame.N,nullptr); frame.mvpMapPoints[2]=&sentinel;
    frame.mvbOutlier.assign(frame.N,false); frame.mvbOutlier[1]=true;
    frame.ResetSemanticMetadata();
    const cv::Mat descriptors=frame.mDescriptors.clone();
    const Eigen::Matrix4f pose=frame.GetPose().matrix();
    const auto keys=frame.mvKeys, undistorted=frame.mvKeysUn;
    const auto mapPoints=frame.mvpMapPoints; const auto outliers=frame.mvbOutlier;
    std::vector<YOLOSegInstance> persons(3);
    const cv::Rect boxes[]={{16,16,32,32},{38,38,10,10},{52,52,8,8}};
    for (int i=0;i<3;++i) {
        persons[i].classId=0; persons[i].trackId=i==2?-1:7+i;
        persons[i].boundingBox=boxes[i]; persons[i].mask=cv::Mat::zeros(gray.size(),CV_8UC1);
        persons[i].mask(boxes[i]).setTo(255);
    }
    FeatureSemanticConfig config; config.enable=true;
    AnnotateRGBDFeatures(frame,gray.size(),depth,persons,{{7,TemporalState::DYNAMIC}},config);
    const std::vector<SemanticRegion> expected={SemanticRegion::BACKGROUND,SemanticRegion::PERSON_BOUNDARY,
        SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_BOUNDARY,SemanticRegion::UNKNOWN,
        SemanticRegion::UNKNOWN,SemanticRegion::UNKNOWN,SemanticRegion::PERSON_BOUNDARY};
    Require(frame.mvSemanticRegion==expected,"Semantic mask/depth/overlap/raw-coordinate classification");
    Require(frame.mvInstanceId[2]==0 && frame.mvTrackId[2]==7 && frame.mvTrackId[4]==-1,"Instance ownership");
    UpdateFeatureTemporalStates(frame,{{7,TemporalState::STATIC}});
    Require(frame.mvPreviousB4State[2]==TemporalState::DYNAMIC && frame.mvCurrentB4State[2]==TemporalState::STATIC,
            "Previous B4 state must remain immutable after current refresh");
    Require(cv::norm(descriptors,frame.mDescriptors,cv::NORM_INF)==0 && frame.mvpMapPoints==mapPoints &&
        frame.mvbOutlier==outliers && (pose-frame.GetPose().matrix()).norm()==0,"Annotation modified SLAM state");
    for (int i=0;i<frame.N;++i) {
        Require(frame.mvKeys[i].pt==keys[i].pt && frame.mvKeysUn[i].pt==undistorted[i].pt,"Keypoint order changed");
        Require(frame.mvFeaturePolicy[i]==FeatureUsePolicy::USE_NORMAL && !frame.mvDynamicExcluded[i],"Unexpected exclusion");
    }
    Frame copied(frame), assigned; assigned=frame;
    Require(copied.SemanticMetadataAligned() && assigned.SemanticMetadataAligned() &&
        copied.mvSemanticRegion==frame.mvSemanticRegion && copied.mvPreviousB4State==frame.mvPreviousB4State &&
        assigned.mvTrackId==frame.mvTrackId,"Copy/assignment lost metadata");
    cv::Mat overlay; cv::cvtColor(gray,overlay,cv::COLOR_GRAY2BGR);
    FrameDrawer::DrawSemanticFeatures(overlay,frame.mvKeys,frame.mvSemanticRegion);
    Require(cv::imwrite("feature_semantic_synthetic.png",overlay),"Cannot save Viewer renderer test");
    Frame empty(gray,depth,1,&extractor,nullptr,K,dist,10,4,&camera,nullptr,IMU::Calib(),cv::Mat(gray.size(),CV_8UC1,cv::Scalar(255)));
    AnnotateRGBDFeatures(empty,gray.size(),depth,{}, {},config);
    Require(empty.N==0 && empty.SemanticMetadataAligned(),"Empty feature handling");
    Frame mono(gray,2,&extractor,nullptr,&camera,dist,10,4);
    Require(mono.SemanticMetadataAligned() && !mono.mbSemanticAnnotated,"Monocular default metadata compatibility");
    bool failed=false;
    persons[0].mask=cv::Mat::zeros(3,3,CV_8UC1);
    try { AnnotateRGBDFeatures(frame,gray.size(),depth,persons,{},config); }
    catch(const std::runtime_error&) { failed=true; }
    Require(failed,"Mismatched mask size was not rejected");
    std::cout<<"PASS arrays, raw coordinates, boundaries, depth steps, overlap UNKNOWN, B4 snapshots, copy/assignment, empty features, no SLAM mutation\n";
}

static void RealPreview(char** argv) {
    Frame::mbInitialComputations = true;
    DynamicDetector detector(argv[1]);
    cv::FileStorage fs(argv[1],cv::FileStorage::READ);
    cv::Mat image=cv::imread(argv[2]), rawDepth=cv::imread(argv[3],cv::IMREAD_UNCHANGED), gray,depth;
    Require(!image.empty()&&!rawDepth.empty(),"Cannot read preview RGB/depth");
    const bool rgb=int(fs["Camera.RGB"])!=0;
    detector.Detect(image,rgb,0);
    auto persons=detector.LastRGBDInstances();
    InstanceTrackManager manager(detector.B4()); manager.Associate(persons,image.size());
    cv::cvtColor(image,gray,rgb?cv::COLOR_RGB2GRAY:cv::COLOR_BGR2GRAY);
    rawDepth.convertTo(depth,CV_32F,1.0/double(fs["RGBD.DepthMapFactor"]));
    const float fx=fs["Camera1.fx"],fy=fs["Camera1.fy"],cx=fs["Camera1.cx"],cy=fs["Camera1.cy"];
    Pinhole camera({fx,fy,cx,cy});
    cv::Mat K=(cv::Mat_<float>(3,3)<<fx,0,cx,0,fy,cy,0,0,1),dist=cv::Mat::zeros(4,1,CV_32F);
    ORBextractor extractor{int(fs["ORBextractor.nFeatures"]),float(fs["ORBextractor.scaleFactor"]),
        int(fs["ORBextractor.nLevels"]),int(fs["ORBextractor.iniThFAST"]),int(fs["ORBextractor.minThFAST"])};
    Frame frame(gray,depth,0,&extractor,nullptr,K,dist,fx*float(fs["Stereo.b"]),4,&camera);
    std::unordered_map<int,TemporalState> states;
    for (const auto& person:persons) states[person.trackId]=TemporalState::PENDING;
    AnnotateRGBDFeatures(frame,gray.size(),depth,persons,states,detector.Frontend().featureSemantics);
    FrameDrawer::DrawSemanticFeatures(image,frame.mvKeys,frame.mvSemanticRegion);
    Require(cv::imwrite(argv[4],image),"Cannot save dataset preview");
    std::cout<<"PASS real RGB-D Frame + YOLO preview ORB="<<frame.N<<" output="<<argv[4]<<'\n';
}

int main(int argc,char** argv) {
    try { Synthetic(); if(argc==5) RealPreview(argv); return 0; }
    catch(const std::exception& error) { std::cerr<<"FAIL: "<<error.what()<<'\n'; return 1; }
}
