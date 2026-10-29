#include "Deep/InstanceTrackManager.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3;
namespace {
const cv::Size size(320,240);
void Check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
template<class F> void Reject(F f){bool bad=false;try{f();}catch(const std::runtime_error&){bad=true;}Check(bad,"invalid input accepted");}
YOLOSegInstance Person(int x,int y=40,int w=50,int h=130) {
    YOLOSegInstance p;p.classId=0;p.confidence=.9;p.boundingBox=cv::Rect(x,y,w,h);
    p.mask=cv::Mat::zeros(size,CV_8UC1);p.mask(p.boundingBox).setTo(255);return p;
}
void Step(InstanceTrackManager& manager,std::vector<YOLOSegInstance>& p,long frame,
          const cv::Mat& gray=cv::Mat(),const cv::Mat& depth=cv::Mat()) {
    manager.Associate(p,size,frame,frame/30.,gray,depth);
}
InstanceAssociationConfig Config(int mode=3){InstanceAssociationConfig c;c.mode=mode;return c;}
void ContinuousFlowAndCamera() {
    InstanceTrackManager manager(B4Config{},Config());
    cv::Mat gray(size,CV_8UC1);cv::randu(gray,0,255);
    std::vector<YOLOSegInstance> p{Person(70)};Step(manager,p,0,gray);
    const int id=p[0].trackId;
    Check(!p[0].association.allowStateInheritance&&!p[0].association.allowGeometryPair,"new track trusted");
    cv::Mat moved;cv::Mat affine=(cv::Mat_<double>(2,3)<<1,0,5,0,1,2);
    cv::warpAffine(gray,moved,affine,size);
    p={Person(75,42)};Step(manager,p,1,moved);
    auto a=p[0].association;
    Check(p[0].trackId==id&&a.associationStatus==AssociationStatus::CONTINUOUS_CONFIRMED&&a.allowGeometryPair&&a.allowStateInheritance,"continuous ID/permissions");
    Check(a.prediction=="LK_FB_MEDIAN"&&a.flowSupport>=8,"camera translation LK prediction");
    Check(std::isnan(a.rowMargin)&&std::isnan(a.columnMargin),"no second candidate must stay missing");
    Reject([&]{Step(manager,p,1,moved);});
    Reject([&]{manager.Associate(p,size,2,0.,moved);});
    Reject([&]{manager.Associate(p,size,2,std::numeric_limits<double>::quiet_NaN(),moved);});
    p={Person(80,44)};Step(manager,p,2,moved);
    Check(p[0].trackId==id,"rejected input mutated history");
    manager.Reset();p={Person(70)};Step(manager,p,0,gray);
    Check(p[0].trackId>id&&manager.SequenceId()==1,"reset must clear history without recycling IDs");
}
void TTLAndReacquisition() {
    B4Config ttl;ttl.trackTTL=2;
    InstanceTrackManager manager(ttl,Config());std::vector<YOLOSegInstance> p{Person(70)};Step(manager,p,0);
    const int id=p[0].trackId;
    p.clear();Step(manager,p,1);Step(manager,p,2);
    p={Person(70)};Step(manager,p,3);
    Check(p[0].trackId==id&&p[0].association.associationStatus==AssociationStatus::REACQUIRED&&
        p[0].association.resetEvidence&&!p[0].association.allowGeometryPair&&!p[0].association.allowStateInheritance,"TTL boundary reacquisition must reset evidence");
    InstanceTrackManager expired(ttl,Config());p={Person(70)};Step(expired,p,0);int old=p[0].trackId;
    p={Person(70)};Step(expired,p,4);
    Check(p[0].trackId!=old,"expired ID revived before assignment");
    bool event=false;for(const auto& e:expired.Events())event|=e.trackId==old&&e.type=="TERMINATED";
    Check(event,"missing explicit termination event");
    InstanceTrackManager noDetection(ttl,Config());p={Person(70)};Step(noDetection,p,0);
    p.clear();Step(noDetection,p,1);Step(noDetection,p,2);Step(noDetection,p,3);
    Check(!noDetection.Events().empty()&&noDetection.Events()[0].type=="TERMINATED","16th/TTL+1 missing frame must terminate");
}
void CrossingMergeSplitAndCompetition() {
    InstanceTrackManager m(B4Config{},Config());std::vector<YOLOSegInstance> p{Person(50),Person(120)};
    Step(m,p,0);const int a=p[0].trackId,b=p[1].trackId;
    // Two silhouettes compete for the same image region at a crossing/merge.
    auto merged=Person(50,40,120,130);p={merged};Step(m,p,1);
    Check(p[0].association.associationStatus==AssociationStatus::AMBIGUOUS&&p[0].trackId!=a&&p[0].trackId!=b&&
        !p[0].association.allowStateInheritance&&!p[0].association.allowGeometryPair,"merged instance inherited identity");
    p={Person(50),Person(120)};Step(m,p,2);
    Check(p[0].trackId==a&&p[1].trackId==b&&p[0].association.resetEvidence&&p[1].association.resetEvidence,"split must recover old IDs only with reset");
    InstanceTrackManager competition(B4Config{},Config());p={Person(70)};Step(competition,p,0);int old=p[0].trackId;
    p={Person(70),Person(70)};Step(competition,p,1);
    Check(p[0].trackId!=p[1].trackId&&p[0].trackId!=old&&p[1].trackId!=old,"ambiguous row competition forced a match");
    Check(p[0].association.associationStatus==AssociationStatus::AMBIGUOUS&&p[1].association.associationStatus==AssociationStatus::AMBIGUOUS,"competition must be logged ambiguous");
}
void DepthFallbackAndNewPerson() {
    InstanceTrackManager m(B4Config{},Config());std::vector<YOLOSegInstance> p{Person(70)};
    cv::Mat blank(size,CV_8UC1,cv::Scalar(0)),depth(size,CV_32F,cv::Scalar(2));
    Step(m,p,0,blank,depth);int id=p[0].trackId;
    p={Person(70)};Step(m,p,1,blank,depth);
    Check(p[0].trackId==id&&p[0].association.prediction=="VELOCITY_LOW_CONFIDENCE","flow failure must fall back explicitly");
    depth.setTo(0);p={Person(70)};Step(m,p,2,blank,depth);
    Check(p[0].trackId==id&&!p[0].association.depthAvailable,"missing depth must not imply new identity");
    InstanceTrackManager newcomer(B4Config{},Config());p={Person(70)};depth.setTo(2);Step(newcomer,p,0,blank,depth);id=p[0].trackId;
    depth.setTo(4);p={Person(70)};Step(newcomer,p,1,blank,depth);
    Check(p[0].trackId!=id&&!p[0].association.allowStateInheritance,"depth-inconsistent newcomer inherited old state");
    auto invalid=Person(70);invalid.mask=cv::Mat::zeros(5,5,CV_8UC1);p={invalid};
    Reject([&]{Step(m,p,3);});
    for(int mode=0;mode<=3;++mode) {
        InstanceTrackManager one(B4Config{},Config(mode));p={Person(70)};Step(one,p,0);id=p[0].trackId;
        Check(!p[0].association.allowGeometryPair,"first frame -1 sentinel mistaken for adjacent history");
        p={Person(71)};Step(one,p,1);Check(p[0].trackId==id,"ablation simple continuity");
    }
}
}
int main(){try {
    cv::setNumThreads(1);
    Check(InstanceTrackManager::SolveAssignment({{.1,.11},{.12,.9}})==std::vector<int>({1,0}),"Hungarian not globally optimal");
    ContinuousFlowAndCamera();TTLAndReacquisition();CrossingMergeSplitAndCompetition();DepthFallbackAndNewPerson();
    std::cout<<"PASS association: global assignment, continuity/LK, crossings/merge/split, ambiguity, TTL, depth, time/reset, A0-A3\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
