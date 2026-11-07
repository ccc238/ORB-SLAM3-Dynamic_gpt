#include "StaticQuality.h"
#include "Optimizer.h"
#include "Pinhole.h"
#include "ORBextractor.h"
#include "FineFeaturePolicy.h"
#include <iostream>
#include <memory>
#include <limits>
using namespace ORB_SLAM3;
static void Check(bool b,const char* text) {if(!b) throw std::runtime_error(text);}
int main() {
 try {
    StaticQualityConfig c;c.enable=true;ValidateStaticQuality(c);
    double last=1;
    for(int i=0;i<1000;++i) {
        double w=StaticDepthWeight(2,2+i*.001,c);
        Check(w<=last && w>=c.weightMin && w<=1,"Weight not monotone/bounded");last=w;
    }
    for(int i=0;i<5;++i) {
        auto bad=c;
        if(i==0)bad.weightMin=0;
        if(i==1)bad.weightMin=1.1;
        if(i==2)bad.depthSigmaA=0;
        if(i==3)bad.depthSigmaB=-1;
        if(i==4)bad.depthSigmaA=std::numeric_limits<double>::quiet_NaN();
        bool caught=false;try{ValidateStaticQuality(bad);}catch(...){caught=true;}
        Check(caught,"Illegal parameter accepted");
    }
    Pinhole camera({500,500,320,240});
    // A real Frame constructor initializes copy-sensitive calibration fields.
    cv::Mat gray(480,640,CV_8U);cv::randu(gray,0,255);
    cv::Mat depth(480,640,CV_32F,cv::Scalar(3));
    cv::Mat K=(cv::Mat_<float>(3,3)<<500,0,320,0,500,240,0,0,1),dist=cv::Mat::zeros(4,1,CV_32F);
    ORBextractor extractor(100,1.2f,4,20,7);
    Frame f(gray,depth,0,&extractor,nullptr,K,dist,40,4,&camera);
    f.N=100;f.ResetSemanticMetadata();f.mbSemanticAnnotated=true;
    f.mvSemanticRegion.assign(f.N,SemanticRegion::BACKGROUND);
    f.mvKeys.resize(f.N);f.mvKeysUn.resize(f.N);f.mvuRight.resize(f.N);
    f.mvDepth.assign(f.N,3);f.mvpMapPoints.resize(f.N);f.mvbOutlier.assign(f.N,false);
    f.mvInvLevelSigma2={1};f.SetPose(Sophus::SE3f());
    std::vector<std::unique_ptr<MapPoint>> points;
    for(int i=0;i<f.N;++i) {
        Eigen::Vector3f xyz((i%10-4.5f)*.12f,(i/10-4.5f)*.10f,3+(i%3)*.2f);
        points.emplace_back(new MapPoint());points.back()->SetWorldPos(xyz);
        f.mvpMapPoints[i]=points.back().get();
        float u=500*xyz.x()/xyz.z()+320,v=500*xyz.y()/xyz.z()+240;
        // Subthreshold image bias: must influence ordinary optimization, not be removed as outliers.
        if(i<30)u+=1.5f;
        f.mvKeys[i]=f.mvKeysUn[i]=cv::KeyPoint(u,v,7);f.mvKeysUn[i].octave=0;
        f.mvuRight[i]=u-40/xyz.z();f.mvDepth[i]=xyz.z();
    }
    auto summary=PrepareStaticQuality(f,Sophus::SE3f(),true,"",c);
    Check(summary.valid==100 && summary.neutral==0 && summary.minimum==1,"Aligned depth weights");
    const float measured=f.mvDepth[5];f.mvDepth[5]+=0.1f;
    summary=PrepareStaticQuality(f,Sophus::SE3f(),true,"",c);
    Check(f.mvStaticQualityValid[5] && f.mvStaticQualityWeight[5]<1 &&
        std::abs(f.mvStaticQualityWeight[5]-StaticDepthWeight(f.mvDepth[5],measured,c))<1e-12,
        "Actual MapPoint/coarse/depth preparation failed");
    f.mvDepth[5]=measured;PrepareStaticQuality(f,Sophus::SE3f(),true,"",c);
    Frame a(f),b(f);
    Check(a.mvStaticQualityWeight==f.mvStaticQualityWeight && a.mbStaticQualityPrepared,"Frame copy weights");
    int na=Optimizer::PoseOptimization(&a),nb=Optimizer::PoseOptimizationStaticQuality(&b);
    Check(na==nb && a.GetPose().matrix().isApprox(b.GetPose().matrix(),1e-7f),"w=1 optimizer compatibility");
    a=f;b=f;
    for(int i=0;i<30;++i)b.mvStaticQualityWeight[i]=.001;
    Optimizer::PoseOptimization(&a);Optimizer::PoseOptimizationStaticQuality(&b);
    for(int i=0;i<30;++i)Check(b.mvStaticQualityWeight[i]==.001,"Optimizer updated frozen weights");
    double oldError=a.GetPose().translation().norm(),newError=b.GetPose().translation().norm();
    Check(newError<oldError*.5,"Low-weight biased 3D observations did not reduce pose influence");
    std::cout<<"synthetic_translation_error old="<<oldError<<" weighted="<<newError<<'\n';
    // Extreme raw residual must be rejected even when its weighted chi2 is tiny.
    b=f;b.mvKeysUn[0].pt.x+=100;b.mvuRight[0]+=100;b.mvStaticQualityWeight[0]=1e-8;
    Optimizer::PoseOptimizationStaticQuality(&b);
    Check(b.mvbOutlier[0] && b.GetPose().matrix().allFinite(),"Weighted stereo outlier bypassed raw gate");
    b=f;b.mvuRight[0]=-1;b.mvKeysUn[0].pt.x+=100;b.mvStaticQualityWeight[0]=1e-8;
    Optimizer::PoseOptimizationStaticQuality(&b);
    Check(b.mvbOutlier[0],"Weighted mono edge bypassed raw gate");
    // Ordinary entry must ignore even prepared, nontrivial weights.
    a=f;b=f;for(auto& w:b.mvStaticQualityWeight)w=.001;
    Optimizer::PoseOptimization(&a);Optimizer::PoseOptimization(&b);
    Check(a.GetPose().matrix().isApprox(b.GetPose().matrix(),1e-7f),"Ordinary optimizer consumed weights");
    f.mvDepth[0]=0;f.mvDepth[1]=std::numeric_limits<float>::quiet_NaN();
    summary=PrepareStaticQuality(f,Sophus::SE3f(),true,"",c);
    Check(summary.neutral==2 && !f.mvStaticQualityValid[0],"Invalid depth not neutral");
    summary=PrepareStaticQuality(f,Sophus::SE3f(),false,"COARSE_FALLBACK",c);
    Check(summary.neutral==100 && summary.valid==0 && !summary.applied,"Unreliable coarse not neutral");
    Sophus::SE3f behind(Eigen::Quaternionf::Identity(),Eigen::Vector3f(0,0,-10));
    summary=PrepareStaticQuality(f,behind,true,"",c);
    Check(summary.neutral==100 && f.mvStaticQualityReason[2]=="PREDICTION_BEHIND_CAMERA","Behind camera");
    f.mvSemanticRegion[3]=SemanticRegion::PERSON_INTERIOR;
    f.mvCurrentB4State[3]=TemporalState::PENDING;f.mvInstanceId[3]=0;f.mvTrackId[3]=0;
    ApplyCurrentFineFeaturePolicy(f);
    summary=PrepareStaticQuality(f,Sophus::SE3f(),true,"",c);
    Check(!f.mvpMapPoints[3] && summary.allowed==99,"Forbidden feature resurrected");
    const auto weights=f.mvStaticQualityWeight;c.enable=false;
    PrepareStaticQuality(f,Sophus::SE3f(),true,"",c);
    Check(weights==f.mvStaticQualityWeight,"Disabled preparation mutated Frame");
    f.ResetStaticQuality();Check(!f.mbStaticQualityPrepared && f.mvStaticQualityWeight.size()==100 &&
        f.mvStaticQualityWeight[3]==1,"Reset/indices");
    Frame empty;empty.ResetStaticQuality();Check(empty.mvStaticQualityWeight.empty(),"Empty reset");
    // Assignment copies metadata; a new frame/reset cannot inherit valid weights.
    b=f;Check(!b.mbStaticQualityPrepared && b.mvStaticQualityWeight==f.mvStaticQualityWeight,"Assignment reset");
    c.enable=true;f.mvDepth.assign(f.N,3);points[2]->SetWorldPos(Eigen::Vector3f(
        std::numeric_limits<float>::quiet_NaN(),0,3));
    summary=PrepareStaticQuality(f,Sophus::SE3f(),true,"",c);
    Check(!f.mvStaticQualityValid[2] && f.mvStaticQualityReason[2]=="PREDICTION_NONFINITE","Nonfinite world point");
    auto depths=f.mvDepth;f.mvDepth.pop_back();
    bool alignmentRejected=false;
    try {PrepareStaticQuality(f,Sophus::SE3f(),true,"",c);}catch(const std::runtime_error&){alignmentRejected=true;}
    Check(alignmentRejected && !f.mbStaticQualityPrepared,"Failed preparation left stale valid state");
    f.mvDepth=depths;
    auto invalidPose=Sophus::SE3f();invalidPose.translation().x()=std::numeric_limits<float>::quiet_NaN();
    summary=PrepareStaticQuality(f,invalidPose,true,"",c);
    Check(!summary.applied && summary.valid==0 && summary.neutral==99,"Nonfinite coarse pose not neutral");
    // Verify actual g2o Huber identity rho_(sqrt(w)*delta)(w*e)=w*rho_delta(e).
    for(double w:{1.,.1,1e-8})for(double e:{0.,1.,10.,10000.}) {
        g2o::RobustKernelHuber original,scaled;original.setDelta(std::sqrt(7.815));scaled.setDelta(std::sqrt(w*7.815));
        Eigen::Vector3d r,s;original.robustify(e,r);scaled.robustify(w*e,s);
        // This vendored g2o caches delta^2 in float (not double).
        Check(std::abs(s[0]-w*r[0])<1e-6*std::max(1.0,std::abs(w*r[0])),"Huber loss scaling");
    }
    std::cout<<"PASS StaticQuality formula/config/lifecycle, neutral depth/pose, hard-filter invariants, real 2D/3D optimizer, raw chi2, Huber, legacy entry\n";
 } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
