#include "StaticQuality.h"
#include "Frame.h"
#include "MapPoint.h"
#include "FineFeaturePolicy.h"
#include <chrono>
namespace ORB_SLAM3 {
StaticQualitySummary PrepareStaticQuality(Frame& f,const Sophus::SE3f& coarse,
    bool reliable,const std::string& skip,const StaticQualityConfig& c) {
    const auto start=std::chrono::steady_clock::now();
    StaticQualitySummary s;
    if(!c.enable) return s; // Disabled: no valid state and no mutation.
    f.ResetStaticQuality(); // A failed preparation cannot leave stale valid weights.
    ValidateStaticQuality(c);
    VerifyFineOptimizerInputs(f); // Cannot resurrect a forbidden association.
    if(f.mvDepth.size()!=size_t(f.N)) throw std::runtime_error("StaticQuality depth index mismatch");
    reliable=reliable && coarse.matrix().allFinite();
    s.applied=reliable;
    s.skipReason=reliable?"NONE":(skip.empty()?"COARSE_POSE_UNRELIABLE":skip);
    double squaredSum=0;
    for(int i=0;i<f.N;++i) {
        auto* point=f.mvpMapPoints[i];
        if(!point) continue; // Exactly the visual optimizer's observation set.
        ++s.allowed;
        std::string reason;
        double w=1;
        if(!reliable) reason=s.skipReason;
        else if(point->isBad()) reason="MAPPOINT_BAD";
        else if(!std::isfinite(f.mvDepth[i]) || f.mvDepth[i]<=0) reason="DEPTH_INVALID";
        else {
            const auto xyz=coarse*point->GetWorldPos();
            if(!xyz.allFinite()) reason="PREDICTION_NONFINITE";
            else if(xyz.z()<=0) reason="PREDICTION_BEHIND_CAMERA";
            else {
                const double z=f.mvDepth[i];
                const double sigma=c.depthSigmaA+c.depthSigmaB*z*z;
                const double u=std::abs(z-xyz.z())/sigma;
                w=StaticDepthWeight(z,xyz.z(),c);
                if(!std::isfinite(sigma) || sigma<=0 || !std::isfinite(u) || !std::isfinite(w)) {w=1;reason="WEIGHT_NONFINITE";}
                else {f.mvStaticQualityValid[i]=true;reason="DEPTH_EVALUATED";++s.valid;}
            }
        }
        if(!f.mvStaticQualityValid[i]) ++s.neutral;
        f.mvStaticQualityWeight[i]=w;
        f.mvStaticQualityReason[i]=reason;
        s.minimum=std::min(s.minimum,w);s.sum+=w;squaredSum+=w*w;
    }
    if(s.allowed) {s.mean=s.sum/s.allowed;s.effective=s.sum*s.sum/squaredSum;}
    if(reliable && !s.valid) {
        s.applied=false;
        s.skipReason=s.allowed?"NO_VALID_DEPTH_QUALITY":"NO_OBSERVATIONS";
    }
    f.mbStaticQualityPrepared=true;
    s.computeMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    return s;
}
}
