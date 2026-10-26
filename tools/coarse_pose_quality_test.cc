#include "CoarsePoseQuality.h"
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3;
int main() {
    try {
        CoarsePoseQuality q; q.match_count=100; q.inlier_count=30;
        CoarsePoseContext c; c.initialized=true; c.initial_success=true;
        c.finite_pose=true; c.background_features=80; c.background_inliers=30;
        const auto check=[&](CoarsePoseQuality a,CoarsePoseContext b,const char* reason) {
            const auto r=EvaluateCoarsePoseQuality(a,b,30,.30f);
            if (r.reason!=reason || r.reliable!=(r.reason=="COARSE_POSE_RELIABLE"))
                throw std::runtime_error(std::string("Unexpected gate: ")+r.reason+" expected "+reason);
        };
        check(q,c,"COARSE_POSE_RELIABLE");
        auto d=c; d.initialized=false; check(q,d,"NOT_INITIALIZED");
        d=c; d.new_map=true; check(q,d,"NEW_MAP");
        d=c; d.relocalizing=true; check(q,d,"RELOCALIZATION");
        d=c; d.just_recovered=true; check(q,d,"JUST_RECOVERED");
        d=c; d.initial_success=false; check(q,d,"COARSE_TRACK_FAILED");
        d=c; d.background_features=29; check(q,d,"INSUFFICIENT_COARSE_FEATURES");
        d=c; d.background_inliers=29; check(q,d,"INSUFFICIENT_COARSE_FEATURES");
        auto a=q; a.inlier_count=29; check(a,c,"INSUFFICIENT_COARSE_INLIERS");
        a=q; a.match_count=101; check(a,c,"POSE_NOT_RELIABLE");
        d=c; d.finite_pose=false; check(q,d,"POSE_NOT_RELIABLE");
        d=c; d.pure_vo=true; check(q,d,"POSE_NOT_RELIABLE");
        a=q; a.inlier_count=101; check(a,c,"POSE_NOT_RELIABLE");
        a=q; a.match_count=0; a.inlier_count=0; check(a,c,"INSUFFICIENT_COARSE_INLIERS");
        if(q.reliable || !q.reason.empty() || q.inlier_ratio!=0.f)
            throw std::runtime_error("Input quality was mutated");
        std::cout<<"PASS all quality reasons, threshold boundaries and immutable input\n";
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 2;}
}
