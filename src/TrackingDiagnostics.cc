#include "Tracking.h"
#include "FeaturePolicyEvaluator.h"
#include <iomanip>
#include <sstream>
#include <tuple>
#include <sys/stat.h>
#include <cerrno>
#include <stdexcept>
namespace ORB_SLAM3 {
namespace {
template<class T> std::string Number(T value) {
    std::ostringstream text; text << std::setprecision(17) << value; return text.str();
}
const char* State(TemporalState s) {
    switch(s) {case TemporalState::STATIC:return "S";case TemporalState::PENDING:return "P";
        case TemporalState::DYNAMIC:return "D";default:return "UNKNOWN";}
}
const char* Region(SemanticRegion s) {
    switch(s) {case SemanticRegion::BACKGROUND:return "BACKGROUND";
        case SemanticRegion::PERSON_INTERIOR:return "PERSON_INTERIOR";
        case SemanticRegion::PERSON_BOUNDARY:return "PERSON_BOUNDARY";default:return "UNKNOWN";}
}
TemporalState Temporal(B4MotionState s) {
    return s==B4MotionState::STATIC ? TemporalState::STATIC :
        s==B4MotionState::DYNAMIC ? TemporalState::DYNAMIC : TemporalState::PENDING;
}
std::string Action(bool allowed,bool enabled,bool ran,bool fallback,bool mapping=false) {
    const std::string proposal=allowed ? "PASS" : mapping ? "DENY" : "REJECT";
    if(!enabled) return "PROPOSE_"+proposal;
    if(fallback) return "RESTORED_FALLBACK";
    if(!ran) return "NOT_RUN";
    return proposal;
}
}
void Tracking::CollectDynamicDiagnostics() {
    // RGB-D diagnostics only. Stereo/mono retain their previous drawing path.
    if(mSensor!=System::RGBD || !mpDynamicDetector) return;
    const auto& cfg=mpDynamicDetector->Frontend();
    if(!cfg.CollectDiagnostics()) return;
    auto& view=mFrontendDiagnostics;
    view.stateActionOverlayEnabled=cfg.diagnosticShowOverlay;
    view.currentFeatureStates=mCurrentFrame.mvCurrentB4State;
    view.stateActionInstances.clear();
    std::ostringstream timing;
    timing << std::fixed << std::setprecision(1) << "ms YOLO/ORB/coarse/B2/B3/B4/fine "
        << view.yoloInferenceMs << '/' << mCurrentFrame.mDiagnosticOrbMs << '/'
        << mDiagnosticStages.coarseMs << '/' << mDiagnosticStages.b2Ms << '/'
        << mDiagnosticStages.b3Ms << '/' << mDiagnosticStages.b4Ms << '/' << mDiagnosticStages.fineMs;
    view.stageTimingSummary=timing.str();
    const auto base=[&](const char* type) {
        return DiagnosticRow{{"row_type",type},{"frame_id",Number(mCurrentFrame.mnId)},
            {"timestamp",Number(mCurrentFrame.mTimeStamp)},{"tracking_state",Number(int(mState))}};
    };
    const auto emit=[&](const DiagnosticRow& row) { if(mpFrontendDiagnosticLog) mpFrontendDiagnosticLog->Append(row); };
    DiagnosticRow frame=base("FRAME");
    frame["semantic_region"]="FRAME_SUMMARY";
    frame["mode"]=cfg.explicitMode ? Number(int(cfg.mode)) : "LEGACY";
    frame["final_orb"]=Number(mCurrentFrame.N);
    frame["raw_fast"]=Number(view.rawFastCandidates);
    frame["b1_mask_rejected"]=Number(view.maskedRejected);
    frame["pose_inliers"]=Number(view.poseInliers);
    int personMainOrb=0;
    for(const auto& key:mCurrentFrame.mvKeys) {
        const int x=cvRound(key.pt.x),y=cvRound(key.pt.y);
        for(const auto& person:mpDynamicDetector->LastRGBDInstances()) {
            if(x>=0 && y>=0 && x<person.mask.cols && y<person.mask.rows && person.mask.at<uchar>(y,x)) {
                ++personMainOrb;break;
            }
        }
    }
    frame["person_main_orb"]=Number(personMainOrb);
    frame["keyframe_created"]=Number(int(view.becameKeyFrame));
    frame["coarse_filter_enabled"]=Number(int(cfg.coarseFilterEnable));
    frame["fine_policy_enabled"]=Number(int(cfg.finePolicyEnable));
    frame["map_admission_enabled"]=Number(int(cfg.mapAdmissionEnable));
    const auto& m=mDiagnosticStages;
    frame["coarse_attempts"]=Number(m.coarseAttempts);
    if(m.coarseAttempts) {
        frame["coarse_matches_before"]=Number(m.coarseBefore);
        frame["coarse_rejected"]=Number(m.coarseRejected);
        frame["coarse_matches_after"]=Number(m.coarseAfter);
        frame["coarse_inliers"]=Number(m.coarseInliers);
    } else if(m.coarseMs>=0 && cfg.useCoarsePoseForB2) {
        frame["coarse_matches_before"]=Number(mCoarsePoseQuality.match_count);
        frame["coarse_matches_after"]=Number(mCoarsePoseQuality.match_count);
        frame["coarse_rejected"]="0";
        frame["coarse_inliers"]=Number(mCoarsePoseQuality.inlier_count);
    }
    if(m.fineBefore>=0) {
        frame["fine_matches_before"]=Number(m.fineBefore);
        frame["fine_reject_P"]=Number(m.fineP); frame["fine_reject_D"]=Number(m.fineD);
        frame["fine_reject_boundary"]=Number(m.fineBoundary); frame["fine_reject_unknown"]=Number(m.fineUnknown);
        frame["fine_matches_after"]=Number(m.fineAfter);
    }
    if(m.fineInliers>=0) frame["fine_inliers"]=Number(m.fineInliers);
    frame["map_counts_scope"]=cfg.mapAdmissionEnable ? "SOURCE_KEYFRAME_FRAME" : "DISABLED";
    frame["map_counts_pending"]=Number(int(cfg.mapAdmissionEnable));
    std::string fallback;
    if(!mCoarseFilterFallbackReason.empty()) fallback=mCoarseFilterFallbackReason;
    else if(cfg.useCoarsePoseForB2 && !mCoarsePoseQuality.reliable) fallback=mCoarsePoseQuality.reason;
    if(mbFineTrackingFailed) {
        if(!fallback.empty()) fallback+=';';
        fallback+="FINE_TRACKING_INSUFFICIENT_STATIC_SUPPORT";
    }
    frame["fallback"]=Number(int(!fallback.empty())); frame["fallback_reason"]=fallback.empty()?"NONE":fallback;
    frame["yolo_ms"]=Number(view.yoloInferenceMs); frame["orb_ms"]=Number(mCurrentFrame.mDiagnosticOrbMs);
    frame["coarse_ms"]=Number(m.coarseMs); frame["b2_ms"]=Number(m.b2Ms);
    frame["b3_ms"]=Number(m.b3Ms); frame["b4_ms"]=Number(m.b4Ms); frame["fine_ms"]=Number(m.fineMs);
    emit(frame);

    // Match B2/B3/B4 by SAME-frame identity. TTL-retained tracks must never
    // inherit geometry or semantic results from their last visible frame.
    const auto instance=[&](int id,int track,const cv::Rect& box,const B4StateResult* b4) {
        DiagnosticRow row=base("INSTANCE");
        row["instance_id"]=Number(id); row["track_id"]=Number(track);
        const bool visible=id>=0;
        row["semantic_region"]=visible ? "PERSON_INSTANCE" : "NOT_VISIBLE";
        TemporalState previous=TemporalState::UNKNOWN,current=TemporalState::UNKNOWN;
        if(b4) {
            previous=Temporal(b4->previousState); current=Temporal(b4->state);
            row["previous_b4_state"]=State(previous); row["current_b4_state"]=State(current);
            row["Ns"]=Number(b4->nStatic);row["Nd"]=Number(b4->nDynamic);row["Nr"]=Number(b4->nRecovery);
            row["missed_frames"]=Number(b4->missedFrames);row["b4_transition_reason"]=b4->reasonCode;
        }
        if(visible) {
            const B3FusionResult* b3=nullptr;
            for(const auto& result:mCurrentB3Results)
                if(result.frame_id==static_cast<long>(mCurrentFrame.mnId) && result.instance_id==id && result.track_id==track) {
                    if(b3) throw std::runtime_error("Duplicate B3 diagnostics identity");
                    b3=&result;
                }
            if(b3) {
                row["b2_raw_r3d"]=Number(b3->raw_r3d_median);row["b2_norm_r3d"]=Number(b3->norm_r3d_median);
                row["b2_high_ratio"]=Number(b3->high_ratio);row["b2_reliable"]=Number(int(b3->geometry_reliable));
                row["b2_reason"]=b3->geometry_reason;
                row["b3_fusion_category"]=B3FusionCategoryName(b3->fusion_category);
                row["b3_reliable"]=Number(int(b3->fusion_reliable));row["b3_reason"]=b3->fusion_reason;
            } else {row["b2_reason"]="NO_B2_RESULT";row["b3_reason"]="NO_B3_RESULT";}
        }
        const bool coarse=visible && previous==TemporalState::STATIC;
        const bool fine=visible && current==TemporalState::STATIC;
        row["allow_coarse"]=Number(int(coarse));row["allow_fine"]=Number(int(fine));row["allow_map"]=Number(int(fine));
        // INSTANCE actions describe interiors only. Boundary exceptions remain
        // explicit in FEATURE_GROUP rows; no whole-box policy is applied here.
        row["coarse_action"]=visible ? Action(coarse,cfg.coarseFilterEnable,m.coarseApplied,!mCoarseFilterFallbackReason.empty()) : "NOT_VISIBLE";
        row["fine_action"]=visible ? Action(fine,cfg.finePolicyEnable,mbFinePolicyApplied,false) : "NOT_VISIBLE";
        row["map_action"]=visible ? Action(fine,cfg.mapAdmissionEnable,true,false,true) : "NOT_VISIBLE";
        emit(row);
        B3ViewerInstance label;label.boundingBox=box;
        label.color=visible ? FrameDrawer::TemporalFeatureColor(SemanticRegion::PERSON_INTERIOR,current) : cv::Scalar(128,128,128);
        label.primaryLabel="ID"+Number(track)+" B4 state="+State(current)+(visible?"":" LOST");
        label.secondaryLabel="interior coarse_action="+row["coarse_action"];
        label.tertiaryLabel="fine_action="+row["fine_action"]+" map_action="+row["map_action"];
        label.quaternaryLabel="Ns/Nd/Nr="+row["Ns"]+'/'+row["Nd"]+'/'+row["Nr"]+" "+row["b4_transition_reason"];
        if(!b4 || visible || b4->missedFrames<=mB4Config.trackTTL)
            view.stateActionInstances.push_back(label);
    };
    const auto& persons=mpDynamicDetector->LastRGBDInstances();
    for(size_t id=0;id<persons.size();++id) {
        const auto& person=persons[id];
        const B4StateResult* b4=nullptr;
        for(const auto& result:mCurrentB4Results)
            if(result.frameId==static_cast<long>(mCurrentFrame.mnId) && !result.isLost &&
               result.instanceId==static_cast<int>(id) && result.trackId==person.trackId) b4=&result;
        instance(static_cast<int>(id),person.trackId,person.boundingBox,b4);
    }
    for(const auto& b4:mCurrentB4Results)
        if(b4.isLost) instance(-1,b4.trackId,b4.boundingBox,&b4);

    // Pure policy queries; never assign to Frame feature or MapPoint arrays.
    if(!mCurrentFrame.SemanticMetadataAligned()) throw std::runtime_error("Diagnostic feature metadata unaligned");
    const bool annotated=mCurrentFrame.mbSemanticAnnotated;
    const auto coarse=annotated ? ComputeFeaturePolicies(mCurrentFrame,FeaturePolicyPhase::COARSE,true) :
        std::vector<FeatureUsePolicy>(mCurrentFrame.N,FeatureUsePolicy::EXCLUDE_UNKNOWN);
    const auto fine=annotated ? ComputeFeaturePolicies(mCurrentFrame,FeaturePolicyPhase::FINE,true) : coarse;
    const auto mapping=annotated ? ComputeFeaturePolicies(mCurrentFrame,FeaturePolicyPhase::MAPPING,true) : coarse;
    std::map<std::string,DiagnosticRow> groups;
    std::map<std::string,int> sizes;
    for(int i=0;i<mCurrentFrame.N;++i) {
        DiagnosticRow row=base("FEATURE_GROUP");
        row["semantic_region"]=Region(mCurrentFrame.mvSemanticRegion[i]);
        row["instance_id"]=Number(mCurrentFrame.mvInstanceId[i]);row["track_id"]=Number(mCurrentFrame.mvTrackId[i]);
        row["previous_b4_state"]=State(mCurrentFrame.mvPreviousB4State[i]);row["current_b4_state"]=State(mCurrentFrame.mvCurrentB4State[i]);
        const bool c=FeaturePolicyAllows(coarse[i],FeaturePolicyPhase::COARSE);
        const bool f=FeaturePolicyAllows(fine[i],FeaturePolicyPhase::FINE);
        const bool a=FeaturePolicyAllows(mapping[i],FeaturePolicyPhase::MAPPING);
        row["allow_coarse"]=Number(int(c));row["allow_fine"]=Number(int(f));row["allow_map"]=Number(int(a));
        row["coarse_action"]=Action(c,cfg.coarseFilterEnable,m.coarseApplied,!mCoarseFilterFallbackReason.empty());
        row["fine_action"]=Action(f,cfg.finePolicyEnable,mbFinePolicyApplied,false);
        row["map_action"]=Action(a,cfg.mapAdmissionEnable,true,false,true);
        if(!annotated) {
            row.erase("allow_coarse");row.erase("allow_fine");row.erase("allow_map");
            row["coarse_action"]=row["fine_action"]=row["map_action"]="DISABLED";
        }
        std::string key;for(const auto& cell:row) key+=cell.first+'='+cell.second+';';
        groups[key]=std::move(row);++sizes[key];
    }
    for(auto& group:groups) {group.second["feature_count"]=Number(sizes[group.first]);emit(group.second);}
    if(cfg.diagnosticSaveOverlay) {
        const std::string folder=mB2Config.csvOutputParent+"/dynamic_frontend_overlays";
        if(mkdir(folder.c_str(),0755)!=0 && errno!=EEXIST)
            throw std::runtime_error("Cannot create diagnostic overlay directory: "+folder);
    }
}
void Tracking::FinalizeDynamicDiagnostics() {
    if(mpFrontendDiagnosticLog)
        mpFrontendDiagnosticLog->Finalize(mMapAdmissionLogPath,mpDynamicDetector->Frontend().mapAdmissionEnable);
}
}
