#include "FeaturePolicyEvaluator.h"
#include <algorithm>
#include <iomanip>
#include <stdexcept>

namespace ORB_SLAM3 {
namespace {
void Validate(const Frame& frame, bool allowAppliedCoarse) {
    if (!frame.mbSemanticAnnotated || !frame.SemanticMetadataAligned())
        throw std::runtime_error("FeaturePolicy requires aligned, annotated Frame metadata");
    if (!allowAppliedCoarse && (std::any_of(frame.mvDynamicExcluded.begin(),frame.mvDynamicExcluded.end(),[](bool value){return value;}) ||
        std::any_of(frame.mvFeaturePolicy.begin(),frame.mvFeaturePolicy.end(),[](FeatureUsePolicy p){return p!=FeatureUsePolicy::USE_NORMAL;})))
        throw std::runtime_error("FeaturePolicy statistics-only mode found an applied feature policy");
}
bool HasOwner(const Frame& frame, int index) {
    return frame.mvInstanceId[index] >= 0 && frame.mvTrackId[index] >= 0;
}
FeatureUsePolicy StatePolicy(TemporalState state) {
    switch (state) {
    case TemporalState::STATIC: return FeatureUsePolicy::USE_NORMAL;
    case TemporalState::PENDING: return FeatureUsePolicy::EXCLUDE_PENDING;
    case TemporalState::DYNAMIC: return FeatureUsePolicy::EXCLUDE_DYNAMIC;
    default: return FeatureUsePolicy::EXCLUDE_UNKNOWN;
    }
}
}

bool FeaturePolicyAllows(FeatureUsePolicy policy, FeaturePolicyPhase phase) {
    return policy == FeatureUsePolicy::USE_NORMAL ||
        (policy == FeatureUsePolicy::USE_POSE_ONLY && phase != FeaturePolicyPhase::MAPPING);
}

std::vector<FeatureUsePolicy> ComputeFeaturePolicies(const Frame& frame, FeaturePolicyPhase phase,
    bool allowAppliedCoarse) {
    Validate(frame,allowAppliedCoarse);
    std::vector<FeatureUsePolicy> proposed(frame.N, FeatureUsePolicy::EXCLUDE_UNKNOWN);
    const auto& states = phase == FeaturePolicyPhase::COARSE ?
        frame.mvPreviousB4State : frame.mvCurrentB4State;
    for (int i = 0; i < frame.N; ++i) {
        switch (frame.mvSemanticRegion[i]) {
        case SemanticRegion::BACKGROUND:
            proposed[i] = FeatureUsePolicy::USE_NORMAL;
            break;
        case SemanticRegion::PERSON_BOUNDARY:
            proposed[i] = FeatureUsePolicy::EXCLUDE_BOUNDARY;
            break;
        case SemanticRegion::PERSON_INTERIOR:
            if (HasOwner(frame,i)) proposed[i] = StatePolicy(states[i]);
            break;
        default: break;
        }
    }
    return proposed;
}

FeaturePolicyStatistics SummarizeFeaturePolicies(const Frame& frame,
    const std::vector<FeatureUsePolicy>& coarseSnapshot, bool allowAppliedCoarse) {
    Validate(frame,allowAppliedCoarse);
    // Previous state is a frozen snapshot. Catch accidental overwrites/reuse.
    if (coarseSnapshot != ComputeFeaturePolicies(frame,FeaturePolicyPhase::COARSE,allowAppliedCoarse))
        throw std::runtime_error("FeaturePolicy coarse snapshot changed across Tracking/B4");
    if (allowAppliedCoarse && std::any_of(frame.mvDynamicExcluded.begin(),frame.mvDynamicExcluded.end(),
        [](bool value){return value;}))
        for (int i=0;i<frame.N;++i)
            if (frame.mvFeaturePolicy[i]!=coarseSnapshot[i] ||
                frame.mvDynamicExcluded[i]==FeaturePolicyAllows(coarseSnapshot[i],FeaturePolicyPhase::COARSE))
                throw std::runtime_error("Applied coarse metadata differs from the previous-state snapshot");
    const auto fine = ComputeFeaturePolicies(frame,FeaturePolicyPhase::FINE,allowAppliedCoarse);
    const auto mapping = ComputeFeaturePolicies(frame,FeaturePolicyPhase::MAPPING,allowAppliedCoarse);
    FeaturePolicyStatistics stats;
    for (int i = 0; i < frame.N; ++i) {
        stats.coarseAllowed += FeaturePolicyAllows(coarseSnapshot[i],FeaturePolicyPhase::COARSE);
        stats.fineAllowed += FeaturePolicyAllows(fine[i],FeaturePolicyPhase::FINE);
        stats.mapAllowed += FeaturePolicyAllows(mapping[i],FeaturePolicyPhase::MAPPING);
        const auto region = frame.mvSemanticRegion[i];
        if (region == SemanticRegion::BACKGROUND) ++stats.background;
        else if (region == SemanticRegion::PERSON_BOUNDARY) ++stats.boundary;
        else if (region != SemanticRegion::PERSON_INTERIOR || !HasOwner(frame,i)) ++stats.unknown;
        else {
            switch (frame.mvCurrentB4State[i]) {
            case TemporalState::STATIC: ++stats.stationary; break;
            case TemporalState::PENDING: ++stats.pending; break;
            case TemporalState::DYNAMIC: ++stats.dynamic; break;
            default: ++stats.unknown; break;
            }
            switch (frame.mvPreviousB4State[i]) {
            case TemporalState::STATIC: ++stats.previousStatic; break;
            case TemporalState::PENDING: ++stats.previousPending; break;
            case TemporalState::DYNAMIC: ++stats.previousDynamic; break;
            default: ++stats.previousUnknown; break;
            }
        }
    }
    if (stats.background + stats.stationary + stats.pending + stats.dynamic + stats.boundary + stats.unknown != frame.N ||
        stats.coarseAllowed != stats.background + stats.previousStatic ||
        stats.fineAllowed != stats.background + stats.stationary || stats.mapAllowed != stats.fineAllowed)
        throw std::runtime_error("FeaturePolicy summary invariants failed");
    return stats;
}

void WriteFeaturePolicyCsvHeader(std::ostream& stream) {
    stream << "frame_id,timestamp,N,coarse_allowed_count,fine_allowed_count,map_allowed_count,"
        "background_count,S_count,P_count,D_count,boundary_count,unknown_count,"
        "previous_S_count,previous_P_count,previous_D_count,previous_unknown_count,"
        "evidence_only,policy_applied,actual_excluded_count\n";
}
void WriteFeaturePolicyCsv(std::ostream& stream, const Frame& frame, const FeaturePolicyStatistics& s,
    bool coarseActionEnabled) {
    const int excluded = std::count(frame.mvDynamicExcluded.begin(),frame.mvDynamicExcluded.end(),true);
    stream << frame.mnId << ',' << std::setprecision(17) << frame.mTimeStamp << ',' << frame.N << ','
        << s.coarseAllowed << ',' << s.fineAllowed << ',' << s.mapAllowed << ',' << s.background << ','
        << s.stationary << ',' << s.pending << ',' << s.dynamic << ',' << s.boundary << ',' << s.unknown << ','
        << s.previousStatic << ',' << s.previousPending << ',' << s.previousDynamic << ',' << s.previousUnknown
        << ',' << int(!coarseActionEnabled) << ',' << int(excluded>0) << ',' << excluded << '\n';
    if (!stream) throw std::runtime_error("Cannot write FeaturePolicy summary CSV");
}
}
