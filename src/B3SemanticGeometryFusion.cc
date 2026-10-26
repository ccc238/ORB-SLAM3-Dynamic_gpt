#include "B3SemanticGeometryFusion.h"

namespace ORB_SLAM3 {
namespace {
GeometryState ToGeometryState(B2InstanceGeometryStatus state)
{
    switch (state)
    {
    case GEOM_UNRELIABLE: return GEOMETRY_STATE_UNRELIABLE;
    case GEOM_STATIC_CANDIDATE: return GEOMETRY_STATE_STATIC_CANDIDATE;
    case GEOM_MOVING_CANDIDATE: return GEOMETRY_STATE_MOVING_CANDIDATE;
    case GEOM_UNCERTAIN: return GEOMETRY_STATE_UNCERTAIN;
    default: return GEOMETRY_STATE_NOT_EVALUATED;
    }
}

const char* GeometryReason(B2InstanceGeometryStatus state,
                           B2GeometryFailureReason reason)
{
    if (state == GEOM_STATIC_CANDIDATE) return "GEOMETRY_STATIC_EVIDENCE";
    if (state == GEOM_MOVING_CANDIDATE) return "GEOMETRY_MOVING_EVIDENCE";
    if (state == GEOM_UNCERTAIN) return "AMBIGUOUS_GEOMETRIC_EVIDENCE";
    switch (reason)
    {
    case INSUFFICIENT_CANDIDATES: return "INSUFFICIENT_CANDIDATES";
    case INSUFFICIENT_MATCHES: return "INSUFFICIENT_MATCHES";
    case LOW_MATCH_RATIO: return "LOW_MATCH_RATIO";
    case INSUFFICIENT_VALID_DEPTH: return "INSUFFICIENT_VALID_DEPTH";
    case LOW_VALID_MATCH_RATIO: return "LOW_VALID_MATCH_RATIO";
    case INSUFFICIENT_BACKGROUND_REFERENCE: return "INSUFFICIENT_BACKGROUND_REFERENCE";
    case AMBIGUOUS_GEOMETRIC_EVIDENCE: return "AMBIGUOUS_GEOMETRIC_EVIDENCE";
    case POSE_WEAK: return "POSE_WEAK";
    default: return "GEOMETRY_UNRELIABLE";
    }
}
} // namespace

const char* GeometryStateName(GeometryState state)
{
    switch (state)
    {
    case GEOMETRY_STATE_STATIC_CANDIDATE: return "STATIC";
    case GEOMETRY_STATE_MOVING_CANDIDATE: return "MOVING";
    case GEOMETRY_STATE_UNCERTAIN: return "UNCERTAIN";
    case GEOMETRY_STATE_UNRELIABLE: return "UNRELIABLE";
    default: return "NOT_EVALUATED";
    }
}

const char* B3FusionCategoryName(B3FusionCategory category)
{
    switch (category)
    {
    case B3_SEM_DYNAMIC_GEOM_MOVING: return "SEM_DYNAMIC_GEOM_MOVING";
    case B3_SEM_DYNAMIC_GEOM_STATIC: return "SEM_DYNAMIC_GEOM_STATIC";
    case B3_SEM_STATIC_GEOM_MOVING: return "SEM_STATIC_GEOM_MOVING";
    case B3_SEM_STATIC_GEOM_STATIC: return "SEM_STATIC_GEOM_STATIC";
    case B3_FUSION_UNRELIABLE: return "FUSION_UNRELIABLE";
    default: return "FUSION_NOT_EVALUATED";
    }
}

B3FusionResult B3Fusion::Fuse(const SemanticInstance& semantic,
                               const B2PersonInstance* geometry) const
{
    B3FusionResult result;
    result.frame_id = semantic.frameId;
    result.timestamp = semantic.timestamp;
    result.instance_id = semantic.instanceId;
    result.track_id = semantic.trackId;
    result.bounding_box = semantic.boundingBox;
    result.semantic_class_id = semantic.classId;
    result.semantic_class_name = semantic.className;
    result.semantic_confidence = semantic.confidence;
    result.semantic_dynamic_prior =
        mbPersonAsDynamicPrior && semantic.className == "person";

    if (!geometry)
    {
        result.geometry_reliable = false;
        result.fusion_reliable = false;
        result.fusion_category = B3_FUSION_UNRELIABLE;
        result.geometry_reason = "NO_B2_RESULT";
        result.fusion_reason = "NO_GEOMETRY_EVIDENCE";
        result.recommended_action = "NO_HARD_DECISION";
        result.applied_action = "NONE_EVIDENCE_ONLY";
        return result;
    }

    result.track_id = geometry->trackId;
    result.geometry_state = ToGeometryState(geometry->geometryDecision);
    result.geometry_reason = GeometryReason(geometry->geometryDecision,
                                            geometry->geometryFailureReason);
    result.candidate_count = geometry->candidateCount;
    result.matched_count = geometry->matchedCount;
    result.valid_count = geometry->validGeometryCount;
    result.match_ratio = geometry->matchRatio;
    result.valid_match_ratio = geometry->validMatchRatio;
    result.raw_r3d_median = geometry->rawR3DMedian;
    result.norm_r3d_median = geometry->normalizedR3DMedian;
    result.high_ratio = geometry->movingFraction;
    result.geometry_reliable =
        result.geometry_state == GEOMETRY_STATE_STATIC_CANDIDATE ||
        result.geometry_state == GEOMETRY_STATE_MOVING_CANDIDATE;

    if (!result.geometry_reliable)
    {
        result.fusion_category = B3_FUSION_UNRELIABLE;
        result.fusion_reliable = false;
        result.fusion_reason = result.geometry_reason;
        result.recommended_action = "NO_HARD_DECISION";
    }
    else if (result.semantic_dynamic_prior &&
             result.geometry_state == GEOMETRY_STATE_MOVING_CANDIDATE)
    {
        result.fusion_category = B3_SEM_DYNAMIC_GEOM_MOVING;
        result.fusion_reliable = true;
        result.fusion_reason = "PERSON_GEOM_MOVING";
        result.recommended_action = "DYNAMIC_CANDIDATE";
    }
    else if (result.semantic_dynamic_prior &&
             result.geometry_state == GEOMETRY_STATE_STATIC_CANDIDATE)
    {
        result.fusion_category = B3_SEM_DYNAMIC_GEOM_STATIC;
        result.fusion_reliable = true;
        result.fusion_reason = "PERSON_GEOM_STATIC";
        result.recommended_action = "DEFER_TO_TEMPORAL";
    }
    else if (!result.semantic_dynamic_prior &&
             result.geometry_state == GEOMETRY_STATE_MOVING_CANDIDATE)
    {
        result.fusion_category = B3_SEM_STATIC_GEOM_MOVING;
        result.fusion_reliable = true;
        result.fusion_reason = "NONPERSON_GEOM_MOVING";
        result.recommended_action = "RECORD_ONLY";
    }
    else
    {
        result.fusion_category = B3_SEM_STATIC_GEOM_STATIC;
        result.fusion_reliable = true;
        result.fusion_reason = "NONPERSON_GEOM_STATIC";
        result.recommended_action = "USE_NORMALLY";
    }

    result.applied_action = mbEvidenceOnly ?
        "NONE_EVIDENCE_ONLY" : result.recommended_action;
    return result;
}

} // namespace ORB_SLAM3
