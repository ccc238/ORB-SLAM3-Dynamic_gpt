/**
 * B3 semantic--geometry fusion result types.
 *
 * These are deliberately value-only records.  They provide an explicit
 * boundary between B2 evidence and a future B3 feature policy: neither a
 * Frame nor a MapPoint can be reached through a B3FusionResult.
 */
#ifndef ORB_SLAM3_B3_SEMANTIC_GEOMETRY_FUSION_H
#define ORB_SLAM3_B3_SEMANTIC_GEOMETRY_FUSION_H

#include "B2MotionEvidence.h"

#include <string>

namespace ORB_SLAM3 {

enum GeometryState {
    GEOMETRY_STATE_NOT_EVALUATED = 0,
    GEOMETRY_STATE_UNRELIABLE = 1,
    GEOMETRY_STATE_STATIC_CANDIDATE = 2,
    GEOMETRY_STATE_MOVING_CANDIDATE = 3,
    GEOMETRY_STATE_UNCERTAIN = 4
};

// The category is intentionally separate from GeometryState.  A later B3
// fusion rule may consider a semantic prior, geometry, or both without
// changing the raw B2 decision stored above.
enum B3FusionCategory {
    B3_FUSION_NOT_EVALUATED = 0,
    B3_FUSION_UNRELIABLE = 1,
    B3_SEM_DYNAMIC_GEOM_MOVING = 2,
    B3_SEM_DYNAMIC_GEOM_STATIC = 3,
    B3_SEM_STATIC_GEOM_MOVING = 4,
    B3_SEM_STATIC_GEOM_STATIC = 5
};

const char* GeometryStateName(GeometryState state);
const char* B3FusionCategoryName(B3FusionCategory category);

// A value copy of the semantic information B3 needs.  trackId is populated
// only when a same-frame B2 instance has been associated successfully.
struct SemanticInstance {
    long frameId = -1;
    double timestamp = 0.0;
    int instanceId = -1;
    int trackId = -1;
    int classId = -1;
    std::string className;
    float confidence = 0.0f;
    cv::Rect boundingBox;
};

struct B3FusionResult {
    long frame_id = -1;
    double timestamp = 0.0;

    int instance_id = -1;
    int track_id = -1;
    cv::Rect bounding_box;

    int semantic_class_id = -1;
    std::string semantic_class_name;
    float semantic_confidence = 0.0f;
    bool semantic_dynamic_prior = false;

    bool geometry_reliable = false;
    GeometryState geometry_state = GEOMETRY_STATE_NOT_EVALUATED;
    std::string geometry_reason;

    int candidate_count = 0;
    int matched_count = 0;
    int valid_count = 0;

    float match_ratio = 0.0f;
    float valid_match_ratio = 0.0f;

    float raw_r3d_median = -1.0f;
    float norm_r3d_median = -1.0f;
    float high_ratio = -1.0f;

    B3FusionCategory fusion_category = B3_FUSION_NOT_EVALUATED;
    bool fusion_reliable = false;

    std::string fusion_reason;
    std::string recommended_action;
    std::string applied_action;
};

class B3Fusion {
public:
    B3Fusion(bool evidenceOnly, bool personAsDynamicPrior)
        : mbEvidenceOnly(evidenceOnly), mbPersonAsDynamicPrior(personAsDynamicPrior) {}

    // The geometry pointer may be null for a YOLO instance with no B2
    // candidate/result in the current frame.  The method never mutates B2,
    // Tracking, Frame, or any SLAM map object.
    B3FusionResult Fuse(const SemanticInstance& semantic,
                         const B2PersonInstance* geometry) const;

private:
    bool mbEvidenceOnly = true;
    bool mbPersonAsDynamicPrior = true;
};

} // namespace ORB_SLAM3

#endif
