#ifndef ORB_SLAM3_FEATURE_SEMANTIC_TYPES_H
#define ORB_SLAM3_FEATURE_SEMANTIC_TYPES_H
namespace ORB_SLAM3 {
enum class SemanticRegion { BACKGROUND, PERSON_INTERIOR, PERSON_BOUNDARY, UNKNOWN };
enum class TemporalState { STATIC, PENDING, DYNAMIC, UNKNOWN };
enum class FeatureUsePolicy {
    USE_NORMAL, USE_POSE_ONLY, EXCLUDE_PENDING, EXCLUDE_DYNAMIC,
    EXCLUDE_BOUNDARY, EXCLUDE_UNKNOWN
};
struct FeatureSemanticConfig {
    bool enable = false; // Existing experiment YAML remains annotation-disabled.
    float safeBoundaryDistancePixels = 3.f;
    bool depthEdgeEnable = true;
    int depthEdgeRadius = 1;
    float depthEdgeAbsoluteMeters = .05f;
    float depthEdgeRelative = .02f;
    bool showOverlay = true;
};
struct FeaturePolicyConfig {
    bool enable = false;
    bool evidenceOnly = true;
    bool saveSummaryCsv = true;
    bool printFrameSummary = true;
};
}
#endif
