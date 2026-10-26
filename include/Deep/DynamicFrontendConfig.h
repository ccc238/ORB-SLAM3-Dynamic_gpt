#ifndef ORB_SLAM3_DYNAMIC_FRONTEND_CONFIG_H
#define ORB_SLAM3_DYNAMIC_FRONTEND_CONFIG_H
#include "FeatureSemanticTypes.h"

namespace ORB_SLAM3 {

enum class DynamicFrontendMode {
    ORIGINAL = 0,
    B1_HARD_MASK = 1,
    TWO_STAGE_EVIDENCE_ONLY = 2,
    TWO_STAGE_ACTION = 3
};

inline const char* DynamicFrontendModeName(DynamicFrontendMode mode)
{
    switch (mode) {
    case DynamicFrontendMode::ORIGINAL: return "ORIGINAL";
    case DynamicFrontendMode::B1_HARD_MASK: return "B1_HARD_MASK";
    case DynamicFrontendMode::TWO_STAGE_EVIDENCE_ONLY: return "TWO_STAGE_EVIDENCE_ONLY";
    case DynamicFrontendMode::TWO_STAGE_ACTION: return "TWO_STAGE_ACTION";
    }
    return "INVALID";
}

struct DynamicFrontendConfig {
    // Absent Mode keeps pre-existing YAML on its explicitly logged legacy
    // path, rather than silently reinterpreting B2/B3/B4-A configurations.
    bool explicitMode = false;
    DynamicFrontendMode mode = DynamicFrontendMode::ORIGINAL;
    bool twoStageEnable = false;
    bool coarseFilterEnable = false;
    bool useCoarsePoseForB2 = false;
    bool finePolicyEnable = false;
    bool mapAdmissionEnable = false;
    bool fallbackEnable = true;
    bool diagnosticSaveCsv = false;
    bool diagnosticShowOverlay = false;
    bool diagnosticSaveOverlay = false;
    bool CollectDiagnostics() const {
        return diagnosticSaveCsv || diagnosticShowOverlay || diagnosticSaveOverlay;
    }
    int minCoarseInliers = 30;
    float minCoarseInlierRatio = 0.30f;
    FeatureSemanticConfig featureSemantics;
    FeaturePolicyConfig featurePolicy;
};

} // namespace ORB_SLAM3
#endif
