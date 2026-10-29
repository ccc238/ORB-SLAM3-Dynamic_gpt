#ifndef ORB_SLAM3_B4_CONFIG_H
#define ORB_SLAM3_B4_CONFIG_H
#include <cmath>
#include <stdexcept>

namespace ORB_SLAM3 {

// Kept dependency-free so the shared YAML reader does not introduce a
// B2/B3/B4 include cycle.
struct B4Config {
    bool enable = false;
    bool evidenceOnly = true;
    bool associationEnable = true;
    int dynamicEnterCount = 3;
    int staticEnterCount = 6;
    int dynamicRecoveryCount = 10;
    int trackTTL = 15;
    bool evidenceExpiryEnable = true;
    double maxEvidenceGapSeconds = 0.20;
    double maxStaticAgeSeconds = 0.30;
    bool useContinuousQuality = false;
    float qualityMin = 0.65f;
    // B4-A does not approximate mask IoU from a bounding box. The setting is
    // retained for reproducibility until masks enter the B3-to-B4 interface.
    float minMaskIoU = 0.05f;
    float minBoxIoU = 0.10f;
    float maxCenterDistance = 0.15f; // normalized by image diagonal
    bool pendingUseForPose = false;
    bool pendingAllowMapCreation = false;
    bool dynamicAllowMapCreation = false;
    bool printFrameSummary = true;
    bool saveSummaryCsv = true;
    bool showOverlay = true;
};

inline void ValidateB4EvidenceExpiry(const B4Config& config) {
    if (!std::isfinite(config.maxEvidenceGapSeconds) || config.maxEvidenceGapSeconds <= 0.0 ||
        !std::isfinite(config.maxStaticAgeSeconds) || config.maxStaticAgeSeconds <= 0.0 ||
        config.maxEvidenceGapSeconds > config.maxStaticAgeSeconds)
        throw std::runtime_error("B4.EvidenceExpiry requires finite positive MaxGapSeconds <= MaxStaticAgeSeconds");
}

} // namespace ORB_SLAM3
#endif
