#ifndef ORB_SLAM3_FEATURE_POLICY_EVALUATOR_H
#define ORB_SLAM3_FEATURE_POLICY_EVALUATOR_H
#include "Frame.h"
#include <ostream>

namespace ORB_SLAM3 {
enum class FeaturePolicyPhase { COARSE, FINE, MAPPING };

// Proposed policies only. No mutable Frame/Map/Tracking pointer is retained.
std::vector<FeatureUsePolicy> ComputeFeaturePolicies(const Frame& frame, FeaturePolicyPhase phase,
    bool allowAppliedCoarse = false);
bool FeaturePolicyAllows(FeatureUsePolicy policy, FeaturePolicyPhase phase);

struct FeaturePolicyStatistics {
    int coarseAllowed = 0, fineAllowed = 0, mapAllowed = 0;
    // Disjoint partition using CURRENT states for valid person interiors.
    int background = 0, stationary = 0, pending = 0, dynamic = 0, boundary = 0, unknown = 0;
    int previousStatic = 0, previousPending = 0, previousDynamic = 0, previousUnknown = 0;
};
FeaturePolicyStatistics SummarizeFeaturePolicies(const Frame& frame,
    const std::vector<FeatureUsePolicy>& coarseSnapshot, bool allowAppliedCoarse = false);
void WriteFeaturePolicyCsvHeader(std::ostream& stream);
void WriteFeaturePolicyCsv(std::ostream& stream, const Frame& frame, const FeaturePolicyStatistics& stats,
    bool coarseActionEnabled = false);
}
#endif
