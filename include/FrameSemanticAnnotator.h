#ifndef ORB_SLAM3_FRAME_SEMANTIC_ANNOTATOR_H
#define ORB_SLAM3_FRAME_SEMANTIC_ANNOTATOR_H
#include "Frame.h"
#include "YOLOSeg.h"
#include <unordered_map>
namespace ORB_SLAM3 {
// Writes ONLY the new metadata. Uses mvKeys (raw pixels), not mvKeysUn.
void AnnotateRGBDFeatures(Frame& frame, const cv::Size& imageSize,
    const cv::Mat& depthMeters, const std::vector<YOLOSegInstance>& persons,
    const std::unordered_map<int, TemporalState>& previousStates,
    const FeatureSemanticConfig& config);
void AssertRGBDFeatureAlignment(const Frame& frame);
void UpdateFeatureTemporalStates(Frame& frame,
    const std::unordered_map<int, TemporalState>& currentStates);
}
#endif
