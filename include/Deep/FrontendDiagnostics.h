#ifndef ORB_SLAM3_FRONTEND_DIAGNOSTICS_H
#define ORB_SLAM3_FRONTEND_DIAGNOSTICS_H

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "FeatureSemanticTypes.h"

namespace ORB_SLAM3 {

enum B2ViewerPointClass {
    B2_VIEW_UNMATCHED = 0,
    B2_VIEW_INVALID = 1,
    B2_VIEW_LOW_RESIDUAL = 2,
    B2_VIEW_HIGH_RESIDUAL = 3,
    B2_VIEW_BOUNDARY = 4
};

struct B2ViewerPoint {
    cv::Point2f pixel;
    B2ViewerPointClass pointClass = B2_VIEW_UNMATCHED;
};

// Viewer-only B3 record. The drawing thread receives no B3 object that can
// reach Tracking, Frame, a KeyFrame, or a MapPoint.
struct B3ViewerInstance {
    cv::Rect boundingBox;
    cv::Scalar color;
    std::string primaryLabel;
    std::string secondaryLabel;
    std::string tertiaryLabel;
    std::string quaternaryLabel;
};

// Snapshot copied from Tracking to FrameDrawer after each input frame.
struct FrontendDiagnostics {
    unsigned long frameId = 0;
    double timestamp = 0.0;
    double yoloInferenceMs = 0.0;
    double personMaskRatio = 0.0;
    int rawFastCandidates = 0;
    int maskedRejected = 0;
    int finalOrbFeatures = 0;
    int matches = 0;
    int poseInliers = 0;
    int trackingState = -1;
    bool becameKeyFrame = false;
    // Evidence-only B2 summary for FrameDrawer. It is never consumed by
    // Tracking, local mapping, or optimization.
    std::string b2EvidenceSummary;
    std::vector<std::string> b2InstanceEvidenceSummaries;
    bool b2OverlayEnabled = false;
    std::vector<B2ViewerPoint> b2Points;
    int b2Candidates = 0;
    int b2Matched = 0;
    int b2Valid = 0;
    float b2MedianR3D = -1.0f;
    float b2MedianNormalizedR3D = -1.0f;
    double b2ProcessingMs = 0.0;
    bool b3OverlayEnabled = false;
    std::string b3EvidenceSummary;
    std::vector<std::string> b3InstanceEvidenceSummaries;
    std::vector<B3ViewerInstance> b3Instances;
    bool b4OverlayEnabled = false;
    std::string b4EvidenceSummary;
    bool stateActionOverlayEnabled = false;
    std::vector<TemporalState> currentFeatureStates;
    std::vector<B3ViewerInstance> stateActionInstances;
    std::string stageTimingSummary;
};

}
#endif
