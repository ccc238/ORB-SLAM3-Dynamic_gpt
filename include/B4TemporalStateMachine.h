/** Read-only B4-A temporal evidence state machine. */
#ifndef ORB_SLAM3_B4_TEMPORAL_STATE_MACHINE_H
#define ORB_SLAM3_B4_TEMPORAL_STATE_MACHINE_H

#include "B3SemanticGeometryFusion.h"
#include "Deep/B4Config.h"
#include <opencv2/core/core.hpp>
#include <string>
#include <limits>
#include <unordered_map>
#include <vector>

namespace ORB_SLAM3 {

enum class B4MotionState { STATIC, PENDING, DYNAMIC };
enum class B4EvidenceType { RELIABLE_STATIC, RELIABLE_MOVING, UNRELIABLE };
const char* B4MotionStateName(B4MotionState state);
const char* B4EvidenceTypeName(B4EvidenceType type);

struct B4TrackState {
    int track_id = -1;
    B4MotionState state = B4MotionState::PENDING;
    int static_count = 0;
    int dynamic_count = 0;
    int recovery_count = 0;
    int unreliable_count = 0;
    int age = 0;
    int missed_frames = 0;
    int first_frame_id = -1;
    int last_frame_id = -1;
    cv::Rect2f last_bbox;
    cv::Point2f last_center;
    cv::Mat last_mask;
    float last_quality = -1.0f;
    std::string transition_reason;
    bool hasReliableEvidence = false;
    double lastReliableTimestamp = std::numeric_limits<double>::quiet_NaN();
};

struct B4StateResult {
    long frameId = -1;
    double timestamp = 0.0;
    int instanceId = -1;
    int sourceB2TrackId = -1;
    int trackId = -1; // Shared InstanceTrackManager identity.
    cv::Rect boundingBox;

    B4MotionState previousState = B4MotionState::PENDING;
    B4MotionState state = B4MotionState::PENDING;
    bool stateTransition = false;
    // History at the current acquisition time, BEFORE this frame's evidence.
    B4MotionState stateAfterExpiry = B4MotionState::PENDING;
    bool hasReliableHistory = false;
    double evidenceAgeSeconds = std::numeric_limits<double>::quiet_NaN();
    bool counterExpired = false;
    bool countersReset = false; // At least one nonzero Ns/Nd/Nr was cleared.
    bool staticStateExpired = false;
    std::string expiryReason = "DISABLED";
    std::string reasonCode;
    B4EvidenceType evidenceType = B4EvidenceType::UNRELIABLE;
    int nDynamic = 0, nStatic = 0, nRecovery = 0, nUnreliable = 0;
    int ageFrames = 0, missedFrames = 0;

    bool geometryReliable = false;
    GeometryState geometryState = GEOMETRY_STATE_NOT_EVALUATED;
    B3FusionCategory fusionCategory = B3_FUSION_NOT_EVALUATED;
    bool b3Reliable = false;
    std::string b3Reason;
    std::string recommendedPolicy;
    std::string appliedPolicy;
    bool evidenceOnly = true;
    // No current YOLO instance: display only the retained track until TTL.
    bool isLost = false;
};

class B4TemporalStateMachine {
public:
    explicit B4TemporalStateMachine(const B4Config& config) : mConfig(config) {
        ValidateB4EvidenceExpiry(config);
    }
    // Read history only: no association, TTL advancement or counter updates.
    bool GetState(int trackId, B4MotionState& state) const;
    bool GetUsableState(int trackId, double timestamp, B4MotionState& state) const;
    // Read-only preflight, also used before RGB-D inference/association.
    void ValidateTimestamp(double timestamp) const;
    void Reset();

    // Advances the state machine once per input frame. B3 values are copied;
    // they are never modified and never escape through mutable references.
    std::vector<B4StateResult> Process(long frameId, double timestamp,
                                       const std::vector<B3FusionResult>& instances,
                                       float imageDiagonalPixels);

private:
    // Only reliable person-semantic B3 categories may advance this person's
    // temporal state.  Every other B3 outcome is deliberately a freeze.
    B4EvidenceType ConvertB3ToB4Evidence(const B3FusionResult& b3) const;
    void UpdateB4State(B4TrackState& track, B4EvidenceType evidence) const;
    B4StateResult ApplyEvidenceExpiry(B4TrackState& track, double timestamp) const;

    B4Config mConfig;
    std::unordered_map<int, B4TrackState> mB4Tracks;
    // A frame counter may restart on an explicit SLAM reset; timestamp is
    // part of the input identity so a later image is not a duplicate frame.
    bool mbHasProcessedFrame = false;
    long mLastProcessedFrameId = -1;
    double mLastProcessedTimestamp = 0.0;
};

} // namespace ORB_SLAM3
#endif
