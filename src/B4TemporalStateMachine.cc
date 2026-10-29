#include "B4TemporalStateMachine.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace ORB_SLAM3 {

const char* B4MotionStateName(B4MotionState state) {
    switch (state) {
    case B4MotionState::STATIC: return "S";
    case B4MotionState::DYNAMIC: return "D";
    default: return "P";
    }
}
const char* B4EvidenceTypeName(B4EvidenceType type) {
    switch (type) {
    case B4EvidenceType::RELIABLE_STATIC: return "RELIABLE_STATIC";
    case B4EvidenceType::RELIABLE_MOVING: return "RELIABLE_MOVING";
    default: return "UNRELIABLE";
    }
}

namespace {
const char* RecommendedPolicy(B4MotionState state) {
    switch (state) {
    case B4MotionState::STATIC: return "USE_NORMAL";
    case B4MotionState::DYNAMIC: return "EXCLUDE_DYNAMIC";
    default: return "EXCLUDE_PENDING";
    }
}

} // namespace

B4EvidenceType B4TemporalStateMachine::ConvertB3ToB4Evidence(const B3FusionResult& b3) const {
    if (!b3.fusion_reliable)
        return B4EvidenceType::UNRELIABLE;

    // B4 tracks people.  A non-person geometric result is useful diagnostic
    // data, but must never advance a person's S/P/D state machine.
    if (b3.fusion_category == B3_SEM_DYNAMIC_GEOM_MOVING)
        return B4EvidenceType::RELIABLE_MOVING;
    if (b3.fusion_category == B3_SEM_DYNAMIC_GEOM_STATIC)
        return B4EvidenceType::RELIABLE_STATIC;
    return B4EvidenceType::UNRELIABLE;
}

void B4TemporalStateMachine::UpdateB4State(B4TrackState& track,
                                            B4EvidenceType evidence) const {
    // Unreliable geometry is explicitly not negative evidence.  In
    // particular, it must not reset any of the reliable-evidence counters.
    if (evidence == B4EvidenceType::UNRELIABLE) {
        ++track.unreliable_count;
        track.transition_reason = "UNRELIABLE_EVIDENCE_FREEZE";
        return;
    }

    switch (track.state) {
    case B4MotionState::PENDING:
        if (evidence == B4EvidenceType::RELIABLE_MOVING) {
            ++track.dynamic_count;
            track.static_count = 0;
            if (track.dynamic_count >= mConfig.dynamicEnterCount) {
                track.state = B4MotionState::DYNAMIC;
                track.recovery_count = 0;
                track.transition_reason = "P_TO_D";
            } else {
                track.transition_reason = "P_ACCUMULATE_MOVING";
            }
        } else { // RELIABLE_STATIC
            ++track.static_count;
            track.dynamic_count = 0;
            if (track.static_count >= mConfig.staticEnterCount) {
                track.state = B4MotionState::STATIC;
                track.transition_reason = "P_TO_S";
            } else {
                track.transition_reason = "P_ACCUMULATE_STATIC";
            }
        }
        break;

    case B4MotionState::STATIC:
        if (evidence == B4EvidenceType::RELIABLE_MOVING) {
            track.state = B4MotionState::PENDING;
            track.dynamic_count = 1;
            track.static_count = 0;
            track.transition_reason = "S_TO_P";
        } else { // RELIABLE_STATIC
            track.dynamic_count = 0;
            track.transition_reason = "S_STAY";
        }
        break;

    case B4MotionState::DYNAMIC:
        if (evidence == B4EvidenceType::RELIABLE_MOVING) {
            track.recovery_count = 0;
            track.transition_reason = "D_STAY";
        } else { // RELIABLE_STATIC
            ++track.recovery_count;
            if (track.recovery_count >= mConfig.dynamicRecoveryCount) {
                // Dynamic tracks must recover through P, then accrue a fresh
                // static sequence before they are allowed to become S.
                track.state = B4MotionState::PENDING;
                track.static_count = 0;
                track.dynamic_count = 0;
                track.recovery_count = 0;
                track.transition_reason = "D_TO_P";
            } else {
                track.transition_reason = "D_ACCUMULATE_RECOVERY";
            }
        }
        break;
    }
}

bool B4TemporalStateMachine::GetState(int trackId, B4MotionState& state) const {
    const auto found = mB4Tracks.find(trackId);
    if (found == mB4Tracks.end()) return false;
    state = found->second.state;
    return true;
}

void B4TemporalStateMachine::ValidateTimestamp(double timestamp) const {
    if (!std::isfinite(timestamp))
        throw std::runtime_error("B4 requires a finite image acquisition timestamp");
    if (mConfig.evidenceExpiryEnable && mbHasProcessedFrame && timestamp < mLastProcessedTimestamp)
        throw std::runtime_error("B4 timestamp moved backwards; Reset() is required before a new sequence");
}

void B4TemporalStateMachine::Reset() {
    mB4Tracks.clear();
    mbHasProcessedFrame = false;
    mLastProcessedFrameId = -1;
    mLastProcessedTimestamp = 0.0;
}

B4StateResult B4TemporalStateMachine::ApplyEvidenceExpiry(B4TrackState& track,
                                                         double timestamp) const {
    B4StateResult history;
    history.previousState = track.state;
    history.hasReliableHistory = track.hasReliableEvidence;
    if (track.hasReliableEvidence)
        history.evidenceAgeSeconds = timestamp - track.lastReliableTimestamp;
    history.expiryReason = mConfig.evidenceExpiryEnable ? "NO_RELIABLE_HISTORY" : "DISABLED";
    if (mConfig.evidenceExpiryEnable && track.hasReliableEvidence) {
        history.expiryReason = "NONE";
        // Compare acquisition timestamps directly. Strict >, with no epsilon
        // or wall-clock substitution; equality at the threshold stays valid.
        if (timestamp > track.lastReliableTimestamp + mConfig.maxEvidenceGapSeconds) {
            history.counterExpired = true;
            history.countersReset = track.static_count || track.dynamic_count || track.recovery_count;
            track.static_count = track.dynamic_count = track.recovery_count = 0;
            history.expiryReason = "COUNTER_GAP_EXPIRED";
        }
        if (track.state == B4MotionState::STATIC &&
            timestamp > track.lastReliableTimestamp + mConfig.maxStaticAgeSeconds) {
            history.staticStateExpired = true;
            history.countersReset = history.countersReset || track.static_count ||
                track.dynamic_count || track.recovery_count;
            track.state = B4MotionState::PENDING;
            track.static_count = track.dynamic_count = track.recovery_count = 0;
            history.expiryReason += ";STATIC_STATE_EXPIRED";
        }
    }
    history.stateAfterExpiry = track.state;
    return history;
}

bool B4TemporalStateMachine::GetUsableState(int trackId, double timestamp,
                                           B4MotionState& state) const {
    ValidateTimestamp(timestamp);
    const auto found = mB4Tracks.find(trackId);
    if (found == mB4Tracks.end()) return false;
    B4TrackState snapshot = found->second;
    ApplyEvidenceExpiry(snapshot, timestamp); // No mutation of live history/TTL.
    state = snapshot.state;
    return true;
}

std::vector<B4StateResult> B4TemporalStateMachine::Process(long frameId, double timestamp,
    const std::vector<B3FusionResult>& instances, float imageDiagonalPixels) {
    (void)imageDiagonalPixels; // association is already completed by InstanceTrackManager.
    // Validate the entire update before changing ages, TTL or evidence counts.
    // Duplicate dispatch must not silently accumulate evidence twice.
    if (mbHasProcessedFrame && frameId == mLastProcessedFrameId &&
        timestamp == mLastProcessedTimestamp)
        throw std::runtime_error("B4 duplicate UpdateTrackState for the same input frame");
    ValidateTimestamp(timestamp);
    if (mConfig.evidenceExpiryEnable && mbHasProcessedFrame &&
        (timestamp == mLastProcessedTimestamp || frameId < mLastProcessedFrameId))
        throw std::runtime_error("B4 requires advancing image timestamps/frame IDs; Reset() before restarting a sequence");
    std::unordered_set<int> inputTrackIds;
    for (const B3FusionResult& instance : instances) {
        if (instance.track_id < 0)
            throw std::runtime_error("B4 received a YOLO instance without shared track_id");
        if (!inputTrackIds.insert(instance.track_id).second)
            throw std::runtime_error("B4 duplicate track_id in one input frame");
    }
    mbHasProcessedFrame = true;
    mLastProcessedFrameId = frameId;
    mLastProcessedTimestamp = timestamp;
    std::unordered_map<int, B4StateResult> history;
    for (auto& entry : mB4Tracks) {
        history.emplace(entry.first, ApplyEvidenceExpiry(entry.second, timestamp));
        ++entry.second.missed_frames;
    }
    std::vector<B4StateResult> output;
    for (size_t i = 0; i < instances.size(); ++i) {
        const B3FusionResult& instance = instances[i];
        const int id = instance.track_id;
        bool created = mB4Tracks.find(id) == mB4Tracks.end();
        if (created) {
            B4TrackState fresh; fresh.track_id = id; fresh.first_frame_id = frameId;
            mB4Tracks.emplace(id, fresh);
            history.emplace(id, ApplyEvidenceExpiry(mB4Tracks.at(id), timestamp));
        }
        B4TrackState& track = mB4Tracks.at(id);
        track.last_bbox = cv::Rect2f(instance.bounding_box);
        track.last_center = cv::Point2f(track.last_bbox.x + .5f*track.last_bbox.width,
                                        track.last_bbox.y + .5f*track.last_bbox.height);
        track.last_frame_id = frameId; track.missed_frames = 0; ++track.age;
        const B4EvidenceType evidence = ConvertB3ToB4Evidence(instance);
        UpdateB4State(track, evidence);
        if (evidence != B4EvidenceType::UNRELIABLE) {
            track.hasReliableEvidence = true;
            track.lastReliableTimestamp = timestamp;
        }
        if (created)
            track.transition_reason = "NEW_TO_P;" + track.transition_reason;
        B4StateResult result = history.at(id);
        result.frameId=frameId; result.timestamp=timestamp; result.instanceId=instance.instance_id;
        result.sourceB2TrackId=instance.track_id; result.trackId=id; result.boundingBox=instance.bounding_box;
        result.state=track.state; result.stateTransition=result.previousState != track.state;
        result.reasonCode=track.transition_reason; result.evidenceType=evidence; result.nDynamic=track.dynamic_count;
        result.nStatic=track.static_count; result.nRecovery=track.recovery_count; result.nUnreliable=track.unreliable_count;
        result.ageFrames=track.age; result.missedFrames=track.missed_frames; result.geometryReliable=instance.geometry_reliable;
        result.geometryState=instance.geometry_state; result.fusionCategory=instance.fusion_category;
        result.b3Reliable=instance.fusion_reliable; result.b3Reason=instance.fusion_reason;
        result.recommendedPolicy=RecommendedPolicy(track.state);
        result.appliedPolicy=mConfig.evidenceOnly ? "NONE_EVIDENCE_ONLY" : result.recommendedPolicy;
        result.evidenceOnly=mConfig.evidenceOnly;
        output.push_back(result);
    }
    for (auto it=mB4Tracks.begin(); it!=mB4Tracks.end();) {
        B4TrackState& track=it->second;
        if (track.missed_frames <= mConfig.trackTTL) {
            if (track.missed_frames > 0) {
                // No current evidence update; expiry was applied once above.
                B4StateResult lost = history.at(track.track_id);
                lost.frameId=frameId; lost.timestamp=timestamp; lost.trackId=track.track_id;
                lost.sourceB2TrackId=track.track_id; lost.boundingBox=track.last_bbox;
                lost.state=track.state;
                lost.stateTransition=lost.previousState != track.state;
                lost.reasonCode=mConfig.evidenceExpiryEnable ? "YOLO_MISSED_TTL_ACTIVE" : track.transition_reason;
                lost.evidenceType=B4EvidenceType::UNRELIABLE;
                lost.nDynamic=track.dynamic_count; lost.nStatic=track.static_count;
                lost.nRecovery=track.recovery_count; lost.nUnreliable=track.unreliable_count;
                lost.ageFrames=track.age; lost.missedFrames=track.missed_frames;
                lost.b3Reason="YOLO_MISSED_TTL_ACTIVE";
                lost.recommendedPolicy=RecommendedPolicy(track.state);
                lost.appliedPolicy=mConfig.evidenceOnly ? "NONE_EVIDENCE_ONLY" : lost.recommendedPolicy;
                lost.evidenceOnly=mConfig.evidenceOnly; lost.isLost=true;
                output.push_back(std::move(lost));
            }
            ++it;
            continue;
        }
        B4StateResult expired = history.at(track.track_id);
        expired.frameId=frameId; expired.timestamp=timestamp; expired.trackId=track.track_id;
        expired.sourceB2TrackId=track.track_id; expired.boundingBox=track.last_bbox;
        expired.state=track.state; expired.stateTransition=expired.previousState != track.state;
        expired.reasonCode="TRACK_TTL_EXPIRED";
        expired.nDynamic=track.dynamic_count; expired.nStatic=track.static_count; expired.nRecovery=track.recovery_count;
        expired.nUnreliable=track.unreliable_count; expired.ageFrames=track.age; expired.missedFrames=track.missed_frames;
        expired.b3Reason="TRACK_TTL_EXPIRED";
        expired.recommendedPolicy=RecommendedPolicy(track.state);
        expired.appliedPolicy=mConfig.evidenceOnly ? "NONE_EVIDENCE_ONLY" : expired.recommendedPolicy;
        expired.evidenceOnly=mConfig.evidenceOnly; expired.isLost=true;
        output.push_back(expired);
        it=mB4Tracks.erase(it);
    }
    return output;
}
} // namespace ORB_SLAM3
