#include "B4TemporalStateMachine.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ORB_SLAM3;

namespace {

B4Config LegacyConfig() {
    B4Config config;
    config.evidenceExpiryEnable = false;
    return config;
}

bool Expect(bool condition, const std::string& message)
{
    if (condition)
        return true;
    std::cerr << "[B4 TEST FAILED] " << message << std::endl;
    return false;
}

B3FusionResult Evidence(int trackId, bool moving)
{
    B3FusionResult result;
    result.instance_id = trackId;
    result.track_id = trackId;
    result.bounding_box = cv::Rect(20, 30, 80, 160);
    result.fusion_reliable = true;
    result.fusion_category = moving ? B3_SEM_DYNAMIC_GEOM_MOVING
                                    : B3_SEM_DYNAMIC_GEOM_STATIC;
    result.fusion_reason = moving ? "PERSON_GEOM_MOVING" : "PERSON_GEOM_STATIC";
    return result;
}

B3FusionResult UnreliableEvidence(int trackId)
{
    B3FusionResult result = Evidence(trackId, false);
    result.fusion_reliable = false;
    result.fusion_category = B3_FUSION_UNRELIABLE;
    result.fusion_reason = "INSUFFICIENT_BACKGROUND_REFERENCE";
    return result;
}

const B4StateResult& Step(B4TemporalStateMachine& machine, long frameId,
                          const B3FusionResult& evidence)
{
    const std::vector<B4StateResult> results = machine.Process(
        frameId, static_cast<double>(frameId), {evidence}, 640.0f);
    if (results.size() != 1)
    {
        std::cerr << "[B4 TEST FAILED] expected one current result, got "
                  << results.size() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    // The test only reads this result before the next Process() call.
    static B4StateResult snapshot;
    snapshot = results.front();
    return snapshot;
}

bool TestPendingToStatic()
{
    B4TemporalStateMachine machine(LegacyConfig());
    for (int frame = 1; frame <= 6; ++frame)
    {
        const B4StateResult& r = Step(machine, frame, Evidence(0, false));
        const B4MotionState expected = frame == 6 ? B4MotionState::STATIC
                                                   : B4MotionState::PENDING;
        if (!Expect(r.state == expected, "P->S state at static evidence " + std::to_string(frame)))
            return false;
    }
    return true;
}

bool TestPendingToDynamic()
{
    B4TemporalStateMachine machine(LegacyConfig());
    for (int frame = 1; frame <= 3; ++frame)
    {
        const B4StateResult& r = Step(machine, frame, Evidence(1, true));
        const B4MotionState expected = frame == 3 ? B4MotionState::DYNAMIC
                                                   : B4MotionState::PENDING;
        if (!Expect(r.state == expected, "P->D state at moving evidence " + std::to_string(frame)))
            return false;
    }
    return true;
}

bool TestStaticToPending()
{
    B4TemporalStateMachine machine(LegacyConfig());
    for (int frame = 1; frame <= 6; ++frame)
        Step(machine, frame, Evidence(2, false));
    const B4StateResult& r = Step(machine, 7, Evidence(2, true));
    return Expect(r.previousState == B4MotionState::STATIC &&
                  r.state == B4MotionState::PENDING && r.nDynamic == 1 &&
                  r.nStatic == 0 && r.reasonCode.find("S_TO_P") != std::string::npos,
                  "S + reliable moving must become P with Nd=1");
}

bool TestDynamicToPending()
{
    B4TemporalStateMachine machine(LegacyConfig());
    for (int frame = 1; frame <= 3; ++frame)
        Step(machine, frame, Evidence(3, true));
    for (int frame = 4; frame <= 12; ++frame)
    {
        const B4StateResult& r = Step(machine, frame, Evidence(3, false));
        if (!Expect(r.state == B4MotionState::DYNAMIC && r.nRecovery == frame - 3,
                    "D must remain D for the first nine static recovery frames"))
            return false;
    }
    const B4StateResult& r = Step(machine, 13, Evidence(3, false));
    return Expect(r.previousState == B4MotionState::DYNAMIC &&
                  r.state == B4MotionState::PENDING && r.nRecovery == 0 &&
                  r.nStatic == 0 && r.nDynamic == 0 &&
                  r.reasonCode.find("D_TO_P") != std::string::npos,
                  "tenth reliable static observation must perform D->P, never D->S");
}

bool TestUnreliableFreeze()
{
    B4TemporalStateMachine machine(LegacyConfig());
    for (int frame = 1; frame <= 3; ++frame)
        Step(machine, frame, Evidence(4, false));
    const B4StateResult& r = Step(machine, 4, UnreliableEvidence(4));
    return Expect(r.state == B4MotionState::PENDING && r.nStatic == 3 &&
                  r.nDynamic == 0 && r.nRecovery == 0 && r.nUnreliable == 1 &&
                  r.reasonCode.find("UNRELIABLE_EVIDENCE_FREEZE") != std::string::npos,
                  "unreliable evidence must freeze P and all reliable counters");
}

bool TestTrackTtl()
{
    B4Config config = LegacyConfig();
    config.trackTTL = 15;
    B4TemporalStateMachine machine(config);
    Step(machine, 0, Evidence(5, false));
    for (int frame = 1; frame <= 15; ++frame)
    {
        const std::vector<B4StateResult> results = machine.Process(frame, frame, {}, 640.0f);
        if (!Expect(results.size() == 1 && results.front().isLost &&
                    results.front().missedFrames == frame &&
                    results.front().reasonCode != "TRACK_TTL_EXPIRED",
                    "track must be retained through missed frame " + std::to_string(frame)))
            return false;
    }
    const std::vector<B4StateResult> expired = machine.Process(16, 16.0, {}, 640.0f);
    if (!Expect(expired.size() == 1 && expired.front().isLost &&
                expired.front().reasonCode == "TRACK_TTL_EXPIRED" &&
                expired.front().missedFrames == 16,
                "track must terminate at the 16th missed frame for TTL=15"))
        return false;
    const std::vector<B4StateResult> removed = machine.Process(17, 17.0, {}, 640.0f);
    return Expect(removed.empty(), "expired track must no longer be active");
}

} // namespace

int main()
{
    const bool passed = TestPendingToStatic() && TestPendingToDynamic() &&
                        TestStaticToPending() && TestDynamicToPending() &&
                        TestUnreliableFreeze() && TestTrackTtl();
    if (!passed)
        return EXIT_FAILURE;
    std::cout << "[B4 TEST PASSED] P->S, P->D, S->P, D->P, freeze, TTL" << std::endl;
    return EXIT_SUCCESS;
}
