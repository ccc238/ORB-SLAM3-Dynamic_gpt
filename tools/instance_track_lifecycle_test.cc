#include "Deep/InstanceTrackManager.h"
#include "B4TemporalStateMachine.h"
#include <stdexcept>
#include <iostream>

using namespace ORB_SLAM3;

static void Require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

static YOLOSegInstance Person() {
    YOLOSegInstance person;
    person.classId = 0;
    person.boundingBox = cv::Rect(100, 80, 100, 200);
    person.confidence = .9f;
    person.mask = cv::Mat::zeros(480, 640, CV_8UC1);
    person.mask(person.boundingBox).setTo(255);
    return person;
}

static B3FusionResult Evidence(int id, bool reliable = false, bool moving = false) {
    B3FusionResult b3;
    b3.instance_id = 0;
    b3.track_id = id;
    b3.bounding_box = Person().boundingBox;
    b3.fusion_reliable = reliable;
    b3.fusion_category = !reliable ? B3_FUSION_UNRELIABLE :
        moving ? B3_SEM_DYNAMIC_GEOM_MOVING : B3_SEM_DYNAMIC_GEOM_STATIC;
    return b3;
}

static void TestRecoveryAndExpiry() {
    B4Config config;
    config.trackTTL = 15;
    InstanceTrackManager manager(config);
    B4TemporalStateMachine states(config);
    std::vector<YOLOSegInstance> persons{Person()};
    manager.Associate(persons, cv::Size(640, 480));
    const int id = persons[0].trackId;
    Require(id >= 0, "ID must exist before Frame construction");
    auto row = states.Process(0, 0., {Evidence(id)}, 800.f).at(0);
    Require(row.state == B4MotionState::PENDING && row.nStatic == 0 && row.nDynamic == 0,
            "New track must initialize in P");
    manager.Associate(persons, cv::Size(640, 480));
    row = states.Process(1, 1., {Evidence(id, true)}, 800.f).at(0);
    Require(row.nStatic == 1 && row.missedFrames == 0, "Detected track must not be missed");
    // A detected but geometrically unreliable person does not age the TTL.
    manager.Associate(persons, cv::Size(640, 480));
    row = states.Process(2, 2., {Evidence(id)}, 800.f).at(0);
    Require(row.nStatic == 1 && row.missedFrames == 0, "Unreliable is not missing");
    for (int frame = 3; frame <= 17; ++frame) {
        persons.clear();
        manager.Associate(persons, cv::Size(640, 480));
        row = states.Process(frame, frame, {}, 800.f).at(0);
        Require(row.isLost && row.missedFrames == frame - 2 && row.nStatic == 1 &&
                row.reasonCode != "TRACK_TTL_EXPIRED", "Retain track and counts through 15 missing frames");
    }
    persons = {Person()};
    manager.Associate(persons, cv::Size(640, 480));
    Require(persons[0].trackId == id, "15 missing frames must allow the original ID to recover");
    row = states.Process(18, 18., {Evidence(persons[0].trackId, true)}, 800.f).at(0);
    Require(row.trackId == id && row.nStatic == 2 && row.missedFrames == 0,
            "Recovered state must preserve counters, increment only once");
    for (int frame = 19; frame <= 34; ++frame) {
        persons.clear();
        manager.Associate(persons, cv::Size(640, 480));
        row = states.Process(frame, frame, {}, 800.f).at(0);
        Require((row.reasonCode == "TRACK_TTL_EXPIRED") == (frame == 34),
                "Only the 16th missing frame may expire the track");
    }
    persons = {Person()};
    manager.Associate(persons, cv::Size(640, 480));
    Require(persons[0].trackId != id, "Expired identity must not be recovered");
    row = states.Process(35, 35., {Evidence(persons[0].trackId)}, 800.f).at(0);
    Require(row.state == B4MotionState::PENDING && row.nStatic == 0 && row.nDynamic == 0,
            "A new identity must not inherit the expired state");
    persons = {Person(), Person()};
    manager.Associate(persons, cv::Size(640, 480));
    Require(persons[0].trackId != persons[1].trackId, "One-to-one assignment must not duplicate IDs");
}

static void TestDuplicateUpdates() {
    B4TemporalStateMachine states(B4Config{});
    auto row = states.Process(0, 0., {Evidence(0, true, true)}, 800.f).at(0);
    Require(row.nDynamic == 1, "First evidence must increment once");
    bool rejected = false;
    try { states.Process(0, 0., {Evidence(0, true, true)}, 800.f); }
    catch (const std::runtime_error&) { rejected = true; }
    Require(rejected, "Duplicate frame must be rejected");
    rejected = false;
    try { states.Process(1, 1., {Evidence(0, true, true), Evidence(0, true, true)}, 800.f); }
    catch (const std::runtime_error&) { rejected = true; }
    Require(rejected, "Duplicate track in the same frame must be rejected");
    row = states.Process(1, 1., {Evidence(0, true, true)}, 800.f).at(0);
    Require(row.nDynamic == 2 && row.ageFrames == 2 && row.state == B4MotionState::PENDING,
            "Rejected updates must not change counters, age or frame guard");
    row = states.Process(2, 2., {Evidence(0, true, true)}, 800.f).at(0);
    Require(row.nDynamic == 3 && row.state == B4MotionState::DYNAMIC,
            "Only three distinct reliable observations may enter D");
}

int main() {
    try {
        TestRecoveryAndExpiry();
        TestDuplicateUpdates();
        std::cout << "PASS shared IDs, NEW->P, one-to-one, unreliable freeze, TTL recovery/expiry, duplicate-update guards\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
