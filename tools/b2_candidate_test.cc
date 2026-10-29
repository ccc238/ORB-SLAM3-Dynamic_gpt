#include "B2MotionEvidence.h"
#include "Deep/InstanceTrackManager.h"
#include "ORBextractor.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main()
{
    try {
        ORB_SLAM3::B2Config config;
        config.enable = true;
        config.maxFeaturesPerInstance = 60;
        config.candidateErodeRadius = 8;
        config.minBoundaryDistance = 5;
        config.minDepthMeters = 0.30f;
        config.maxDepthMeters = 5.00f;
        config.saveCsv = true;
        config.csvOutputParent = "build_yolo11/validation";
        config.geometryDecisionEnable = true;
        config.minMatchedFeatures = 1;
        config.minMatchRatio = 0.5f;
        config.minValidFeatures = 1;
        config.minValidMatchRatio = 0.5f;
        ORB_SLAM3::B2MotionEvidence evidence(config);
        const cv::Mat calibration = (cv::Mat_<float>(3, 3) <<
            100.0f, 0.0f, 160.0f, 0.0f, 100.0f, 120.0f, 0.0f, 0.0f, 1.0f);
        evidence.SetCameraCalibration(calibration);
        ORB_SLAM3::ORBextractor frameExtractor(1000, 1.2f, 8, 20, 7);

        cv::Mat image(240, 320, CV_8UC1);
        cv::randu(image, 0, 255);
        cv::Mat depth(image.size(), CV_32F, cv::Scalar(2.0f));
        cv::Mat rawMask = cv::Mat::zeros(image.size(), CV_8UC1);
        const cv::Rect box(70, 45, 180, 150);
        cv::rectangle(rawMask, box, cv::Scalar(255), cv::FILLED);

        ORB_SLAM3::YOLOSegInstance source;
        source.mask = rawMask;
        source.boundingBox = box;
        source.confidence = 0.9f;
        source.classId = 0;
        ORB_SLAM3::B4Config trackConfig;
        ORB_SLAM3::InstanceTrackManager tracks(trackConfig);
        std::vector<ORB_SLAM3::YOLOSegInstance> semantics{source};
        tracks.Associate(semantics, image.size());
        source = semantics.front();
        const auto frame = evidence.ExtractCandidates(
            image, depth, std::vector<ORB_SLAM3::YOLOSegInstance>{source},
            12.5, frameExtractor);
        if (frame.persons.size() != 1 ||
            cv::norm(frame.persons[0].mask, rawMask, cv::NORM_INF) != 0)
            throw std::runtime_error("raw person mask was not preserved");
        if (frame.persons[0].features.empty() ||
            frame.persons[0].features.size() > 60)
            throw std::runtime_error("candidate feature limit was not respected");
        for (const auto& feature : frame.persons[0].features) {
            const int x = cvRound(feature.keypoint.pt.x);
            const int y = cvRound(feature.keypoint.pt.y);
            if (rawMask.at<uchar>(y, x) == 0 ||
                feature.boundaryDistance < config.minBoundaryDistance ||
                !feature.depthValid || std::fabs(feature.depthMeters - 2.0f) > 1e-5f ||
                feature.depthStatus != ORB_SLAM3::DEPTH_VALID ||
                feature.descriptor.rows != 1 || feature.descriptor.cols != 32)
                throw std::runtime_error("invalid B2 candidate feature");
        }
        cv::Mat invalidDepth(image.size(), CV_32F, cv::Scalar(0.10f));
        const auto invalidFrame = evidence.ExtractCandidates(
            image, invalidDepth, std::vector<ORB_SLAM3::YOLOSegInstance>{source},
            12.6, frameExtractor);
        if (invalidFrame.persons.empty() || invalidFrame.persons[0].features.empty())
            throw std::runtime_error("invalid-depth candidate setup failed");
        for (const auto& feature : invalidFrame.persons[0].features)
            if (feature.depthValid || feature.depthStatus != ORB_SLAM3::DEPTH_INVALID ||
                feature.depthMeters >= 0.0f)
                throw std::runtime_error("out-of-range depth was not marked DEPTH_INVALID");
        evidence.Process(frame);
        if (!evidence.HasPreviousFrame() || evidence.PreviousFrame().timestamp != 12.5)
            throw std::runtime_error("B2 previous-frame ownership failed");
        if (frame.persons[0].trackId != source.trackId ||
            frame.persons[0].features[0].depthStatus != ORB_SLAM3::DEPTH_VALID)
            throw std::runtime_error("B2 Process modified caller-owned candidate input");
        std::ifstream candidateCsv(evidence.CsvOutputDirectory() + "/b2_candidate_features.csv");
        std::ifstream instanceCsv(evidence.CsvOutputDirectory() + "/b2_instance_evidence.csv");
        std::ifstream metadata(evidence.CsvOutputDirectory() + "/b2_metadata.txt");
        std::string header;
        if (!candidateCsv || !instanceCsv || !metadata ||
            !std::getline(candidateCsv, header) ||
            header.find("r3d_normalized") == std::string::npos ||
            !std::getline(instanceCsv, header) ||
            header.find("valid_match_ratio") == std::string::npos ||
            header.find("processing_ms") == std::string::npos ||
            !std::getline(metadata, header) || header.find("git_commit=") != 0)
            throw std::runtime_error("B2 CSV/metadata output was not created correctly");
        const int firstTrackId = evidence.PreviousFrame().persons[0].trackId;
        ORB_SLAM3::B2CandidateFrame associated;
        associated.timestamp = 12.7;
        ORB_SLAM3::B2PersonInstance nearby;
        nearby.instanceId = 0;
        nearby.classId = 0;
        nearby.boundingBox = cv::Rect(74, 48, 180, 150);
        semantics.front().boundingBox = nearby.boundingBox;
        tracks.Associate(semantics, image.size());
        nearby.trackId = semantics.front().trackId;
        nearby.association.allowGeometryPair = true; // Isolated legacy join fixture, not managed capture times.
        associated.persons.push_back(nearby);
        evidence.Process(associated);
        const auto& linked = evidence.PreviousFrame().persons[0];
        if (linked.trackId != firstTrackId || linked.previousInstanceId != 0 ||
            !evidence.LastEndedTrackIds().empty())
            throw std::runtime_error("nearby same-class person was not associated");
        ORB_SLAM3::B2CandidateFrame unmatched;
        unmatched.timestamp = 12.8;
        ORB_SLAM3::B2PersonInstance distant;
        distant.instanceId = 0;
        distant.classId = 0;
        distant.boundingBox = cv::Rect(0, 0, 30, 30);
        semantics.front().boundingBox = distant.boundingBox;
        semantics.front().mask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::rectangle(semantics.front().mask, distant.boundingBox, cv::Scalar(255), cv::FILLED);
        tracks.Associate(semantics, image.size());
        distant.trackId = semantics.front().trackId;
        unmatched.persons.push_back(distant);
        evidence.Process(unmatched);
        if (evidence.PreviousFrame().persons[0].trackId == firstTrackId ||
            evidence.PreviousFrame().persons[0].previousInstanceId >= 0 ||
            !evidence.LastEndedTrackIds().empty())
            throw std::runtime_error("shared instance ID was not restarted/preserved independently of B2");

        ORB_SLAM3::B2MotionEvidence matchingEvidence(config);
        matchingEvidence.SetCameraCalibration(calibration);
        auto makeFeature = [](int id, uchar value, bool depthValid) {
            ORB_SLAM3::B2CandidateFeature feature;
            feature.candidateId = id;
            feature.descriptor = cv::Mat(1, 32, CV_8U, cv::Scalar(value)).clone();
            feature.depthValid = depthValid;
            feature.depthStatus = depthValid ? ORB_SLAM3::DEPTH_VALID : ORB_SLAM3::DEPTH_INVALID;
            feature.depthMeters = depthValid ? 2.0f : -1.0f;
            feature.keypoint = cv::KeyPoint(100.0f + static_cast<float>(id % 2) * 20.0f,
                                            100.0f, 8.0f);
            return feature;
        };
        ORB_SLAM3::B2CandidateFrame matchingPrevious;
        matchingPrevious.imageSize = image.size();
        matchingPrevious.poseValid = true;
        matchingPrevious.Tcw = Sophus::SE3f();
        for (unsigned long id = 1; id <= 50; ++id) {
            ORB_SLAM3::B2BackgroundObservation observation;
            observation.mapPointId = id;
            observation.pixel = cv::Point2f(float((id % 10) * 20 + 5),
                                            float((id / 10) * 20 + 5));
            observation.depthMeters = 2.0f;
            matchingPrevious.backgroundObservations.push_back(observation);
        }
        ORB_SLAM3::B2PersonInstance matchingPreviousPerson;
        matchingPreviousPerson.instanceId = 0;
        matchingPreviousPerson.trackId = 0;
        matchingPreviousPerson.classId = 0;
        matchingPreviousPerson.boundingBox = box;
        matchingPreviousPerson.features = {makeFeature(0, 0, true), makeFeature(1, 255, true)};
        matchingPrevious.persons.push_back(matchingPreviousPerson);
        ORB_SLAM3::B2PersonInstance sparsePreviousPerson;
        sparsePreviousPerson.instanceId = 1;
        sparsePreviousPerson.trackId = 1;
        sparsePreviousPerson.classId = 0;
        sparsePreviousPerson.boundingBox = cv::Rect(10, 10, 35, 35);
        sparsePreviousPerson.features = {makeFeature(2, 17, true)};
        matchingPrevious.persons.push_back(sparsePreviousPerson);
        matchingEvidence.Process(matchingPrevious);
        ORB_SLAM3::B2CandidateFrame matchingCurrent;
        matchingCurrent.imageSize = image.size();
        matchingCurrent.poseValid = true;
        matchingCurrent.Tcw = Sophus::SE3f();
        matchingCurrent.backgroundObservations = matchingPrevious.backgroundObservations;
        ORB_SLAM3::B2PersonInstance matchingCurrentPerson;
        matchingCurrentPerson.instanceId = 0;
        matchingCurrentPerson.trackId = 0;
        matchingCurrentPerson.association.allowGeometryPair = true;
        matchingCurrentPerson.classId = 0;
        matchingCurrentPerson.boundingBox = cv::Rect(72, 46, 180, 150);
        matchingCurrentPerson.features = {makeFeature(10, 0, true), makeFeature(11, 255, false)};
        if (cv::norm(matchingPreviousPerson.features[0].descriptor,
                     matchingCurrentPerson.features[1].descriptor, cv::NORM_HAMMING) != 256 ||
            cv::norm(matchingPreviousPerson.features[0].descriptor,
                     matchingCurrentPerson.features[0].descriptor, cv::NORM_HAMMING) != 0)
            throw std::runtime_error("B2 test descriptor setup failed");
        matchingCurrent.persons.push_back(matchingCurrentPerson);
        ORB_SLAM3::B2PersonInstance sparseCurrentPerson;
        sparseCurrentPerson.instanceId = 1;
        sparseCurrentPerson.trackId = 1;
        sparseCurrentPerson.association.allowGeometryPair = true;
        sparseCurrentPerson.classId = 0;
        sparseCurrentPerson.boundingBox = cv::Rect(11, 11, 35, 35);
        sparseCurrentPerson.features = {makeFeature(12, 17, true)};
        matchingCurrent.persons.push_back(sparseCurrentPerson);
        matchingEvidence.Process(matchingCurrent);
        const auto& processedPersons = matchingEvidence.PreviousFrame().persons;
        const auto matchedIt = std::find_if(processedPersons.begin(), processedPersons.end(),
            [](const ORB_SLAM3::B2PersonInstance& person) { return person.instanceId == 0; });
        const auto sparseIt = std::find_if(processedPersons.begin(), processedPersons.end(),
            [](const ORB_SLAM3::B2PersonInstance& person) { return person.instanceId == 1; });
        if (matchedIt == processedPersons.end() || sparseIt == processedPersons.end())
            throw std::runtime_error("B2 per-instance results were not retained");
        const auto& matched = *matchedIt;
        if (matched.candidateCount != 2 || matched.forwardMatchCount != 2 ||
            matched.mutualMatchCount != 2 || matched.depthValidMatchCount != 1 ||
            matched.matchedCount != 2 ||
            matched.matches.size() != 2 || matched.matches[0].status != ORB_SLAM3::MATCH_VALID ||
            matched.matches[1].status != ORB_SLAM3::MATCH_INVALID) {
            std::cerr << "match diagnostics candidate=" << matched.candidateCount
                      << " forward=" << matched.forwardMatchCount
                      << " mutual=" << matched.mutualMatchCount
                      << " depth=" << matched.depthValidMatchCount
                      << " records=" << matched.matches.size() << '\n';
            for (const auto& match : matched.matches)
                std::cerr << "  prev=" << match.previousCandidateId
                          << " cur=" << match.currentCandidateId
                          << " dist=" << match.hammingDistance
                          << " status=" << int(match.status) << '\n';
            throw std::runtime_error("B2 mutual descriptor/depth matching failed");
        }
        if (matched.matches[0].geometryStatus != ORB_SLAM3::GEOMETRY_VALID ||
            std::fabs(matched.matches[0].rawR3D) > 1e-5f ||
            std::fabs(matched.matches[0].normalizedR3D) > 1e-5f ||
            std::fabs(matched.matches[0].r2D) > 1e-5f ||
            matched.matches[1].geometryStatus != ORB_SLAM3::GEOMETRY_NOT_EVALUATED ||
            matched.validGeometryCount != 1 || std::fabs(matched.validGeometryRatio - 0.5f) > 1e-5f ||
            matched.geometryDecision != ORB_SLAM3::GEOM_STATIC_CANDIDATE)
            throw std::runtime_error("B2 relative-pose geometry residuals failed");
        if (sparseIt->candidateCount != 1 || sparseIt->matchedCount != 0 ||
            sparseIt->geometryDecision != ORB_SLAM3::GEOM_UNRELIABLE ||
            sparseIt->geometryFailureReason != ORB_SLAM3::INSUFFICIENT_MATCHES)
            throw std::runtime_error("B2 person instances were merged or evidence gates were not independent");
        // Batch 6: a failed coarse pose must not be replaced by an available
        // final pose or drive evidence. The next frame must also respect the
        // invalid previous-stage snapshot.
        auto weakPoseCurrent = matchingCurrent;
        weakPoseCurrent.poseValid = false;
        const auto weakResult = matchingEvidence.Process(weakPoseCurrent);
        for (const auto& person : weakResult.persons)
            if (person.geometryDecision != ORB_SLAM3::GEOM_UNRELIABLE ||
                person.geometryFailureReason != ORB_SLAM3::POSE_WEAK)
                throw std::runtime_error("Invalid coarse pose produced reliable geometry");
        if (matchingEvidence.PreviousFrame().poseValid || weakPoseCurrent.poseValid)
            throw std::runtime_error("Invalid coarse-pose snapshot was overwritten");
        const auto invalidPreviousResult = matchingEvidence.Process(matchingCurrent);
        for (const auto& person : invalidPreviousResult.persons)
            if (person.geometryDecision != ORB_SLAM3::GEOM_UNRELIABLE ||
                person.geometryFailureReason != ORB_SLAM3::POSE_WEAK)
                throw std::runtime_error("Invalid previous coarse pose was ignored");
        // Explicit motion must be authoritative even when both absolute Tcw
        // snapshots are identity. Person AND background raw residuals must
        // reflect this supplied translation, not a hidden reconstructed pose.
        ORB_SLAM3::B2MotionEvidence explicitEvidence(config);
        explicitEvidence.SetCameraCalibration(calibration);
        auto explicitPrevious=matchingPrevious;
        auto explicitCurrent=matchingCurrent;
        explicitPrevious.poseSource=explicitCurrent.poseSource="coarse";
        Sophus::SE3f suppliedMotion;
        suppliedMotion.translation().x()=0.10f;
        const auto explicitResult=explicitEvidence.Process(
            &explicitPrevious,explicitCurrent,suppliedMotion,true);
        const auto& explicitPerson=explicitResult.persons.front();
        if (std::fabs(explicitPerson.rawR3DMedian-.10f)>1e-5f ||
            std::fabs(explicitPerson.backgroundMedianR3D-.10f)>1e-5f ||
            std::fabs(explicitPerson.normalizedR3DMedian)>1e-5f ||
            !explicitPrevious.Tcw.matrix().isIdentity() || !explicitCurrent.poseValid)
            throw std::runtime_error("Explicit motion was ignored or caller snapshots were mutated");
        const auto explicitWeak=explicitEvidence.Process(
            &explicitPrevious,explicitCurrent,suppliedMotion,false);
        for (const auto& person:explicitWeak.persons)
            if (person.geometryDecision!=ORB_SLAM3::GEOM_UNRELIABLE ||
                person.geometryFailureReason!=ORB_SLAM3::POSE_WEAK)
                throw std::runtime_error("Explicit weak pose produced reliable evidence");
        auto wrongSource=explicitCurrent;
        wrongSource.poseSource="final";
        bool rejectedMixedSource=false;
        try {explicitEvidence.Process(&explicitPrevious,wrongSource,suppliedMotion,true);}
        catch (const std::runtime_error&) {rejectedMixedSource=true;}
        if (!rejectedMixedSource) throw std::runtime_error("Mixed coarse/final history was accepted");
        const auto bootstrap=explicitEvidence.Process(nullptr,explicitCurrent,Sophus::SE3f(),true);
        if (!explicitEvidence.PreviousFrame().poseValid ||
            bootstrap.persons.front().geometryFailureReason!=ORB_SLAM3::POSE_WEAK)
            throw std::runtime_error("Explicit bootstrap failed to retain valid current pose for next pair");
        std::cout << "PASS explicit_motion=authoritative coarse_final_mix=rejected candidates="
                  << frame.persons[0].features.size()
                  << " invalid_coarse_and_previous_pose_gates=passed" << '\n';
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 2;
    }
    return 0;
}
