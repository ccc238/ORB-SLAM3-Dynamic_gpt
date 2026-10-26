#include "B2MotionEvidence.h"
#include "ORBextractor.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_map>
#include <utility>

namespace ORB_SLAM3 {
namespace {
// ORB uses a 31x31 descriptor patch.  Components smaller than one patch are
// not useful as a safe person-interior feature region and are discarded.
const int kMinimumCandidateComponentAreaPixels = 31 * 31;

float BoundingBoxIoU(const cv::Rect& a, const cv::Rect& b)
{
    const cv::Rect intersection = a & b;
    if (intersection.empty()) return 0.0f;
    const float unionArea = static_cast<float>(a.area() + b.area() - intersection.area());
    return unionArea > 0.0f ? static_cast<float>(intersection.area()) / unionArea : 0.0f;
}

float CenterDistance(const cv::Rect& a, const cv::Rect& b)
{
    const cv::Point2f centerA(a.x + 0.5f * a.width, a.y + 0.5f * a.height);
    const cv::Point2f centerB(b.x + 0.5f * b.width, b.y + 0.5f * b.height);
    return cv::norm(centerA - centerB);
}

struct NearestDescriptor {
    int index = -1;
    int distance = -1;
    bool accepted = false;
};

bool IsOrbDescriptor(const cv::Mat& descriptor)
{
    return !descriptor.empty() && descriptor.type() == CV_8U &&
           descriptor.rows == 1 && descriptor.cols == 32;
}

NearestDescriptor FindNearest(const B2CandidateFeature& query,
                              const std::vector<B2CandidateFeature>& candidates,
                              float loweRatio, int maxHammingDistance)
{
    NearestDescriptor result;
    if (!IsOrbDescriptor(query.descriptor)) return result;
    int secondDistance = std::numeric_limits<int>::max();
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        if (!IsOrbDescriptor(candidates[i].descriptor)) continue;
        const int distance = static_cast<int>(cv::norm(
            query.descriptor, candidates[i].descriptor, cv::NORM_HAMMING));
        if (result.index < 0)
        {
            result.distance = distance;
            result.index = static_cast<int>(i);
        }
        else if (distance < result.distance)
        {
            secondDistance = result.distance;
            result.distance = distance;
            result.index = static_cast<int>(i);
        }
        else if (distance < secondDistance)
            secondDistance = distance;
    }
    // A ratio test requires a distinct second neighbour.  A one-feature
    // instance is deliberately marked invalid rather than guessed.
    if (result.index >= 0 && secondDistance != std::numeric_limits<int>::max() &&
        result.distance <= maxHammingDistance &&
        static_cast<float>(result.distance) < loweRatio * static_cast<float>(secondDistance))
        result.accepted = true;
    return result;
}

float Median(std::vector<float> values)
{
    if (values.empty()) return -1.0f;
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const float upper = values[middle];
    if (values.size() % 2 != 0) return upper;
    std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
    return 0.5f * (upper + values[middle - 1]);
}

struct BackgroundStatistics {
    float median = -1.0f;
    float sigma = -1.0f;
    int count = 0;
    bool valid = false;
};

BackgroundStatistics MakeBackgroundStatistics(const std::vector<float>& values,
                                              float sigmaFloor)
{
    BackgroundStatistics result;
    result.count = static_cast<int>(values.size());
    if (values.empty()) return result;
    result.median = Median(values);
    std::vector<float> deviations;
    deviations.reserve(values.size());
    for (const float value : values)
        deviations.push_back(std::fabs(value - result.median));
    result.sigma = std::max(1.4826f * Median(deviations), sigmaFloor);
    result.valid = std::isfinite(result.median) && std::isfinite(result.sigma);
    return result;
}

// Background residuals must not be learned from a person contour or the
// immediate uncertainty band around it.  This performs an on-mask dilation
// query without ever using B1's union/dilated reject mask.
bool IsOutsideAllPersonMasks(const B2CandidateFrame& frame, const cv::Point2f& pixel,
                             int exclusionRadius)
{
    const int x = cvRound(pixel.x), y = cvRound(pixel.y);
    if (x < 0 || y < 0 || x >= frame.imageSize.width || y >= frame.imageSize.height)
        return false;
    for (const B2PersonInstance& person : frame.persons)
    {
        if (person.mask.empty() || person.mask.type() != CV_8UC1 ||
            person.mask.size() != frame.imageSize)
            continue;
        for (int dy = -exclusionRadius; dy <= exclusionRadius; ++dy)
        {
            for (int dx = -exclusionRadius; dx <= exclusionRadius; ++dx)
            {
                if (dx * dx + dy * dy > exclusionRadius * exclusionRadius)
                    continue;
                const int mx = x + dx, my = y + dy;
                if (mx >= 0 && my >= 0 && mx < person.mask.cols && my < person.mask.rows &&
                    person.mask.at<uchar>(my, mx) != 0)
                    return false;
            }
        }
    }
    return true;
}

const char* EvidenceName(B2InstanceGeometryStatus evidence)
{
    switch (evidence) {
    case GEOM_UNRELIABLE: return "GEOM_UNRELIABLE";
    case GEOM_STATIC_CANDIDATE: return "GEOM_STATIC_CANDIDATE";
    case GEOM_MOVING_CANDIDATE: return "GEOM_MOVING_CANDIDATE";
    case GEOM_UNCERTAIN: return "GEOM_UNCERTAIN";
    default: return "GEOM_NOT_EVALUATED";
    }
}

const char* FailureReasonName(B2GeometryFailureReason reason)
{
    switch (reason) {
    case GEOM_FAILURE_NONE: return "GEOMETRY_DECISION_VALID";
    case INSUFFICIENT_CANDIDATES: return "INSUFFICIENT_CANDIDATES";
    case INSUFFICIENT_MATCHES: return "INSUFFICIENT_MATCHES";
    case LOW_MATCH_RATIO: return "LOW_MATCH_RATIO";
    case INSUFFICIENT_VALID_DEPTH: return "INSUFFICIENT_VALID_DEPTH";
    case LOW_VALID_MATCH_RATIO: return "LOW_VALID_MATCH_RATIO";
    case INSUFFICIENT_BACKGROUND_REFERENCE: return "INSUFFICIENT_BACKGROUND_REFERENCE";
    case AMBIGUOUS_GEOMETRIC_EVIDENCE: return "AMBIGUOUS_GEOMETRIC_EVIDENCE";
    case POSE_WEAK: return "POSE_WEAK";
    default: return "GEOMETRY_UNRELIABLE";
    }
}

const char* MatchReason(const B2CandidateMatch& match)
{
    if (!match.descriptorMatched) return "MATCH_INVALID";
    if (match.status != MATCH_VALID) return "DEPTH_INVALID";
    if (match.geometryStatus != GEOMETRY_VALID) return "GEOMETRY_INVALID";
    if (match.normalizedR3D < 0.0f) return "INSUFFICIENT_BACKGROUND_REFERENCE";
    return "OK";
}
}

B2MotionEvidence::B2MotionEvidence(const B2Config& config)
    : mConfig(config)
{
    if (mConfig.saveCsv && (mConfig.saveFeatureCsv || mConfig.saveSummaryCsv))
        InitializeCsvOutput();
}

void B2MotionEvidence::InitializeCsvOutput()
{
    struct stat parentStatus;
    if (mConfig.csvOutputParent.empty() ||
        ::stat(mConfig.csvOutputParent.c_str(), &parentStatus) != 0 ||
        !S_ISDIR(parentStatus.st_mode))
        throw std::runtime_error("B2.Debug.OutputDir must be an existing directory: " +
                                 mConfig.csvOutputParent);
    const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
    mCsvOutputDirectory = mConfig.csvOutputParent + "/b2_evidence_" +
        std::to_string(stamp) + "_" + std::to_string(getpid());
    if (::mkdir(mCsvOutputDirectory.c_str(), 0755) != 0)
        throw std::runtime_error("Cannot create B2 CSV output directory " + mCsvOutputDirectory +
                                 ": " + std::strerror(errno));
    if (mConfig.saveFeatureCsv)
        mCandidateCsv.open(mCsvOutputDirectory + "/b2_candidate_features.csv");
    if (mConfig.saveSummaryCsv)
        mInstanceCsv.open(mCsvOutputDirectory + "/b2_instance_evidence.csv");
    std::ofstream metadata(mCsvOutputDirectory + "/b2_metadata.txt");
    if ((mConfig.saveFeatureCsv && !mCandidateCsv) ||
        (mConfig.saveSummaryCsv && !mInstanceCsv) || !metadata)
        throw std::runtime_error("Cannot open B2 CSV or metadata output in " + mCsvOutputDirectory);
    if (mConfig.saveFeatureCsv)
        mCandidateCsv << "frame_id,timestamp,instance_id,candidate_id,"
                      << "u_prev,v_prev,u_curr,v_curr,descriptor_distance,depth_prev,depth_curr,"
                      << "boundary_distance,r3d,r3d_normalized,r2d,valid,reason_code\n";
    if (mConfig.saveSummaryCsv)
        mInstanceCsv << "frame_id,timestamp,instance_id,track_id,candidate_count,matched_count,valid_count,"
                     << "match_ratio,valid_match_ratio,background_sample_count,background_median_r3d,"
                     << "background_sigma_r3d,median_r3d,mean_r3d,mad_r3d,median_normalized_r3d,"
                     << "high_residual_ratio,median_r2d,pose_valid,geometry_state,decision_reason,processing_ms\n";
#ifdef ORB_SLAM3_GIT_COMMIT
    metadata << "git_commit=" << ORB_SLAM3_GIT_COMMIT << '\n';
#else
    metadata << "git_commit=unknown\n";
#endif
    metadata << "yaml_path=" << mConfig.yamlPath << '\n'
             << "yaml_sha256=" << mConfig.yamlSha256 << '\n'
             << "yolo_model_path=" << mConfig.yoloModelPath << '\n'
             << "yolo_model_sha256=" << mConfig.yoloModelSha256 << '\n'
             << "dataset_name=" << mConfig.datasetName << '\n'
             << "B2.Enable=" << mConfig.enable << '\n'
             << "B2.EvidenceOnly=" << mConfig.evidenceOnly << '\n'
             << "B2.Candidate.Enable=" << mConfig.candidateEnable << '\n'
             << "B2.Candidate.MaxFeaturesPerInstance=" << mConfig.maxFeaturesPerInstance << '\n'
             << "B2.Candidate.ErodeRadius=" << mConfig.candidateErodeRadius << '\n'
             << "B2.Candidate.MinBoundaryDistance=" << mConfig.minBoundaryDistance << '\n'
             << "B2.Association.Enable=" << mConfig.associationEnable << '\n'
             << "B2.Association.UseMutualMatch=" << mConfig.useMutualMatch << '\n'
             << "B2.Association.LoweRatio=" << mConfig.loweRatio << '\n'
             << "B2.Association.MaxHammingDistance=" << mConfig.maxHammingDistance << '\n'
             << "B2.Association.MinInstanceIoU=" << mConfig.minInstanceIoU << '\n'
             << "B2.Association.MaxCenterDistancePixels=" << mConfig.maxInstanceCenterDistancePixels << '\n'
             << "B2.Association.MinInstanceScore=" << mConfig.minInstanceAssociationScore << '\n'
             << "B2.Geometry.Enable=" << mConfig.geometryEnable << '\n'
             << "B2.Geometry.UseDepthMedian3x3=" << mConfig.useDepthMedian3x3 << '\n'
             << "B2.Geometry.MinDepthMeters=" << mConfig.minDepthMeters << '\n'
             << "B2.Geometry.MaxDepthMeters=" << mConfig.maxDepthMeters << '\n'
             << "B2.Geometry.UseNormalizedResidual=" << mConfig.useNormalizedResidual << '\n'
             << "B2.Geometry.MinMatchedFeatures=" << mConfig.minMatchedFeatures << '\n'
             << "B2.Geometry.MinMatchRatio=" << mConfig.minMatchRatio << '\n'
             << "B2.Geometry.MinValidFeatures=" << mConfig.minValidFeatures << '\n'
             << "B2.Geometry.MinValidMatchRatio=" << mConfig.minValidMatchRatio << '\n'
             << "B2.Geometry.BackgroundExclusionDilateRadius=" << mConfig.backgroundExclusionDilateRadius << '\n'
             << "B2.Geometry.Decision.Enable=" << mConfig.geometryDecisionEnable << '\n'
             << "B2.Geometry.Decision.Method=" << mConfig.geometryDecisionMethod << '\n'
             << "B2.Geometry.Decision.DepthBinMeters=" << mConfig.depthBinMeters << '\n'
             << "B2.Geometry.Decision.MinSamplesPerBin=" << mConfig.minSamplesPerBin << '\n'
             << "B2.Geometry.Decision.MinBackgroundSamples=" << mConfig.minBackgroundSamples << '\n'
             << "B2.Geometry.Decision.SigmaFloorMeters=" << mConfig.sigmaFloorMeters << '\n'
             << "B2.Geometry.RawMovingThreshold=" << mConfig.rawMovingThreshold << '\n'
             << "B2.Geometry.NormMovingThreshold=" << mConfig.normMovingThreshold << '\n'
             << "B2.Geometry.HighRatioMovingThreshold=" << mConfig.highRatioMovingThreshold << '\n'
             << "B2.Geometry.RawStaticThreshold=" << mConfig.rawStaticThreshold << '\n'
             << "B2.Geometry.NormStaticThreshold=" << mConfig.normStaticThreshold << '\n'
             << "B2.Geometry.HighRatioStaticThreshold=" << mConfig.highRatioStaticThreshold << '\n'
             << "high_ratio_point_normalized_cutoff=" << mConfig.highResidualNormThreshold << '\n'
             << "B2.Debug.Enable=" << mConfig.debugEnable << '\n'
             << "B2.Debug.SaveCsv=" << mConfig.saveCsv << '\n'
             << "B2.Debug.SaveFeatureCsv=" << mConfig.saveFeatureCsv << '\n'
             << "B2.Debug.SaveSummaryCsv=" << mConfig.saveSummaryCsv << '\n'
             << "B2.Debug.VerbosePerFeature=" << mConfig.verbosePerFeature << '\n'
             << "B2.Debug.PrintFrameSummary=" << mConfig.printFrameSummary << '\n'
             << "B2.Debug.ShowOverlay=" << mConfig.showOverlay << '\n'
             << "B2.Debug.OutputDir=" << mConfig.csvOutputParent << '\n'
             << "B2.Debug.DatasetName=" << mConfig.datasetName << '\n';
    if (!metadata) throw std::runtime_error("Cannot write B2 metadata");
    std::cout << "[B2 CSV] output_directory=" << mCsvOutputDirectory << std::endl;
}

void B2MotionEvidence::SetCameraCalibration(const cv::Mat& K)
{
    if (K.empty() || K.rows != 3 || K.cols != 3)
        throw std::runtime_error("B2 RGB-D geometry requires a 3x3 camera calibration matrix");
    cv::Mat calibration;
    K.convertTo(calibration, CV_32F);
    mFx = calibration.at<float>(0, 0);
    mFy = calibration.at<float>(1, 1);
    mCx = calibration.at<float>(0, 2);
    mCy = calibration.at<float>(1, 2);
    if (!std::isfinite(mFx) || !std::isfinite(mFy) ||
        !std::isfinite(mCx) || !std::isfinite(mCy) ||
        mFx <= 0.0f || mFy <= 0.0f)
        throw std::runtime_error("B2 RGB-D geometry received invalid camera intrinsics");
    mbCameraCalibrated = true;
}

void B2MotionEvidence::EnsureCandidateExtractor(const ORBextractor& mainExtractor)
{
    if (mpCandidateExtractor)
        return;
    // Do not share the main Frame extractor: its pyramid/diagnostic state is
    // mutable and belongs solely to Frame construction.
    mpCandidateExtractor.reset(new ORBextractor(
        mConfig.maxFeaturesPerInstance,
        mainExtractor.GetScaleFactor(), mainExtractor.GetLevels(),
        mainExtractor.GetInitialFastThreshold(),
        mainExtractor.GetMinimumFastThreshold()));
}

float B2MotionEvidence::SampleDepth(const cv::Mat& depthMeters,
                                    const cv::Point2f& point, bool& valid) const
{
    valid = false;
    if (!mConfig.geometryEnable || depthMeters.empty() ||
        depthMeters.type() != CV_32F)
        return -1.0f;
    const int x = cvRound(point.x), y = cvRound(point.y);
    if (x < 0 || y < 0 || x >= depthMeters.cols || y >= depthMeters.rows)
        return -1.0f;

    std::vector<float> values;
    const int radius = mConfig.useDepthMedian3x3 ? 1 : 0;
    for (int yy = std::max(0, y - radius); yy <= std::min(depthMeters.rows - 1, y + radius); ++yy)
        for (int xx = std::max(0, x - radius); xx <= std::min(depthMeters.cols - 1, x + radius); ++xx)
        {
            const float value = depthMeters.at<float>(yy, xx);
            // The depth image supplied by Tracking is already in metres.
            // Only sensor-invalid values are discarded before the median;
            // range validity is judged from that median below.
            if (std::isfinite(value) && value > 0.0f)
                values.push_back(value);
        }
    if (values.empty())
        return -1.0f;
    const size_t medianIndex = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + medianIndex, values.end());
    const float median = values[medianIndex];
    if (median < mConfig.minDepthMeters || median > mConfig.maxDepthMeters)
        return -1.0f;
    valid = true;
    return median;
}

B2CandidateFrame B2MotionEvidence::ExtractCandidates(
    const cv::Mat& grayImage, const cv::Mat& depthMeters,
    const std::vector<YOLOSegInstance>& semanticInstances, double timestamp,
    const ORBextractor& mainExtractor)
{
    if (grayImage.empty() || grayImage.type() != CV_8UC1)
        throw std::runtime_error("B2 candidate extraction requires CV_8UC1 grayscale image");
    if (!depthMeters.empty() && (depthMeters.type() != CV_32F || depthMeters.size() != grayImage.size()))
        throw std::runtime_error("B2 candidate depth must be CV_32F and match RGB size");

    B2CandidateFrame frame;
    frame.timestamp = timestamp;
    frame.imageSize = grayImage.size();
    if (!mConfig.candidateEnable)
        return frame;
    EnsureCandidateExtractor(mainExtractor);

    int candidateId = 0;
    const cv::Rect imageBounds(0, 0, grayImage.cols, grayImage.rows);
    for (size_t i = 0; i < semanticInstances.size(); ++i)
    {
        const YOLOSegInstance& source = semanticInstances[i];
        if (source.mask.type() != CV_8UC1 || source.mask.size() != grayImage.size())
            throw std::runtime_error("B2 raw person instance mask type/size mismatch");

        cv::Mat rawMask;
        cv::threshold(source.mask, rawMask, 0, 255, cv::THRESH_BINARY);

        B2PersonInstance person;
        person.instanceId = static_cast<int>(i);
        person.trackId = source.trackId;
        person.classId = source.classId;
        person.confidence = source.confidence;
        person.boundingBox = source.boundingBox & imageBounds;
        person.mask = rawMask; // preserve the un-eroded semantic output
        if (person.boundingBox.empty())
            continue;

        cv::Mat eroded;
        if (mConfig.candidateErodeRadius > 0)
            cv::erode(rawMask, eroded, cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(2 * mConfig.candidateErodeRadius + 1,
                                             2 * mConfig.candidateErodeRadius + 1)));
        else
            eroded = rawMask;
        if (cv::countNonZero(eroded) == 0) {
            frame.persons.push_back(std::move(person));
            continue;
        }

        // Discard tiny disconnected remnants after erosion.  This protects
        // descriptor extraction from isolated segmentation speckles.
        cv::Mat labels, stats, centroids;
        const int componentCount = cv::connectedComponentsWithStats(
            eroded, labels, stats, centroids, 8, CV_32S);
        cv::Mat safeInterior = cv::Mat::zeros(eroded.size(), CV_8UC1);
        for (int label = 1; label < componentCount; ++label) {
            if (stats.at<int>(label, cv::CC_STAT_AREA) < kMinimumCandidateComponentAreaPixels)
                continue;
            safeInterior.setTo(255, labels == label);
        }
        if (cv::countNonZero(safeInterior) == 0) {
            frame.persons.push_back(std::move(person));
            continue;
        }

        // ORBextractor uses nonzero pixels as exclusions.  Invert the
        // safe person interior so only it can supply B2 features.
        cv::Mat exclusionMask;
        cv::bitwise_not(safeInterior, exclusionMask);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        // RGB-D has no stereo overlap.  ORBextractor expects two bounds even
        // when no overlap exists; this empty interval keeps all keys primary.
        std::vector<int> lapping = {0, -1};
        (*mpCandidateExtractor)(grayImage, exclusionMask, keypoints, descriptors, lapping);
        if (descriptors.rows != static_cast<int>(keypoints.size()))
            throw std::runtime_error("B2 ORB keypoint/descriptor count mismatch");

        // ORBextractor's per-level allocation is rounded independently, so
        // its total can exceed the requested number by a few points.  B2's
        // YAML limit is a strict per-instance cap.
        std::vector<size_t> selected(keypoints.size());
        for (size_t keypointIndex = 0; keypointIndex < selected.size(); ++keypointIndex)
            selected[keypointIndex] = keypointIndex;
        std::sort(selected.begin(), selected.end(), [&](size_t a, size_t b) {
            return keypoints[a].response > keypoints[b].response;
        });
        if (selected.size() > static_cast<size_t>(mConfig.maxFeaturesPerInstance))
            selected.resize(static_cast<size_t>(mConfig.maxFeaturesPerInstance));

        cv::Mat distances;
        cv::distanceTransform(rawMask, distances, cv::DIST_L2, 3);
        for (size_t keypointIndex : selected)
        {
            const cv::KeyPoint& keypoint = keypoints[keypointIndex];
            const int x = cvRound(keypoint.pt.x), y = cvRound(keypoint.pt.y);
            if (x < 0 || y < 0 || x >= distances.cols || y >= distances.rows)
                continue;
            const float boundaryDistance = distances.at<float>(y, x);
            if (boundaryDistance < static_cast<float>(mConfig.minBoundaryDistance))
                continue;

            B2CandidateFeature feature;
            feature.candidateId = candidateId++;
            feature.instanceId = person.instanceId;
            feature.keypoint = keypoint;
            feature.descriptor = descriptors.row(static_cast<int>(keypointIndex)).clone();
            feature.boundaryDistance = boundaryDistance;
            feature.nearMaskBoundary = boundaryDistance < static_cast<float>(mConfig.minBoundaryDistance);
            feature.depthMeters = SampleDepth(depthMeters, keypoint.pt, feature.depthValid);
            feature.depthStatus = feature.depthValid ? DEPTH_VALID : DEPTH_INVALID;
            // A depth-edge threshold is intentionally not invented here; the
            // flag is reserved until the B2 geometry decision design defines
            // one explicitly.
            feature.nearDepthEdge = false;
            person.features.push_back(std::move(feature));
        }
        frame.persons.push_back(std::move(person));
    }
    return frame;
}

B2Result B2MotionEvidence::Process(const B2CandidateFrame& current)
{
    Sophus::SE3f relative;
    if (mbHasPreviousFrame && mPreviousFrame.poseValid && current.poseValid)
        relative = current.Tcw * mPreviousFrame.Tcw.inverse();
    return ProcessInput(mbHasPreviousFrame ? &mPreviousFrame : nullptr,
                        current, relative, current.poseValid, false);
}

B2Result B2MotionEvidence::Process(const B2CandidateFrame* previous,
    const B2CandidateFrame& current, const Sophus::SE3f& currentFromPrevious,
    bool currentPoseReliable)
{
    return ProcessInput(previous,current,currentFromPrevious,currentPoseReliable,true);
}

B2Result B2MotionEvidence::ProcessInput(const B2CandidateFrame* previous,
    const B2CandidateFrame& current, const Sophus::SE3f& currentFromPrevious,
    bool currentPoseReliable, bool explicitInput)
{
    if (previous && !previous->poseSource.empty() && !current.poseSource.empty() &&
        previous->poseSource != current.poseSource)
        throw std::runtime_error("B2 cannot mix coarse and final pose histories");
    if (currentPoseReliable && (!current.poseValid || !currentFromPrevious.matrix().allFinite()))
        throw std::runtime_error("B2 reliable input requires a valid current pose and finite relative pose");
    // Copy before replacing owned history: previous may alias PreviousFrame().
    const B2CandidateFrame previousSnapshot = previous ? *previous : B2CandidateFrame();
    mbHasPreviousFrame = previous != nullptr;
    mPreviousFrame = previousSnapshot;
    mCurrentFromPrevious = currentFromPrevious;
    mbPosePairReliable = mbHasPreviousFrame && currentPoseReliable &&
        current.poseValid && mPreviousFrame.poseValid;
    mbExplicitPoseInput = explicitInput;
    const auto started = std::chrono::steady_clock::now();
    // Never mutate the caller's snapshot. This is deliberately a deep-ish
    // value copy of B2-owned cv::Mat/vector handles; no Frame or MapPoint
    // pointer is present in B2CandidateFrame.
    B2CandidateFrame working = current;
    working.poseValid = current.poseValid && currentPoseReliable;
    // IDs are assigned once immediately after YOLO inference by
    // InstanceTrackManager. B2 only joins its current instances to its own
    // previous measurements; it must never allocate or replace track IDs.
    mLastEndedTrackIds.clear();
    if (mbHasPreviousFrame)
        for (B2PersonInstance& person : working.persons) {
            const auto previous = std::find_if(mPreviousFrame.persons.begin(),
                mPreviousFrame.persons.end(), [&](const B2PersonInstance& item) {
                    return item.trackId == person.trackId && person.trackId >= 0;
                });
            if (previous != mPreviousFrame.persons.end())
                person.previousInstanceId = previous->instanceId;
        }
    if (mbHasPreviousFrame && mConfig.associationEnable)
    {
        MatchAssociatedInstances(working);
        if (mConfig.geometryEnable)
        {
            ComputeGeometryResiduals(working);
            if (mConfig.geometryDecisionEnable)
                ComputeInstanceDecisions(working);
        }
    }
    if (explicitInput && !mbPosePairReliable)
        for (auto& person : working.persons)
        {
            person.geometryDecision = GEOM_UNRELIABLE;
            person.geometryFailureReason = POSE_WEAK;
        }
    B2Result result;
    result.frameId = working.frameId;
    result.processed = true;
    result.persons = working.persons;
    const double processingMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    result.processingMs = processingMs;
    if (mConfig.saveCsv && (mConfig.saveFeatureCsv || mConfig.saveSummaryCsv))
        WriteCsv(working, result, processingMs);
    // Deliberately keep B2-owned state rather than retaining a Frame reference.
    mPreviousFrame = std::move(working);
    mbHasPreviousFrame = true;
    return result;
}

void B2MotionEvidence::WriteCsv(const B2CandidateFrame& current,
                                const B2Result& result, double processingMs)
{
    if ((mConfig.saveFeatureCsv && !mCandidateCsv) ||
        (mConfig.saveSummaryCsv && !mInstanceCsv))
        throw std::runtime_error("B2 CSV stream is not writable");
    for (const B2PersonInstance& person : result.persons)
    {
        const auto previousPerson = std::find_if(mPreviousFrame.persons.begin(),
            mPreviousFrame.persons.end(), [&](const B2PersonInstance& previous) {
                return previous.instanceId == person.previousInstanceId &&
                       previous.trackId == person.trackId;
            });
        if (mConfig.saveFeatureCsv) for (const B2CandidateMatch& match : person.matches)
        {
            cv::Point2f previousPixel(-1.0f, -1.0f), currentPixel(-1.0f, -1.0f);
            float previousDepth = -1.0f, currentDepth = -1.0f, boundaryDistance = -1.0f;
            if (previousPerson != mPreviousFrame.persons.end())
            {
                const auto previousFeature = std::find_if(previousPerson->features.begin(),
                    previousPerson->features.end(), [&](const B2CandidateFeature& feature) {
                        return feature.candidateId == match.previousCandidateId;
                    });
                if (previousFeature != previousPerson->features.end())
                {
                    previousPixel = previousFeature->keypoint.pt;
                    previousDepth = previousFeature->depthMeters;
                }
            }
            const auto currentFeature = std::find_if(person.features.begin(), person.features.end(),
                [&](const B2CandidateFeature& feature) {
                    return feature.candidateId == match.currentCandidateId;
                });
            if (currentFeature != person.features.end())
            {
                currentPixel = currentFeature->keypoint.pt;
                currentDepth = currentFeature->depthMeters;
                boundaryDistance = currentFeature->boundaryDistance;
            }
            const bool valid = match.geometryStatus == GEOMETRY_VALID &&
                match.normalizedR3D >= 0.0f;
            mCandidateCsv << current.frameId << ',' << std::setprecision(17) << current.timestamp << ','
                          << person.instanceId << ',' << match.currentCandidateId << ','
                          << previousPixel.x << ',' << previousPixel.y << ','
                          << currentPixel.x << ',' << currentPixel.y << ','
                          << match.hammingDistance << ',' << previousDepth << ',' << currentDepth << ','
                          << boundaryDistance << ',' << match.rawR3D << ',' << match.normalizedR3D << ','
                          << match.r2D << ',' << int(valid) << ',' << MatchReason(match) << '\n';
        }
        const bool poseValid = mbExplicitPoseInput ? mbPosePairReliable :
            current.poseValid && (!mbHasPreviousFrame || mPreviousFrame.poseValid);
        if (mConfig.saveSummaryCsv)
            mInstanceCsv << current.frameId << ',' << std::setprecision(17) << current.timestamp << ','
                     << person.instanceId << ',' << person.trackId << ','
                     << person.candidateCount << ',' << person.matchedCount << ','
                     << person.validGeometryCount << ',' << person.matchRatio << ','
                     << person.validMatchRatio << ',' << person.backgroundSampleCount << ','
                     << person.backgroundMedianR3D << ',' << person.backgroundSigmaR3D << ','
                     << person.rawR3DMedian << ','
                     << person.rawR3DMean << ',' << person.rawR3DMad << ','
                     << person.normalizedR3DMedian << ',' << person.movingFraction << ','
                     << person.r2DMedian << ',' << int(poseValid) << ','
                     << EvidenceName(person.geometryDecision) << ','
                     << FailureReasonName(person.geometryFailureReason) << ','
                         << processingMs << '\n';
    }
    if (mConfig.saveFeatureCsv)
        mCandidateCsv.flush();
    if (mConfig.saveSummaryCsv)
        mInstanceCsv.flush();
    if ((mConfig.saveFeatureCsv && !mCandidateCsv) ||
        (mConfig.saveSummaryCsv && !mInstanceCsv))
        throw std::runtime_error("Cannot write B2 CSV output");
}

bool B2MotionEvidence::BackProject(const cv::Point2f& pixel, float depth,
                                   Eigen::Vector3f& point) const
{
    if (!mbCameraCalibrated || !std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
        !std::isfinite(depth) || depth <= 0.0f)
        return false;
    point.x() = (pixel.x - mCx) * depth / mFx;
    point.y() = (pixel.y - mCy) * depth / mFy;
    point.z() = depth;
    return point.allFinite();
}

bool B2MotionEvidence::Project(const Eigen::Vector3f& point,
                               const cv::Size& imageSize, cv::Point2f& pixel) const
{
    if (!mbCameraCalibrated || imageSize.width <= 0 || imageSize.height <= 0 ||
        !point.allFinite() || point.z() <= 0.0f)
        return false;
    pixel.x = mFx * point.x() / point.z() + mCx;
    pixel.y = mFy * point.y() / point.z() + mCy;
    return std::isfinite(pixel.x) && std::isfinite(pixel.y) &&
           pixel.x >= 0.0f && pixel.x < static_cast<float>(imageSize.width) &&
           pixel.y >= 0.0f && pixel.y < static_cast<float>(imageSize.height);
}

void B2MotionEvidence::ComputeGeometryResiduals(B2CandidateFrame& current) const
{
    if (!mbCameraCalibrated || !mbPosePairReliable)
        return;

    const Sophus::SE3f& currentFromPrevious = mCurrentFromPrevious;
    for (B2PersonInstance& currentPerson : current.persons)
    {
        if (currentPerson.previousInstanceId < 0)
            continue;
        const auto previousPerson = std::find_if(mPreviousFrame.persons.begin(),
            mPreviousFrame.persons.end(), [&](const B2PersonInstance& person) {
                return person.instanceId == currentPerson.previousInstanceId &&
                       person.trackId == currentPerson.trackId;
            });
        if (previousPerson == mPreviousFrame.persons.end())
            continue;

        for (B2CandidateMatch& match : currentPerson.matches)
        {
            if (match.status != MATCH_VALID)
                continue;
            match.geometryStatus = GEOMETRY_INVALID;
            const auto previousFeature = std::find_if(previousPerson->features.begin(),
                previousPerson->features.end(), [&](const B2CandidateFeature& feature) {
                    return feature.candidateId == match.previousCandidateId;
                });
            const auto currentFeature = std::find_if(currentPerson.features.begin(),
                currentPerson.features.end(), [&](const B2CandidateFeature& feature) {
                    return feature.candidateId == match.currentCandidateId;
                });
            if (previousFeature == previousPerson->features.end() ||
                currentFeature == currentPerson.features.end() ||
                !previousFeature->depthValid || !currentFeature->depthValid)
                continue;
            if (previousFeature->keypoint.pt.x < 0.0f || previousFeature->keypoint.pt.x >= mPreviousFrame.imageSize.width ||
                previousFeature->keypoint.pt.y < 0.0f || previousFeature->keypoint.pt.y >= mPreviousFrame.imageSize.height ||
                currentFeature->keypoint.pt.x < 0.0f || currentFeature->keypoint.pt.x >= current.imageSize.width ||
                currentFeature->keypoint.pt.y < 0.0f || currentFeature->keypoint.pt.y >= current.imageSize.height)
                continue;

            Eigen::Vector3f previousPoint, currentPoint;
            if (!BackProject(previousFeature->keypoint.pt, previousFeature->depthMeters, previousPoint) ||
                !BackProject(currentFeature->keypoint.pt, currentFeature->depthMeters, currentPoint))
                continue;
            const Eigen::Vector3f predictedPoint = currentFromPrevious * previousPoint;
            cv::Point2f predictedPixel;
            if (!predictedPoint.allFinite() || predictedPoint.z() <= 0.0f ||
                !Project(predictedPoint, current.imageSize, predictedPixel))
                continue;

            match.rawR3D = (currentPoint - predictedPoint).norm();
            // The background-MAD normalized value is filled later, after the
            // complete current-frame background reference has been built.
            match.normalizedR3D = -1.0f;
            match.r2D = cv::norm(currentFeature->keypoint.pt - predictedPixel);
            if (!std::isfinite(match.rawR3D) || !std::isfinite(match.r2D))
            {
                match.rawR3D = -1.0f;
                match.r2D = -1.0f;
                continue;
            }
            match.geometryStatus = GEOMETRY_VALID;
            if (mConfig.debugEnable && mConfig.verbosePerFeature)
                std::cout << "[B2 GEOMETRY] track_id=" << currentPerson.trackId
                          << " previous_candidate_id=" << match.previousCandidateId
                          << " current_candidate_id=" << match.currentCandidateId
                          << " raw_r3d=" << match.rawR3D
                          << " r2d=" << match.r2D << std::endl;
        }
    }
}

void B2MotionEvidence::ComputeInstanceDecisions(B2CandidateFrame& current) const
{
    // Build an independent current-frame reference from Tracking inliers that
    // lie outside both frames' raw person masks. MapPoint IDs provide the
    // established background correspondence; they are never changed here.
    std::unordered_map<unsigned long, const B2BackgroundObservation*> previousByMapPoint;
    for (const B2BackgroundObservation& observation : mPreviousFrame.backgroundObservations)
        if (IsOutsideAllPersonMasks(mPreviousFrame, observation.pixel,
                                    mConfig.backgroundExclusionDilateRadius))
            previousByMapPoint[observation.mapPointId] = &observation;

    std::vector<float> allBackgroundResiduals;
    std::unordered_map<int, std::vector<float> > residualsByBin;
    if (mbCameraCalibrated && mbPosePairReliable)
    {
        const Sophus::SE3f& currentFromPrevious = mCurrentFromPrevious;
        for (const B2BackgroundObservation& currentObservation : current.backgroundObservations)
        {
            const auto previousIt = previousByMapPoint.find(currentObservation.mapPointId);
            if (previousIt == previousByMapPoint.end() ||
                !IsOutsideAllPersonMasks(current, currentObservation.pixel,
                                         mConfig.backgroundExclusionDilateRadius))
                continue;
            const B2BackgroundObservation& previousObservation = *previousIt->second;
            Eigen::Vector3f previousPoint, currentPoint;
            if (!BackProject(previousObservation.pixel, previousObservation.depthMeters, previousPoint) ||
                !BackProject(currentObservation.pixel, currentObservation.depthMeters, currentPoint))
                continue;
            const Eigen::Vector3f predicted = currentFromPrevious * previousPoint;
            if (!predicted.allFinite() || predicted.z() <= 0.0f)
                continue;
            const float residual = (currentPoint - predicted).norm();
            if (!std::isfinite(residual))
                continue;
            allBackgroundResiduals.push_back(residual);
            const int bin = static_cast<int>(std::floor(currentObservation.depthMeters / mConfig.depthBinMeters));
            residualsByBin[bin].push_back(residual);
        }
    }

    const BackgroundStatistics fallback = MakeBackgroundStatistics(
        allBackgroundResiduals, mConfig.sigmaFloorMeters);
    const bool backgroundReferenceValid = fallback.count >= mConfig.minBackgroundSamples && fallback.valid;
    std::unordered_map<int, BackgroundStatistics> binStatistics;
    for (const auto& entry : residualsByBin)
    {
        BackgroundStatistics statistics = MakeBackgroundStatistics(entry.second, mConfig.sigmaFloorMeters);
        if (statistics.count >= mConfig.minSamplesPerBin)
            binStatistics[entry.first] = statistics;
    }
    for (B2PersonInstance& person : current.persons)
    {
        person.validGeometryCount = 0;
        person.matchRatio = person.candidateCount > 0 ?
            static_cast<float>(person.matchedCount) / person.candidateCount : 0.0f;
        person.validMatchRatio = 0.0f;
        person.validGeometryRatio = 0.0f;
        person.backgroundSampleCount = fallback.count;
        person.backgroundMedianR3D = fallback.median;
        person.backgroundSigmaR3D = fallback.sigma;
        person.rawR3DMedian = person.rawR3DMean = person.rawR3DMad = -1.0f;
        person.normalizedR3DMedian = person.r2DMedian = person.movingFraction = -1.0f;
        person.geometryDecision = GEOM_UNRELIABLE;
        person.geometryFailureReason = GEOM_FAILURE_NONE;
        std::vector<float> raw, normalized, r2d;
        for (B2CandidateMatch& match : person.matches)
        {
            if (match.geometryStatus != GEOMETRY_VALID)
                continue;
            raw.push_back(match.rawR3D);
            r2d.push_back(match.r2D);
            if (backgroundReferenceValid)
            {
                const auto currentFeature = std::find_if(person.features.begin(), person.features.end(),
                    [&](const B2CandidateFeature& feature) { return feature.candidateId == match.currentCandidateId; });
                if (currentFeature != person.features.end())
                {
                    const int bin = static_cast<int>(std::floor(currentFeature->depthMeters / mConfig.depthBinMeters));
                    const auto binIt = binStatistics.find(bin);
                    const BackgroundStatistics& reference = binIt == binStatistics.end() ? fallback : binIt->second;
                    match.normalizedR3D = std::max(0.0f, match.rawR3D - reference.median) / reference.sigma;
                    normalized.push_back(match.normalizedR3D);
                }
            }
        }
        person.validGeometryCount = static_cast<int>(raw.size());
        person.validMatchRatio = person.matchedCount > 0 ?
            static_cast<float>(person.validGeometryCount) / person.matchedCount : 0.0f;
        person.validGeometryRatio = person.validMatchRatio;
        if (!raw.empty())
        {
            person.rawR3DMedian = Median(raw);
            float sum = 0.0f;
            for (const float value : raw) sum += value;
            person.rawR3DMean = sum / raw.size();
            std::vector<float> deviations;
            for (const float value : raw) deviations.push_back(std::fabs(value - person.rawR3DMedian));
            person.rawR3DMad = Median(deviations);
            person.r2DMedian = Median(r2d);
        }
        if (!normalized.empty())
        {
            person.normalizedR3DMedian = Median(normalized);
            int moving = 0;
            for (const float value : normalized)
                if (value >= mConfig.highResidualNormThreshold) ++moving;
            person.movingFraction = static_cast<float>(moving) / normalized.size();
        }

        // Reliability is resolved first. Keep GEOM_UNRELIABLE (set above)
        // for every failed gate; residual magnitudes cannot override a failure.
        if (!mbPosePairReliable)
            person.geometryFailureReason = POSE_WEAK;
        else if (person.candidateCount <= 0)
            person.geometryFailureReason = INSUFFICIENT_CANDIDATES;
        else if (person.matchedCount < mConfig.minMatchedFeatures)
            person.geometryFailureReason = INSUFFICIENT_MATCHES;
        else if (person.matchRatio < mConfig.minMatchRatio)
            person.geometryFailureReason = LOW_MATCH_RATIO;
        else if (person.validGeometryCount < mConfig.minValidFeatures)
            person.geometryFailureReason = INSUFFICIENT_VALID_DEPTH;
        else if (person.validMatchRatio < mConfig.minValidMatchRatio)
            person.geometryFailureReason = LOW_VALID_MATCH_RATIO;
        else if (!backgroundReferenceValid || normalized.size() != raw.size())
            person.geometryFailureReason = INSUFFICIENT_BACKGROUND_REFERENCE;
        else
        {
            // Only reliable instances enter the three-zone decision.
            const bool moving =
                person.rawR3DMedian >= mConfig.rawMovingThreshold &&
                person.normalizedR3DMedian >= mConfig.normMovingThreshold &&
                person.movingFraction >= mConfig.highRatioMovingThreshold;
            const bool stationary =
                person.rawR3DMedian <= mConfig.rawStaticThreshold &&
                person.normalizedR3DMedian <= mConfig.normStaticThreshold &&
                person.movingFraction <= mConfig.highRatioStaticThreshold;

            if (moving)
                person.geometryDecision = GEOM_MOVING_CANDIDATE;
            else if (stationary)
                person.geometryDecision = GEOM_STATIC_CANDIDATE;
            else
            {
                person.geometryDecision = GEOM_UNCERTAIN;
                person.geometryFailureReason = AMBIGUOUS_GEOMETRIC_EVIDENCE;
            }
        }
        if (mConfig.debugEnable && mConfig.printFrameSummary)
            std::cout << "[B2 SUMMARY] frame=" << current.frameId
                      << " track_id=" << person.trackId
                      << " instance_id=" << person.instanceId
                      << " candidate=" << person.candidateCount
                      << " matched=" << person.matchedCount
                      << " valid=" << person.validGeometryCount
                      << " match_ratio=" << person.matchRatio
                      << " valid_match_ratio=" << person.validMatchRatio
                      << " bg_n=" << person.backgroundSampleCount
                      << " bg_median=" << person.backgroundMedianR3D
                      << " bg_sigma=" << person.backgroundSigmaR3D
                      << " raw_median=" << person.rawR3DMedian
                      << " norm_median=" << person.normalizedR3DMedian
                      << " high_ratio=" << person.movingFraction
                      << " state=" << EvidenceName(person.geometryDecision)
                      << " reason=" << FailureReasonName(person.geometryFailureReason)
                      << std::endl;
    }
}

void B2MotionEvidence::MatchAssociatedInstances(B2CandidateFrame& current)
{
    for (B2PersonInstance& currentPerson : current.persons)
    {
        if (currentPerson.previousInstanceId < 0)
            continue;
        const auto previous = std::find_if(mPreviousFrame.persons.begin(),
            mPreviousFrame.persons.end(), [&](const B2PersonInstance& person) {
                return person.instanceId == currentPerson.previousInstanceId &&
                       person.trackId == currentPerson.trackId;
            });
        if (previous != mPreviousFrame.persons.end())
            MatchPersonFeatures(*previous, currentPerson);
    }
}

void B2MotionEvidence::MatchPersonFeatures(const B2PersonInstance& previous,
                                           B2PersonInstance& current) const
{
    current.candidateCount = static_cast<int>(previous.features.size());
    current.forwardMatchCount = 0;
    current.mutualMatchCount = 0;
    current.matchedCount = 0;
    current.depthValidMatchCount = 0;
    current.matches.clear();
    current.matches.reserve(previous.features.size());

    for (size_t previousIndex = 0; previousIndex < previous.features.size(); ++previousIndex)
    {
        const B2CandidateFeature& previousFeature = previous.features[previousIndex];
        B2CandidateMatch match;
        match.previousCandidateId = previousFeature.candidateId;
        match.status = MATCH_INVALID;
        const NearestDescriptor forward = FindNearest(
            previousFeature, current.features, mConfig.loweRatio,
            mConfig.maxHammingDistance);
        if (!forward.accepted)
        {
            current.matches.push_back(match);
            continue;
        }
        const B2CandidateFeature& currentFeature = current.features[forward.index];
        match.currentCandidateId = currentFeature.candidateId;
        match.hammingDistance = forward.distance;
        ++current.forwardMatchCount;

        const NearestDescriptor reverse = FindNearest(
            currentFeature, previous.features, mConfig.loweRatio,
            mConfig.maxHammingDistance);
        const bool mutual = reverse.accepted &&
            reverse.index == static_cast<int>(previousIndex);
        if (mutual)
            ++current.mutualMatchCount;
        if (mConfig.useMutualMatch && !mutual)
        {
            current.matches.push_back(match);
            continue;
        }
        ++current.matchedCount;
        match.descriptorMatched = true;
        if (!previousFeature.depthValid || !currentFeature.depthValid)
        {
            current.matches.push_back(match);
            continue;
        }
        ++current.depthValidMatchCount;
        match.status = MATCH_VALID;
        current.matches.push_back(match);
    }
}

void B2MotionEvidence::AssociateInstances(B2CandidateFrame& current)
{
    mLastEndedTrackIds.clear();
    if (!mbHasPreviousFrame)
    {
        for (B2PersonInstance& person : current.persons)
            person.trackId = mnNextTrackId++;
        return;
    }
    if (!mConfig.associationEnable)
    {
        for (const B2PersonInstance& previous : mPreviousFrame.persons)
            if (previous.trackId >= 0)
                mLastEndedTrackIds.push_back(previous.trackId);
        for (B2PersonInstance& person : current.persons)
            person.trackId = mnNextTrackId++;
        return;
    }

    struct CandidatePair {
        size_t previousIndex;
        size_t currentIndex;
        float score;
    };
    std::vector<CandidatePair> pairs;
    for (size_t previousIndex = 0; previousIndex < mPreviousFrame.persons.size(); ++previousIndex)
    {
        const B2PersonInstance& previous = mPreviousFrame.persons[previousIndex];
        for (size_t currentIndex = 0; currentIndex < current.persons.size(); ++currentIndex)
        {
            const B2PersonInstance& candidate = current.persons[currentIndex];
            if (previous.classId != candidate.classId)
                continue;
            const float iou = BoundingBoxIoU(previous.boundingBox, candidate.boundingBox);
            const float centerDistance = CenterDistance(previous.boundingBox, candidate.boundingBox);
            if (iou < mConfig.minInstanceIoU ||
                centerDistance > mConfig.maxInstanceCenterDistancePixels)
                continue;
            const float centerScore = 1.0f -
                centerDistance / mConfig.maxInstanceCenterDistancePixels;
            const float score = 0.5f * (iou + centerScore);
            if (score >= mConfig.minInstanceAssociationScore)
                pairs.push_back({previousIndex, currentIndex, score});
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const CandidatePair& a, const CandidatePair& b) {
        return a.score > b.score;
    });

    std::vector<bool> previousUsed(mPreviousFrame.persons.size(), false);
    std::vector<bool> currentUsed(current.persons.size(), false);
    for (const CandidatePair& pair : pairs)
    {
        if (previousUsed[pair.previousIndex] || currentUsed[pair.currentIndex])
            continue;
        const B2PersonInstance& previous = mPreviousFrame.persons[pair.previousIndex];
        B2PersonInstance& candidate = current.persons[pair.currentIndex];
        candidate.trackId = previous.trackId;
        candidate.previousInstanceId = previous.instanceId;
        candidate.associationScore = pair.score;
        previousUsed[pair.previousIndex] = true;
        currentUsed[pair.currentIndex] = true;
    }
    for (size_t previousIndex = 0; previousIndex < previousUsed.size(); ++previousIndex)
        if (!previousUsed[previousIndex] && mPreviousFrame.persons[previousIndex].trackId >= 0)
            mLastEndedTrackIds.push_back(mPreviousFrame.persons[previousIndex].trackId);
    for (size_t currentIndex = 0; currentIndex < currentUsed.size(); ++currentIndex)
        if (!currentUsed[currentIndex])
            current.persons[currentIndex].trackId = mnNextTrackId++;
}

} // namespace ORB_SLAM3
