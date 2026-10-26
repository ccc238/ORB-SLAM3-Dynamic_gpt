/**
 * B2 per-person motion-evidence data.
 *
 * These types intentionally do not live in Frame.  B2 candidates are an
 * auxiliary measurement stream and must never change the indexing of the
 * ORB features used by tracking, matching, or map-point association.
 */
#ifndef ORB_SLAM3_B2_MOTION_EVIDENCE_H
#define ORB_SLAM3_B2_MOTION_EVIDENCE_H

#include "Deep/DynamicDetector.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sophus/se3.hpp>

#include <memory>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace ORB_SLAM3 {

class ORBextractor;

enum B2DepthStatus {
    DEPTH_NOT_EVALUATED = 0,
    DEPTH_VALID = 1,
    DEPTH_INVALID = 2
};

enum B2MatchStatus {
    MATCH_NOT_EVALUATED = 0,
    MATCH_INVALID = 1,
    MATCH_VALID = 2
};

enum B2GeometryStatus {
    GEOMETRY_NOT_EVALUATED = 0,
    GEOMETRY_INVALID = 1,
    GEOMETRY_VALID = 2
};

enum B2InstanceGeometryStatus {
    GEOM_NOT_EVALUATED = 0,
    GEOM_UNRELIABLE = 1,
    GEOM_STATIC_CANDIDATE = 2,
    GEOM_MOVING_CANDIDATE = 3,
    GEOM_UNCERTAIN = 4
};

enum B2GeometryFailureReason {
    GEOM_FAILURE_NONE = 0,
    INSUFFICIENT_CANDIDATES = 1,
    INSUFFICIENT_MATCHES = 2,
    LOW_MATCH_RATIO = 3,
    INSUFFICIENT_VALID_DEPTH = 4,
    LOW_VALID_MATCH_RATIO = 5,
    INSUFFICIENT_BACKGROUND_REFERENCE = 6,
    AMBIGUOUS_GEOMETRIC_EVIDENCE = 7,
    POSE_WEAK = 8
};

struct B2CandidateFeature {
    int candidateId = -1;
    int instanceId = -1;

    cv::KeyPoint keypoint;
    cv::Mat descriptor;

    float depthMeters = -1.0f;
    float boundaryDistance = 0.0f;

    bool depthValid = false;
    B2DepthStatus depthStatus = DEPTH_NOT_EVALUATED;
    bool nearMaskBoundary = false;
    bool nearDepthEdge = false;
};

struct B2CandidateMatch {
    int previousCandidateId = -1;
    int currentCandidateId = -1;
    int hammingDistance = -1;
    // True only after the configured descriptor/mutual-match gate.  It
    // remains true for depth-invalid matches so Viewer can distinguish gray
    // invalid evidence from blue unmatched candidates.
    bool descriptorMatched = false;
    B2MatchStatus status = MATCH_NOT_EVALUATED;
    B2GeometryStatus geometryStatus = GEOMETRY_NOT_EVALUATED;
    float rawR3D = -1.0f;
    float normalizedR3D = -1.0f;
    float r2D = -1.0f;
};

// A Tracking inlier snapshot.  It is copied into B2's own frame record, so
// no B2 temporal logic relies on Tracking::mLastFrame after Track() returns.
struct B2BackgroundObservation {
    unsigned long mapPointId = 0;
    cv::Point2f pixel;
    float depthMeters = -1.0f;
};

struct B2PersonInstance {
    int instanceId = -1;
    int trackId = -1;
    int previousInstanceId = -1;
    int classId = -1;
    float confidence = 0.0f;
    float associationScore = 0.0f;

    // Per associated-person matching diagnostics. candidateCount is the
    // number of previous-frame candidate descriptors queried forward;
    // matchedCount is post descriptor/mutual matching and pre depth check;
    // validGeometryCount is the subset with a usable 3-D residual.
    int candidateCount = 0;
    int forwardMatchCount = 0;
    int mutualMatchCount = 0;
    int matchedCount = 0;
    int depthValidMatchCount = 0;
    int validGeometryCount = 0;
    float matchRatio = 0.0f;
    float validMatchRatio = 0.0f;
    // Compatibility name for diagnostics consumers; it has the explicit
    // validGeometryCount / matchedCount denominator.
    float validGeometryRatio = 0.0f;
    int backgroundSampleCount = 0;
    float backgroundMedianR3D = -1.0f;
    float backgroundSigmaR3D = -1.0f;
    float rawR3DMedian = -1.0f;
    float rawR3DMean = -1.0f;
    float rawR3DMad = -1.0f;
    float normalizedR3DMedian = -1.0f;
    float r2DMedian = -1.0f;
    float movingFraction = -1.0f;
    B2InstanceGeometryStatus geometryDecision = GEOM_NOT_EVALUATED;
    B2GeometryFailureReason geometryFailureReason = GEOM_FAILURE_NONE;

    cv::Rect boundingBox;
    cv::Mat mask;

    std::vector<B2CandidateFeature> features;
    std::vector<B2CandidateMatch> matches;
};

struct B2CandidateFrame {
    long frameId = -1;
    double timestamp = 0.0;

    bool poseValid = false;
    Sophus::SE3f Tcw;
    // Prevent a coarse history from being silently paired with a final pose.
    std::string poseSource;
    cv::Size imageSize;
    std::vector<B2BackgroundObservation> backgroundObservations;

    std::vector<B2PersonInstance> persons;
};

// Immutable result exported by the evidence-only path.  It contains copied
// B2 diagnostics only; it never exposes or owns a SLAM Frame/MapPoint.
struct B2Result {
    long frameId = -1;
    bool processed = false;
    double processingMs = 0.0;
    std::vector<B2PersonInstance> persons;
};

// Keeps B2's temporal state separate from Tracking::mLastFrame.  The latter
// is updated within Track(), so it is not a reliable ownership boundary for a
// consecutive B2 evidence pair.
class B2MotionEvidence {
public:
    explicit B2MotionEvidence(const B2Config& config);
    // B2 RGB-D geometry currently uses the same calibrated pinhole K as the
    // RGB-D Frame. K must be a finite 3x3 matrix with positive focal lengths.
    void SetCameraCalibration(const cv::Mat& K);

    // RGB-D-only candidate stage. semanticInstances contain raw, individual
    // YOLO masks; this function erodes each one and never accepts B1's
    // union/dilated reject mask.
    B2CandidateFrame ExtractCandidates(const cv::Mat& grayImage,
                                       const cv::Mat& depthMeters,
                                       const std::vector<YOLOSegInstance>& semanticInstances,
                                       double timestamp,
                                       const ORBextractor& mainExtractor);

    // Read-only evidence boundary: work is performed on an internal copy and
    // the result contains no mutable SLAM state.  This function must not
    // change Frame, optimizer, matcher, local-mapping, or map objects.
    B2Result Process(const B2CandidateFrame& current);
    // Explicit immutable evidence input; nullptr previous is the bootstrap.
    // currentPoseReliable is the CURRENT pose gate; previous.poseValid remains
    // a separate gate. The supplied relative pose is used for person AND
    // background residuals, never silently reconstructed from absolute Tcw.
    B2Result Process(const B2CandidateFrame* previous, const B2CandidateFrame& current,
                     const Sophus::SE3f& currentFromPrevious, bool currentPoseReliable);
    B2Result PushCurrentFrame(const B2CandidateFrame& current) { return Process(current); }

    bool HasPreviousFrame() const { return mbHasPreviousFrame; }
    const B2CandidateFrame& PreviousFrame() const { return mPreviousFrame; }
    const std::vector<int>& LastEndedTrackIds() const { return mLastEndedTrackIds; }
    const B2Config& Config() const { return mConfig; }
    const std::string& CsvOutputDirectory() const { return mCsvOutputDirectory; }

private:
    B2Result ProcessInput(const B2CandidateFrame* previous, const B2CandidateFrame& current,
                         const Sophus::SE3f& currentFromPrevious, bool currentPoseReliable,
                         bool explicitInput);
    void EnsureCandidateExtractor(const ORBextractor& mainExtractor);
    float SampleDepth(const cv::Mat& depthMeters, const cv::Point2f& point,
                      bool& valid) const;
    void AssociateInstances(B2CandidateFrame& current);
    void MatchAssociatedInstances(B2CandidateFrame& current);
    void MatchPersonFeatures(const B2PersonInstance& previous,
                             B2PersonInstance& current) const;
    void ComputeGeometryResiduals(B2CandidateFrame& current) const;
    void ComputeInstanceDecisions(B2CandidateFrame& current) const;
    bool BackProject(const cv::Point2f& pixel, float depth, Eigen::Vector3f& point) const;
    bool Project(const Eigen::Vector3f& point, const cv::Size& imageSize,
                 cv::Point2f& pixel) const;
    void InitializeCsvOutput();
    void WriteCsv(const B2CandidateFrame& current, const B2Result& result,
                  double processingMs);

    B2Config mConfig;
    std::unique_ptr<ORBextractor> mpCandidateExtractor;
    float mFx = 0.0f, mFy = 0.0f, mCx = 0.0f, mCy = 0.0f;
    bool mbCameraCalibrated = false;
    B2CandidateFrame mPreviousFrame;
    bool mbHasPreviousFrame = false;
    Sophus::SE3f mCurrentFromPrevious;
    bool mbPosePairReliable = false;
    bool mbExplicitPoseInput = false;
    int mnNextTrackId = 0;
    std::vector<int> mLastEndedTrackIds;
    std::string mCsvOutputDirectory;
    std::ofstream mCandidateCsv;
    std::ofstream mInstanceCsv;
};

} // namespace ORB_SLAM3

#endif
