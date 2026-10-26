#include "FrameSemanticAnnotator.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ORB_SLAM3 {
void UpdateFeatureTemporalStates(Frame& frame,
    const std::unordered_map<int, TemporalState>& currentStates) {
    if (!frame.mbSemanticAnnotated) return;
    AssertRGBDFeatureAlignment(frame);
    for (int i = 0; i < frame.N; ++i) {
        const auto state = currentStates.find(frame.mvTrackId[i]);
        frame.mvCurrentB4State[i] = state == currentStates.end() ? TemporalState::UNKNOWN : state->second;
    }
}

void AssertRGBDFeatureAlignment(const Frame& frame) {
    const size_t n = static_cast<size_t>(frame.N);
    if (!frame.SemanticMetadataAligned() || frame.mvKeys.size() != n ||
        frame.mvKeysUn.size() != n || frame.mDescriptors.rows != frame.N ||
        frame.mvpMapPoints.size() != n || frame.mvbOutlier.size() != n)
        throw std::runtime_error("RGB-D feature/semantic index alignment failed");
}

namespace {
bool IsDepthEdge(const cv::Mat& depth, int x, int y, const FeatureSemanticConfig& config) {
    if (!config.depthEdgeEnable || depth.empty()) return false;
    const float center = depth.at<float>(y,x);
    if (!std::isfinite(center) || center <= 0.f) return false;
    const float limit = std::max(config.depthEdgeAbsoluteMeters, config.depthEdgeRelative * center);
    for (int v = std::max(0,y-config.depthEdgeRadius); v <= std::min(depth.rows-1,y+config.depthEdgeRadius); ++v)
        for (int u = std::max(0,x-config.depthEdgeRadius); u <= std::min(depth.cols-1,x+config.depthEdgeRadius); ++u) {
            const float neighbor = depth.at<float>(v,u);
            if (std::isfinite(neighbor) && neighbor > 0.f && std::abs(neighbor-center) > limit)
                return true;
        }
    return false;
}
}

void AnnotateRGBDFeatures(Frame& frame, const cv::Size& imageSize,
    const cv::Mat& depthMeters, const std::vector<YOLOSegInstance>& persons,
    const std::unordered_map<int, TemporalState>& previousStates,
    const FeatureSemanticConfig& config) {
    if (!config.enable) return;
    AssertRGBDFeatureAlignment(frame);
    if (imageSize.width <= 0 || imageSize.height <= 0 ||
        (!depthMeters.empty() && (depthMeters.type() != CV_32FC1 || depthMeters.size() != imageSize)))
        throw std::runtime_error("Feature annotation requires extraction-size CV_32FC1 metre depth");
    std::vector<cv::Mat> masks, distances;
    for (const auto& person : persons) {
        if (person.mask.type() != CV_8UC1 || person.mask.size() != imageSize)
            throw std::runtime_error("Feature annotation mask/image coordinate mismatch");
        cv::Mat binary, padded, distance;
        cv::compare(person.mask, 0, binary, cv::CMP_GT);
        if (person.classId != 0) binary.setTo(0); // Only person is annotated here.
        // Treat clipped silhouettes at image edges as boundaries too.
        cv::copyMakeBorder(binary, padded, 1,1,1,1,cv::BORDER_CONSTANT,cv::Scalar(0));
        cv::distanceTransform(padded, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        masks.push_back(binary);
        distances.push_back(distance(cv::Rect(1,1,imageSize.width,imageSize.height)).clone());
    }
    frame.ResetSemanticMetadata();
    for (int i = 0; i < frame.N; ++i) {
        const cv::Point2f pixel = frame.mvKeys[i].pt;
        if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || pixel.x < 0.f || pixel.y < 0.f ||
            pixel.x >= imageSize.width || pixel.y >= imageSize.height) continue;
        const int x = cvRound(pixel.x), y = cvRound(pixel.y);
        if (x >= imageSize.width || y >= imageSize.height) continue;
        int owner = -1;
        bool ambiguous = false;
        for (size_t p = 0; p < masks.size(); ++p)
            if (masks[p].at<uchar>(y,x)) {
                if (owner >= 0) { ambiguous = true; break; }
                owner = static_cast<int>(p);
            }
        if (ambiguous) continue; // Overlap: no unique instance/track assignment.
        if (owner < 0) { frame.mvSemanticRegion[i] = SemanticRegion::BACKGROUND; continue; }
        const auto& person = persons[owner];
        if (person.trackId < 0) continue;
        frame.mvInstanceId[i] = owner;
        frame.mvTrackId[i] = person.trackId;
        const auto state = previousStates.find(person.trackId);
        if (state != previousStates.end())
            frame.mvPreviousB4State[i] = frame.mvCurrentB4State[i] = state->second;
        frame.mvSemanticRegion[i] = distances[owner].at<float>(y,x) > config.safeBoundaryDistancePixels &&
            !IsDepthEdge(depthMeters,x,y,config) ? SemanticRegion::PERSON_INTERIOR : SemanticRegion::PERSON_BOUNDARY;
    }
    frame.mbSemanticAnnotated = true;
    frame.mbSemanticOverlayEnabled = config.showOverlay;
    // All policies stay USE_NORMAL and all dynamic exclusions stay false.
    AssertRGBDFeatureAlignment(frame);
}
}
