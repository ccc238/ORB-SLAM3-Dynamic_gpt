#include "Deep/DynamicDetector.h"
#include "Deep/MaskPreview.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
#include <stdexcept>
#include <cstdlib>
#include <cstdio>
#include <cmath>

namespace ORB_SLAM3 {
namespace {
bool Flag(const cv::FileStorage& fs, const char* name, bool fallback = false) {
    auto n = fs[name];
    if (n.empty()) return fallback;
    if (!n.isInt() || (int(n) != 0 && int(n) != 1))
        throw std::runtime_error(std::string(name) + " must be 0 or 1");
    return int(n) == 1;
}
int Integer(const cv::FileStorage& fs, const char* key, int fallback) {
    auto n = fs[key];
    if (n.empty()) return fallback;
    if (!n.isInt()) throw std::runtime_error(std::string(key) + " must be an integer");
    return int(n);
}
float Number(const cv::FileStorage& fs, const char* key, float fallback) {
    auto n = fs[key];
    if (n.empty()) return fallback;
    if (!n.isInt() && !n.isReal())
        throw std::runtime_error(std::string(key) + " must be a number");
    const float value = float(n.real());
    if (!std::isfinite(value))
        throw std::runtime_error(std::string(key) + " must be finite");
    return value;
}
double Seconds(const cv::FileStorage& fs, const char* key, double fallback) {
    const auto n = fs[key];
    if (n.empty()) return fallback;
    if (!n.isInt() && !n.isReal())
        throw std::runtime_error(std::string(key) + " must be a number");
    const double value = n.real();
    if (!std::isfinite(value))
        throw std::runtime_error(std::string(key) + " must be finite");
    return value;
}
void ValidateB2(const B2Config& b2) {
    if (b2.maxFeaturesPerInstance <= 0)
        throw std::runtime_error("B2.Candidate.MaxFeaturesPerInstance must be > 0");
    if (b2.candidateErodeRadius < 0 || b2.minBoundaryDistance < 0)
        throw std::runtime_error("B2 candidate radii/distances must be >= 0");
    if (b2.loweRatio <= 0.f || b2.loweRatio >= 1.f)
        throw std::runtime_error("B2.Association.LoweRatio must be in (0, 1)");
    if (b2.maxHammingDistance < 0 || b2.maxHammingDistance > 256)
        throw std::runtime_error("B2.Association.MaxHammingDistance must be in [0, 256]");
    if (b2.minInstanceIoU < 0.f || b2.minInstanceIoU > 1.f ||
        b2.maxInstanceCenterDistancePixels <= 0.f ||
        b2.minInstanceAssociationScore < 0.f || b2.minInstanceAssociationScore > 1.f)
        throw std::runtime_error("B2 instance-association thresholds are invalid");
    if (b2.minDepthMeters <= 0.f || b2.maxDepthMeters <= b2.minDepthMeters)
        throw std::runtime_error("B2.Geometry depth range is invalid");
    if (b2.minMatchedFeatures <= 0 || b2.minMatchRatio <= 0.f || b2.minMatchRatio > 1.f ||
        b2.minValidFeatures <= 0 || b2.minValidMatchRatio <= 0.f || b2.minValidMatchRatio > 1.f)
        throw std::runtime_error("B2.Geometry evidence thresholds are invalid");
    if (b2.backgroundExclusionDilateRadius < 0)
        throw std::runtime_error("B2.Geometry.BackgroundExclusionDilateRadius must be >= 0");
    if (b2.geometryDecisionMethod != "background_mad")
        throw std::runtime_error("B2.Geometry.Decision.Method must be background_mad");
    if (b2.depthBinMeters <= 0.f || b2.minSamplesPerBin <= 0 ||
        b2.minBackgroundSamples <= 0 || b2.sigmaFloorMeters <= 0.f ||
        !std::isfinite(b2.rawMovingThreshold) || b2.rawMovingThreshold < 0.f ||
        !std::isfinite(b2.normMovingThreshold) || b2.normMovingThreshold < 0.f ||
        !std::isfinite(b2.highRatioMovingThreshold) ||
        b2.highRatioMovingThreshold < 0.f || b2.highRatioMovingThreshold > 1.f ||
        b2.rawStaticThreshold < 0.f ||
        !std::isfinite(b2.normStaticThreshold) || b2.normStaticThreshold < 0.f ||
        !std::isfinite(b2.highRatioStaticThreshold) ||
        b2.highRatioStaticThreshold < 0.f || b2.highRatioStaticThreshold > 1.f ||
        (std::isfinite(b2.rawStaticThreshold) && b2.rawStaticThreshold >= b2.rawMovingThreshold) ||
        b2.normStaticThreshold >= b2.normMovingThreshold ||
        !std::isfinite(b2.highResidualNormThreshold) || b2.highResidualNormThreshold < 0.f)
        throw std::runtime_error("B2.Geometry.Decision thresholds are invalid");
}
void ValidateFrontend(const cv::FileStorage& fs, const DynamicFrontendConfig& frontend,
                      bool yolo, bool rejectMask, const B2Config& b2,
                      const B3Config& b3, const B4Config& b4)
{
    const char* keys[] = {"TwoStage.Enable", "TwoStage.CoarseFilter.Enable",
        "TwoStage.UseCoarsePoseForB2", "TwoStage.FinePolicy.Enable",
        "TwoStage.MapAdmission.Enable", "TwoStage.Fallback.Enable"};
    if (!frontend.explicitMode)
    {
        for (const char* key : keys)
            if (!fs[key].empty())
                throw std::runtime_error("TwoStage.* requires an explicit DynamicFrontend.Mode");
        return;
    }
    const int mode = static_cast<int>(frontend.mode);
    if (mode < 0 || mode > 3)
        throw std::runtime_error("DynamicFrontend.Mode must be an integer in [0,3]");
    if (mode <= 1)
    {
        if (frontend.twoStageEnable || frontend.coarseFilterEnable || frontend.useCoarsePoseForB2 ||
            frontend.finePolicyEnable || frontend.mapAdmissionEnable || b2.enable || b3.enable || b4.enable)
            throw std::runtime_error("DynamicFrontend.Mode=0/1 requires TwoStage.Enable=0, "
                "all action submodules=0 and B2/B3/B4.Enable=0");
        if (mode == 0 && (yolo || rejectMask))
            throw std::runtime_error("DynamicFrontend.Mode=0 requires Yolo.Enable=0 and MaskFeatureReject.Enable=0");
        if (mode == 1 && (!yolo || !rejectMask))
            throw std::runtime_error("DynamicFrontend.Mode=1 requires Yolo.Enable=1 and MaskFeatureReject.Enable=1");
    }
    else
    {
        if (!frontend.twoStageEnable || !yolo || rejectMask || !b2.enable || !b3.enable || !b4.enable)
            throw std::runtime_error("DynamicFrontend.Mode=2/3 requires TwoStage.Enable=1, Yolo.Enable=1, "
                "MaskFeatureReject.Enable=0 and B2/B3/B4.Enable=1");
        if (!b2.evidenceOnly || !b3.evidenceOnly || (mode==2 && !b4.evidenceOnly))
            throw std::runtime_error("DynamicFrontend.Mode=2 requires B2/B3/B4.EvidenceOnly=1; Mode=3 keeps B2/B3 read-only");
        if (mode == 2 && (frontend.coarseFilterEnable ||
            frontend.finePolicyEnable || frontend.mapAdmissionEnable))
            throw std::runtime_error("TwoStage coarse filtering/fine policy/map admission are not "
                "implemented in Mode=2 evidence-only; use Mode=3 for supported actions");
        if (mode == 3 && (!frontend.coarseFilterEnable || !frontend.useCoarsePoseForB2 ||
            !frontend.fallbackEnable || !frontend.featureSemantics.enable || !frontend.featurePolicy.enable ||
            frontend.featurePolicy.evidenceOnly))
            throw std::runtime_error("Mode=3 full ACTION is not implemented; batch9 coarse-only requires "
                "CoarseFilter=1, UseCoarsePoseForB2=1, Fallback=1, FeatureSemantic.Enable=1, "
                "FeaturePolicy.Enable=1, FeaturePolicy.EvidenceOnly=0");
        if(mode==3 && frontend.mapAdmissionEnable && !frontend.finePolicyEnable)
            throw std::runtime_error("MapAdmission requires TwoStage.FinePolicy.Enable=1; coarse-only map admission is not implemented");
        if(mode==3 && b4.evidenceOnly==frontend.finePolicyEnable)
            throw std::runtime_error("B4.EvidenceOnly must be 0 iff TwoStage.FinePolicy.Enable=1 in Mode=3");
        if(mode==3 && frontend.finePolicyEnable && (b4.pendingUseForPose ||
            b4.pendingAllowMapCreation || b4.dynamicAllowMapCreation))
            throw std::runtime_error("FinePolicy requires B4.Pending.UseForPose=0 and reserved map-creation switches=0; no pending emergency support");
    }
}
cv::Mat ToBGR(const cv::Mat& image, bool rgb) {
    cv::Mat bgr;
    if (image.type() == CV_8UC3) {
        if (rgb) cv::cvtColor(image,bgr,cv::COLOR_RGB2BGR);
        else bgr = image;
    } else if (image.type() == CV_8UC1) cv::cvtColor(image,bgr,cv::COLOR_GRAY2BGR);
    else if (image.type() == CV_8UC4)
        cv::cvtColor(image,bgr,rgb ? cv::COLOR_RGBA2BGR : cv::COLOR_BGRA2BGR);
    else throw std::runtime_error("YOLO unsupported input type");
    return bgr;
}
double Milliseconds(std::chrono::steady_clock::time_point a,
                    std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double,std::milli>(b-a).count();
}
std::string ShellQuote(const std::string& value) {
    std::string quoted("'");
    for (const char character : value)
        quoted += character == '\'' ? "'\\''" : std::string(1, character);
    return quoted + "'";
}
std::string FileSha256(const std::string& path) {
    if (path.empty()) return "";
    const std::string command = "sha256sum -- " + ShellQuote(path);
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) throw std::runtime_error("Cannot start sha256sum for " + path);
    char buffer[256] = {0};
    const bool read = fgets(buffer, sizeof(buffer), pipe) != nullptr;
    const int status = pclose(pipe);
    if (!read || status != 0) throw std::runtime_error("Cannot hash file " + path);
    std::istringstream row(buffer);
    std::string hash;
    row >> hash;
    if (hash.size() != 64) throw std::runtime_error("Invalid SHA256 output for " + path);
    return hash;
}
}
DynamicDetector::DynamicDetector(const std::string& settingsPath) {
    cv::FileStorage fs(settingsPath,cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("Cannot open settings: " + settingsPath);
    frontend_.explicitMode = !fs["DynamicFrontend.Mode"].empty();
    frontend_.mode = static_cast<DynamicFrontendMode>(Integer(fs,"DynamicFrontend.Mode",0));
    frontend_.twoStageEnable = Flag(fs,"TwoStage.Enable",false);
    frontend_.coarseFilterEnable = Flag(fs,"TwoStage.CoarseFilter.Enable",false);
    frontend_.useCoarsePoseForB2 = Flag(fs,"TwoStage.UseCoarsePoseForB2",false);
    frontend_.finePolicyEnable = Flag(fs,"TwoStage.FinePolicy.Enable",false);
    frontend_.mapAdmissionEnable = Flag(fs,"TwoStage.MapAdmission.Enable",false);
    frontend_.fallbackEnable = Flag(fs,"TwoStage.Fallback.Enable",true);
    frontend_.diagnosticSaveCsv = Flag(fs,"DynamicFrontend.Debug.SaveCsv",false);
    frontend_.diagnosticShowOverlay = Flag(fs,"DynamicFrontend.Debug.ShowOverlay",false);
    frontend_.diagnosticSaveOverlay = Flag(fs,"DynamicFrontend.Debug.SaveOverlay",false);
    std::cout << "[FRONTEND DIAGNOSTICS] SaveCsv=" << frontend_.diagnosticSaveCsv
              << " ShowOverlay=" << frontend_.diagnosticShowOverlay
              << " SaveOverlay=" << frontend_.diagnosticSaveOverlay << std::endl;
    frontend_.minCoarseInliers = Integer(fs,"TwoStage.MinCoarseInliers",30);
    frontend_.minCoarseInlierRatio = Number(fs,"TwoStage.MinCoarseInlierRatio",0.30f);
    if (frontend_.minCoarseInliers < 1 || frontend_.minCoarseInlierRatio <= 0.f ||
        frontend_.minCoarseInlierRatio > 1.f)
        throw std::runtime_error("TwoStage coarse quality thresholds require inliers>0 and ratio in (0,1]");
    if (frontend_.useCoarsePoseForB2 && !frontend_.fallbackEnable)
        throw std::runtime_error("TwoStage.UseCoarsePoseForB2 requires TwoStage.Fallback.Enable=1");
    auto& annotation = frontend_.featureSemantics;
    annotation.enable = Flag(fs,"FeatureSemantic.Enable",false);
    annotation.safeBoundaryDistancePixels = Number(fs,"FeatureSemantic.SafeBoundaryDistancePixels",3.f);
    annotation.depthEdgeEnable = Flag(fs,"FeatureSemantic.DepthEdge.Enable",true);
    annotation.depthEdgeRadius = Integer(fs,"FeatureSemantic.DepthEdge.Radius",1);
    annotation.depthEdgeAbsoluteMeters = Number(fs,"FeatureSemantic.DepthEdge.AbsoluteMeters",.05f);
    annotation.depthEdgeRelative = Number(fs,"FeatureSemantic.DepthEdge.Relative",.02f);
    annotation.showOverlay = Flag(fs,"FeatureSemantic.ShowOverlay",true);
    auto& policy = frontend_.featurePolicy;
    policy.enable = Flag(fs,"FeaturePolicy.Enable",false);
    policy.evidenceOnly = Flag(fs,"FeaturePolicy.EvidenceOnly",true);
    policy.saveSummaryCsv = Flag(fs,"FeaturePolicy.SaveSummaryCsv",true);
    policy.printFrameSummary = Flag(fs,"FeaturePolicy.PrintFrameSummary",true);
    enabled_ = Flag(fs,"Yolo.Enable");
    // New key defaults to true: absent key preserves the previous hard-mask
    // behavior for every existing YOLO-enabled configuration.
    maskFeatureRejectEnabled_ = Flag(fs,"MaskFeatureReject.Enable",
        !frontend_.explicitMode || frontend_.mode == DynamicFrontendMode::B1_HARD_MASK);
    b2_.enable = Flag(fs,"B2.Enable",false);
    b2_.evidenceOnly = Flag(fs,"B2.EvidenceOnly",true);
    b2_.candidateEnable = Flag(fs,"B2.Candidate.Enable",true);
    b2_.maxFeaturesPerInstance = Integer(fs,"B2.Candidate.MaxFeaturesPerInstance",300);
    b2_.candidateErodeRadius = Integer(fs,"B2.Candidate.ErodeRadius",3);
    b2_.minBoundaryDistance = Integer(fs,"B2.Candidate.MinBoundaryDistance",3);
    b2_.associationEnable = Flag(fs,"B2.Association.Enable",true);
    b2_.useMutualMatch = Flag(fs,"B2.Association.UseMutualMatch",true);
    b2_.loweRatio = Number(fs,"B2.Association.LoweRatio",0.75f);
    b2_.maxHammingDistance = Integer(fs,"B2.Association.MaxHammingDistance",64);
    b2_.minInstanceIoU = Number(fs,"B2.Association.MinInstanceIoU",0.10f);
    b2_.maxInstanceCenterDistancePixels = Number(fs,"B2.Association.MaxCenterDistancePixels",100.0f);
    b2_.minInstanceAssociationScore = Number(fs,"B2.Association.MinInstanceScore",0.30f);
    b2_.geometryEnable = Flag(fs,"B2.Geometry.Enable",true);
    b2_.useDepthMedian3x3 = Flag(fs,"B2.Geometry.UseDepthMedian3x3",true);
    b2_.minDepthMeters = Number(fs,"B2.Geometry.MinDepthMeters",0.30f);
    b2_.maxDepthMeters = Number(fs,"B2.Geometry.MaxDepthMeters",5.00f);
    b2_.useNormalizedResidual = Flag(fs,"B2.Geometry.UseNormalizedResidual",true);
    b2_.minMatchedFeatures = Integer(fs,"B2.Geometry.MinMatchedFeatures",30);
    b2_.minMatchRatio = Number(fs,"B2.Geometry.MinMatchRatio",0.15f);
    // Pre-split names are accepted only as a migration path.  They described
    // an ambiguous denominator, so configurations must not specify both.
    if (!fs["B2.Geometry.MinValidFeatures"].empty() &&
        !fs["B2.Geometry.MinValidFeaturesPerInstance"].empty())
        throw std::runtime_error("Use only B2.Geometry.MinValidFeatures (legacy MinValidFeaturesPerInstance conflicts)");
    if (!fs["B2.Geometry.MinValidMatchRatio"].empty() &&
        !fs["B2.Geometry.MinValidRatio"].empty())
        throw std::runtime_error("Use only B2.Geometry.MinValidMatchRatio (legacy MinValidRatio conflicts)");
    b2_.minValidFeatures = !fs["B2.Geometry.MinValidFeatures"].empty() ?
        Integer(fs,"B2.Geometry.MinValidFeatures",10) :
        Integer(fs,"B2.Geometry.MinValidFeaturesPerInstance",10);
    b2_.minValidMatchRatio = !fs["B2.Geometry.MinValidMatchRatio"].empty() ?
        Number(fs,"B2.Geometry.MinValidMatchRatio",0.40f) :
        Number(fs,"B2.Geometry.MinValidRatio",0.40f);
    b2_.backgroundExclusionDilateRadius = Integer(
        fs,"B2.Geometry.BackgroundExclusionDilateRadius",3);
    b2_.geometryDecisionEnable = Flag(fs,"B2.Geometry.Decision.Enable",false);
    if (!fs["B2.Geometry.Decision.Method"].empty())
        b2_.geometryDecisionMethod = std::string(fs["B2.Geometry.Decision.Method"]);
    b2_.depthBinMeters = Number(fs,"B2.Geometry.Decision.DepthBinMeters",0.50f);
    b2_.minSamplesPerBin = Integer(fs,"B2.Geometry.Decision.MinSamplesPerBin",30);
    b2_.minBackgroundSamples = Integer(fs,"B2.Geometry.Decision.MinBackgroundSamples",50);
    b2_.sigmaFloorMeters = Number(fs,"B2.Geometry.Decision.SigmaFloorMeters",0.01f);
    // New six-field interface takes precedence.  If absent, reproduce the
    // old rule exactly: the three moving gates and two static gates come from
    // the legacy Decision keys, while static raw residual was unconstrained.
    b2_.rawMovingThreshold = Number(fs,"B2.Geometry.RawMovingThreshold",
                                    Number(fs,"B2.Geometry.Decision.MinRawMovingMeters",0.02f));
    b2_.normMovingThreshold = Number(fs,"B2.Geometry.NormMovingThreshold",
                                     Number(fs,"B2.Geometry.Decision.MovingThreshold",3.5f));
    b2_.highRatioMovingThreshold = Number(fs,"B2.Geometry.HighRatioMovingThreshold",
                                          Number(fs,"B2.Geometry.Decision.MinMovingFraction",0.60f));
    b2_.rawStaticThreshold = Number(fs,"B2.Geometry.RawStaticThreshold",
                                    std::numeric_limits<float>::infinity());
    b2_.normStaticThreshold = Number(fs,"B2.Geometry.NormStaticThreshold",
                                     Number(fs,"B2.Geometry.Decision.StaticThreshold",2.0f));
    b2_.highRatioStaticThreshold = Number(fs,"B2.Geometry.HighRatioStaticThreshold",
                                          Number(fs,"B2.Geometry.Decision.MaxStaticFraction",0.20f));
    b2_.highResidualNormThreshold = Number(fs,"B2.Geometry.Decision.MovingThreshold",3.5f);
    const char* zoneKeys[] = {"B2.Geometry.RawMovingThreshold", "B2.Geometry.NormMovingThreshold",
        "B2.Geometry.HighRatioMovingThreshold", "B2.Geometry.RawStaticThreshold",
        "B2.Geometry.NormStaticThreshold", "B2.Geometry.HighRatioStaticThreshold"};
    bool hasZoneKeys = false;
    for (const char* key : zoneKeys) hasZoneKeys = hasZoneKeys || !fs[key].empty();
    if (hasZoneKeys && !(b2_.rawStaticThreshold < b2_.rawMovingThreshold &&
                        b2_.normStaticThreshold < b2_.normMovingThreshold &&
                        b2_.highRatioStaticThreshold < b2_.highRatioMovingThreshold))
        throw std::runtime_error("B2 three-zone thresholds require RawStatic<RawMoving, "
                                 "NormStatic<NormMoving and HighRatioStatic<HighRatioMoving; "
                                 "set all six B2.Geometry.*Threshold keys");
    if (hasZoneKeys)
        std::cout << "[B2 CONFIG] New Geometry.*Threshold keys override legacy instance gates; "
                     "Decision.MovingThreshold still defines the point-level high_ratio cutoff." << std::endl;
    b2_.debugEnable = Flag(fs,"B2.Debug.Enable",true);
    b2_.saveCsv = Flag(fs,"B2.Debug.SaveCsv",true);
    b2_.saveFeatureCsv = Flag(fs,"B2.Debug.SaveFeatureCsv",true);
    b2_.saveSummaryCsv = Flag(fs,"B2.Debug.SaveSummaryCsv",true);
    b2_.verbosePerFeature = Flag(fs,"B2.Debug.VerbosePerFeature",false);
    b2_.printFrameSummary = Flag(fs,"B2.Debug.PrintFrameSummary",true);
    b2_.showOverlay = Flag(fs,"B2.Debug.ShowOverlay",true);
    b2_.csvOutputParent = fs["B2.Debug.OutputDir"].empty() ? "." :
        std::string(fs["B2.Debug.OutputDir"]);
    b2_.datasetName = fs["B2.Debug.DatasetName"].empty() ? "unknown" :
        std::string(fs["B2.Debug.DatasetName"]);
    b2_.yamlPath = settingsPath;
    b2_.yamlSha256 = FileSha256(settingsPath);
    ValidateB2(b2_);
    b3_.enable = Flag(fs,"B3.Enable",false);
    b3_.evidenceOnly = Flag(fs,"B3.EvidenceOnly",true);
    b3_.personAsDynamicPrior = Flag(fs,"B3.Semantic.PersonAsDynamicPrior",true);
    b3_.printFrameSummary = Flag(fs,"B3.Debug.PrintFrameSummary",true);
    b3_.saveSummaryCsv = Flag(fs,"B3.Debug.SaveSummaryCsv",true);
    b3_.showOverlay = Flag(fs,"B3.Debug.ShowOverlay",true);
    b4_.enable = Flag(fs,"B4.Enable",false);
    b4_.evidenceOnly = Flag(fs,"B4.EvidenceOnly",true);
    b4_.associationEnable = Flag(fs,"B4.Association.Enable",true);
    b4_.dynamicEnterCount = Integer(fs,"B4.DynamicEnterCount",3);
    b4_.staticEnterCount = Integer(fs,"B4.StaticEnterCount",6);
    b4_.dynamicRecoveryCount = Integer(fs,"B4.DynamicRecoveryCount",10);
    b4_.trackTTL = Integer(fs,"B4.TrackTTL",15);
    b4_.evidenceExpiryEnable = Flag(fs,"B4.EvidenceExpiry.Enable",false);
    b4_.maxEvidenceGapSeconds = Seconds(fs,"B4.EvidenceExpiry.MaxGapSeconds",0.20);
    b4_.maxStaticAgeSeconds = Seconds(fs,"B4.EvidenceExpiry.MaxStaticAgeSeconds",0.30);
    ValidateB4EvidenceExpiry(b4_);
    b4_.useContinuousQuality = Flag(fs,"B4.UseContinuousQuality",false);
    b4_.qualityMin = Number(fs,"B4.QualityMin",0.65f);
    b4_.minMaskIoU = Number(fs,"B4.Track.MinMaskIoU",0.05f);
    b4_.minBoxIoU = Number(fs,"B4.Track.MinBoxIoU",0.10f);
    b4_.maxCenterDistance = Number(fs,"B4.Track.MaxCenterDistance",0.15f);
    b4_.pendingUseForPose = Flag(fs,"B4.Pending.UseForPose",false);
    b4_.pendingAllowMapCreation = Flag(fs,"B4.Pending.AllowMapCreation",false);
    b4_.dynamicAllowMapCreation = Flag(fs,"B4.Dynamic.AllowMapCreation",false);
    b4_.printFrameSummary = Flag(fs,"B4.Debug.PrintFrameSummary",true);
    b4_.saveSummaryCsv = Flag(fs,"B4.Debug.SaveSummaryCsv",true);
    b4_.showOverlay = Flag(fs,"B4.Debug.ShowOverlay",true);
    ValidateFrontend(fs,frontend_,enabled_,maskFeatureRejectEnabled_,b2_,b3_,b4_);
    if (policy.enable && (!annotation.enable || !frontend_.explicitMode ||
        (frontend_.mode != DynamicFrontendMode::TWO_STAGE_EVIDENCE_ONLY &&
         frontend_.mode != DynamicFrontendMode::TWO_STAGE_ACTION)))
        throw std::runtime_error("FeaturePolicy.Enable=1 requires FeatureSemantic.Enable=1 and DynamicFrontend.Mode=2/3");
    if (policy.enable && !policy.evidenceOnly && frontend_.mode != DynamicFrontendMode::TWO_STAGE_ACTION)
        throw std::runtime_error("FeaturePolicy.EvidenceOnly=0 is not implemented: policies are statistics only");
    std::cout << "[FEATURE POLICY CONFIG] Enable=" << policy.enable
              << " EvidenceOnly=" << policy.evidenceOnly << " SaveSummaryCsv=" << policy.saveSummaryCsv
              << " PrintFrameSummary=" << policy.printFrameSummary
              << " CoarseAction=" << frontend_.coarseFilterEnable
              << " FineAction=" << frontend_.finePolicyEnable << " MapAction=0" << std::endl;
    if (annotation.enable && (!frontend_.explicitMode ||
        (frontend_.mode != DynamicFrontendMode::TWO_STAGE_EVIDENCE_ONLY &&
         frontend_.mode != DynamicFrontendMode::TWO_STAGE_ACTION)))
        throw std::runtime_error("FeatureSemantic.Enable=1 currently requires DynamicFrontend.Mode=2/3");
    if (annotation.safeBoundaryDistancePixels < 0.f || annotation.depthEdgeRadius < 1 ||
        annotation.depthEdgeRadius > 10 || annotation.depthEdgeAbsoluteMeters < 0.f ||
        annotation.depthEdgeRelative < 0.f ||
        (annotation.depthEdgeAbsoluteMeters == 0.f && annotation.depthEdgeRelative == 0.f))
        throw std::runtime_error("Invalid FeatureSemantic boundary/depth-edge parameters");
    std::cout << "[FEATURE SEMANTIC CONFIG] Enable=" << annotation.enable
              << " SafeBoundaryDistancePixels=" << annotation.safeBoundaryDistancePixels
              << " DepthEdge.Enable=" << annotation.depthEdgeEnable
              << " DepthEdge.Radius=" << annotation.depthEdgeRadius
              << " DepthEdge.AbsoluteMeters=" << annotation.depthEdgeAbsoluteMeters
              << " DepthEdge.Relative=" << annotation.depthEdgeRelative
              << " ShowOverlay=" << annotation.showOverlay << std::endl;
    std::cout << "[DYNAMIC FRONTEND CONFIG] mode="
              << (frontend_.explicitMode ? DynamicFrontendModeName(frontend_.mode) : "LEGACY_UNSPECIFIED")
              << " TwoStage.Enable=" << frontend_.twoStageEnable
              << " CoarseFilter.Enable=" << frontend_.coarseFilterEnable
              << " UseCoarsePoseForB2=" << frontend_.useCoarsePoseForB2
              << " FinePolicy.Enable=" << frontend_.finePolicyEnable
              << " MapAdmission.Enable=" << frontend_.mapAdmissionEnable
              << " Fallback.Enable=" << frontend_.fallbackEnable
              << " MinCoarseInliers=" << frontend_.minCoarseInliers
              << " MinCoarseInlierRatio=" << frontend_.minCoarseInlierRatio << std::endl;
    if (frontend_.explicitMode && frontend_.mode == DynamicFrontendMode::TWO_STAGE_EVIDENCE_ONLY)
        std::cout << "[DYNAMIC FRONTEND STAGE] evidence pose="
                  << (frontend_.useCoarsePoseForB2 ? "quality-gated coarse" : "final Track()")
                  << "; feature filtering and map policies remain disabled" << std::endl;
    if (frontend_.explicitMode && frontend_.mode == DynamicFrontendMode::TWO_STAGE_ACTION)
        std::cout << "[DYNAMIC FRONTEND STAGE] coarse filtering enabled; fine filtering="
                  << frontend_.finePolicyEnable << "; map admission disabled; B2/B3 remain evidence-only; "
                     "B4 policy scope=" << (frontend_.finePolicyEnable ? "FINE_POSE_ONLY" : "EVIDENCE_ONLY") << std::endl;
    if (b2_.enable && !enabled_)
        throw std::runtime_error("B2.Enable=1 requires Yolo.Enable=1");
    if (b2_.enable && !b2_.evidenceOnly)
        throw std::runtime_error("B2 formal decision mode is not implemented yet");
    if (b3_.enable && !enabled_)
        throw std::runtime_error("B3.Enable=1 requires Yolo.Enable=1");
    if (b3_.enable && !b3_.evidenceOnly)
        throw std::runtime_error("B3.EvidenceOnly=0 is not available until the B3 feature-policy stage is implemented");
    if (b4_.enable && (!b2_.enable || !b3_.enable))
        throw std::runtime_error("B4.Enable=1 requires B2.Enable=1 and B3.Enable=1");
    if (b4_.enable && !b4_.evidenceOnly && (!frontend_.explicitMode ||
        frontend_.mode!=DynamicFrontendMode::TWO_STAGE_ACTION || !frontend_.finePolicyEnable))
        throw std::runtime_error("B4.EvidenceOnly=0 requires Mode=3 and TwoStage.FinePolicy.Enable=1");
    if (b4_.enable && b4_.useContinuousQuality)
        throw std::runtime_error("B4.UseContinuousQuality=1 is not available: no measured continuous quality score is implemented");
    if (b4_.minMaskIoU < 0.f || b4_.minMaskIoU > 1.f || b4_.minBoxIoU < 0.f ||
        b4_.minBoxIoU > 1.f || b4_.maxCenterDistance <= 0.f ||
        b4_.dynamicEnterCount <= 0 || b4_.staticEnterCount <= 0 ||
        b4_.dynamicRecoveryCount <= 0 || b4_.trackTTL < 0 ||
        b4_.qualityMin < 0.f || b4_.qualityMin > 1.f)
        throw std::runtime_error("B4 association/state thresholds are invalid");
    showMask_ = Flag(fs,"Yolo.ShowMask");
    if (showMask_ && !enabled_)
        throw std::runtime_error("Yolo.ShowMask=1 requires Yolo.Enable=1");
    if (showMask_ && !std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY"))
        throw std::runtime_error("Yolo.ShowMask=1 requires a graphical desktop (DISPLAY/WAYLAND_DISPLAY); run from the Ubuntu desktop terminal");
    if (showMask_) MaskPreview::Reset();
    const auto legacy = fs["Yolo.DilateRadius"];
    bool postEnabled = Flag(fs,"MaskPostProcess.Enable");
    int radius = Integer(fs,"MaskPostProcess.DilateRadius",3);
    if (!legacy.empty()) {
        if (!fs["MaskPostProcess.Enable"].empty() || !fs["MaskPostProcess.DilateRadius"].empty())
            throw std::runtime_error("Configuration conflict: remove legacy Yolo.DilateRadius when using MaskPostProcess.*");
        radius = Integer(fs,"Yolo.DilateRadius",3);
        postEnabled = enabled_ && radius > 0;
        std::cerr << "[CONFIG migration] Yolo.DilateRadius -> MaskPostProcess.DilateRadius="
                  << radius << ", Enable=" << postEnabled << std::endl;
    }
    if (postEnabled && !enabled_)
        throw std::runtime_error("MaskPostProcess.Enable=1 requires Yolo.Enable=1");
    post_ = MaskPostProcessor(postEnabled,radius);
    std::cout << "[MODULES] Yolo.Enable=" << enabled_
              << " MaskPostProcess.Enable=" << postEnabled << " DilateRadius=" << radius
              << " Yolo.ShowMask=" << showMask_
              << " MaskFeatureReject.Enable=" << maskFeatureRejectEnabled_ << std::endl;
    std::cout << "[B2 CONFIG]" << std::endl
              << "Enable=" << b2_.enable << std::endl
              << "EvidenceOnly=" << b2_.evidenceOnly << std::endl
              << "Candidate.Enable=" << b2_.candidateEnable
              << " Candidate.MaxFeaturesPerInstance=" << b2_.maxFeaturesPerInstance
              << " Candidate.ErodeRadius=" << b2_.candidateErodeRadius
              << " Candidate.MinBoundaryDistance=" << b2_.minBoundaryDistance << std::endl
              << "Association.Enable=" << b2_.associationEnable
              << " Association.UseMutualMatch=" << b2_.useMutualMatch
              << " Association.LoweRatio=" << b2_.loweRatio
              << " Association.MaxHammingDistance=" << b2_.maxHammingDistance
              << " Association.MinInstanceIoU=" << b2_.minInstanceIoU
              << " Association.MaxCenterDistancePixels=" << b2_.maxInstanceCenterDistancePixels
              << " Association.MinInstanceScore=" << b2_.minInstanceAssociationScore << std::endl
              << "Geometry.Enable=" << b2_.geometryEnable
              << " Geometry.UseDepthMedian3x3=" << b2_.useDepthMedian3x3
              << " Geometry.MinDepthMeters=" << b2_.minDepthMeters
              << " Geometry.MaxDepthMeters=" << b2_.maxDepthMeters
              << " Geometry.UseNormalizedResidual=" << b2_.useNormalizedResidual
              << " Geometry.MinMatchedFeatures=" << b2_.minMatchedFeatures
              << " Geometry.MinMatchRatio=" << b2_.minMatchRatio
              << " Geometry.MinValidFeatures=" << b2_.minValidFeatures
              << " Geometry.MinValidMatchRatio=" << b2_.minValidMatchRatio
              << " Geometry.BackgroundExclusionDilateRadius=" << b2_.backgroundExclusionDilateRadius << std::endl
              << "Geometry.Decision.Enable=" << b2_.geometryDecisionEnable
              << " Geometry.Decision.Method=" << b2_.geometryDecisionMethod
              << " Geometry.Decision.DepthBinMeters=" << b2_.depthBinMeters
              << " Geometry.Decision.MinSamplesPerBin=" << b2_.minSamplesPerBin
              << " Geometry.Decision.MinBackgroundSamples=" << b2_.minBackgroundSamples
              << " Geometry.Decision.SigmaFloorMeters=" << b2_.sigmaFloorMeters
              << " Geometry.RawMovingThreshold=" << b2_.rawMovingThreshold
              << " Geometry.NormMovingThreshold=" << b2_.normMovingThreshold
              << " Geometry.HighRatioMovingThreshold=" << b2_.highRatioMovingThreshold
              << " Geometry.RawStaticThreshold=" << b2_.rawStaticThreshold
              << " Geometry.NormStaticThreshold=" << b2_.normStaticThreshold
              << " Geometry.HighRatioStaticThreshold=" << b2_.highRatioStaticThreshold
              << " HighRatio.PointNormCutoff=" << b2_.highResidualNormThreshold << std::endl
              << "Debug.Enable=" << b2_.debugEnable
              << " Debug.SaveCsv=" << b2_.saveCsv
              << " Debug.SaveFeatureCsv=" << b2_.saveFeatureCsv
              << " Debug.SaveSummaryCsv=" << b2_.saveSummaryCsv
              << " Debug.VerbosePerFeature=" << b2_.verbosePerFeature
              << " Debug.PrintFrameSummary=" << b2_.printFrameSummary
              << " Debug.ShowOverlay=" << b2_.showOverlay << std::endl;
    std::cout << "[B3 CONFIG]" << std::endl
              << "Enable=" << b3_.enable << std::endl
              << "EvidenceOnly=" << b3_.evidenceOnly << std::endl
              << "Semantic.PersonAsDynamicPrior=" << b3_.personAsDynamicPrior << std::endl
              << "Debug.PrintFrameSummary=" << b3_.printFrameSummary
              << " Debug.SaveSummaryCsv=" << b3_.saveSummaryCsv
              << " Debug.ShowOverlay=" << b3_.showOverlay << std::endl;
    std::cout << "[B4 EVIDENCE EXPIRY CONFIG] Enable=" << b4_.evidenceExpiryEnable
              << " MaxGapSeconds=" << b4_.maxEvidenceGapSeconds
              << " MaxStaticAgeSeconds=" << b4_.maxStaticAgeSeconds
              << " clock=image_acquisition expiry=strict_greater_than" << std::endl;
    std::cout << "[B4 CONFIG]" << std::endl
              << "Enable=" << b4_.enable << std::endl
              << "EvidenceOnly=" << b4_.evidenceOnly << std::endl
              << "DynamicEnterCount=" << b4_.dynamicEnterCount
              << " StaticEnterCount=" << b4_.staticEnterCount
              << " DynamicRecoveryCount=" << b4_.dynamicRecoveryCount
              << " TrackTTL=" << b4_.trackTTL << std::endl
              << "UseContinuousQuality=" << b4_.useContinuousQuality
              << " QualityMin=" << b4_.qualityMin << std::endl
              << "Track.MinMaskIoU=" << b4_.minMaskIoU
              << " Track.MinBoxIoU=" << b4_.minBoxIoU
              << " Track.MaxCenterDistance=" << b4_.maxCenterDistance
              << " (normalized image diagonal; mask-IoU deferred)" << std::endl
              << "Pending.UseForPose=" << b4_.pendingUseForPose
              << " Pending.AllowMapCreation=" << b4_.pendingAllowMapCreation
              << " Dynamic.AllowMapCreation=" << b4_.dynamicAllowMapCreation << std::endl
              << "Debug.PrintFrameSummary=" << b4_.printFrameSummary
              << " Debug.SaveSummaryCsv=" << b4_.saveSummaryCsv
              << " Debug.ShowOverlay=" << b4_.showOverlay << std::endl;
    if (!enabled_) return; // No model path checks, session, conversion, or masks.
    YOLOSegOptions options;
    if (!fs["Yolo.Confidence"].empty()) options.confidence = float(fs["Yolo.Confidence"]);
    if (!fs["Yolo.NmsIoU"].empty()) options.nmsIoU = float(fs["Yolo.NmsIoU"]);
    if (!fs["Yolo.MaskThreshold"].empty()) options.maskThreshold = float(fs["Yolo.MaskThreshold"]);
    options.intraOpThreads = Integer(fs,"Yolo.IntraOpThreads",2);
    auto classes = fs["Yolo.DynamicClasses"];
    if (!classes.empty()) {
        if (!classes.isSeq()) throw std::runtime_error("Yolo.DynamicClasses must be a sequence");
        options.dynamicClasses.clear();
        for (const auto& c : classes) options.dynamicClasses.push_back(int(c));
    }
    const std::string model = fs["Yolo.ModelPath"].empty() ? "" : std::string(fs["Yolo.ModelPath"]);
    struct stat st;
    if (model.empty() || ::stat(model.c_str(),&st) != 0 || !S_ISREG(st.st_mode))
        throw std::runtime_error("YOLO model does not exist: " + model);
    b2_.yoloModelPath = model;
    b2_.yoloModelSha256 = FileSha256(model);
    try { segmentor_.reset(new YOLOSeg(model,options)); }
    catch (const std::exception& e) { throw std::runtime_error("YOLO model load failed: " + std::string(e.what())); }
    saveMask_ = Flag(fs,"Yolo.SaveMask");
    saveOverlay_ = Flag(fs,"Yolo.SaveOverlay");
    logTiming_ = Flag(fs,"Yolo.LogTiming");
    if (saveMask_ || saveOverlay_) {
        const std::string parent = fs["Yolo.OutputDir"].empty() ? "." : std::string(fs["Yolo.OutputDir"]);
        const auto id = std::chrono::system_clock::now().time_since_epoch().count();
        outputDirectory_ = parent + "/masks_" + std::to_string(id) + "_" + std::to_string(getpid());
        if (::mkdir(outputDirectory_.c_str(),0755) != 0)
            throw std::runtime_error("Cannot exclusively create mask directory (parent must exist): " + outputDirectory_);
        index_.open(outputDirectory_ + "/index.csv");
        if (!index_) throw std::runtime_error("Cannot write mask index");
        index_ << "frame,timestamp,view,mask,overlay,inference_ms,postprocess_ms\n";
    }
}
DynamicDetector::InferenceOutput DynamicDetector::InferOne(
    const cv::Mat& image, bool rgb, const char* view, double timestamp) {
    if (!enabled_) return InferenceOutput{};
    const cv::Mat bgr = ToBGR(image,rgb);
    const auto start = std::chrono::steady_clock::now();
    YOLOSegResult raw;
    try { raw = segmentor_->inferInstances(bgr); }
    catch (const std::exception& e) { throw std::runtime_error(std::string("YOLO inference failed [") + view + "]: " + e.what()); }
    ++inferenceCalls_;
    const auto inferred = std::chrono::steady_clock::now();
    if (raw.personMask.empty() || raw.personMask.type()!=CV_8UC1 || raw.personMask.size()!=image.size())
        throw std::runtime_error("YOLO mask type/size mismatch");
    cv::Mat mask = raw.personMask;
    // B1 dilates only the union used to reject main ORB candidates.  The
    // instance masks returned below deliberately remain raw for B2.
    if (post_.Enabled()) { mask = post_.Apply(raw.personMask); ++postCalls_; }
    const auto processed = std::chrono::steady_clock::now();
    const double inferMs = Milliseconds(start,inferred);
    const double postMs = post_.Enabled() ? Milliseconds(inferred,processed) : 0.0;
    lastInferenceMs_ += inferMs; lastPostMs_ += postMs;
    totalInferenceMs_ += inferMs; totalPostMs_ += postMs;
    if (logTiming_)
        std::cout << "[YOLO] frame=" << sequence_ << " view=" << view << " inference_ms=" << inferMs
                  << " postprocess_ms=" << postMs << " excluded_pixels=" << cv::countNonZero(mask) << std::endl;
    if (saveMask_ || saveOverlay_) {
        std::ostringstream stem; stem << std::setw(6) << std::setfill('0') << sequence_ << '_' << view;
        const std::string maskName = saveMask_ ? stem.str()+"_mask.png" : "";
        const std::string overlayName = saveOverlay_ ? stem.str()+"_overlay.png" : "";
        if (saveMask_ && !cv::imwrite(outputDirectory_+'/'+maskName,mask))
            throw std::runtime_error("Cannot write mask");
        if (saveOverlay_) {
            cv::Mat colored=bgr.clone(),overlay; colored.setTo(cv::Scalar(0,0,255),mask);
            cv::addWeighted(bgr,.65,colored,.35,0,overlay);
            if (!cv::imwrite(outputDirectory_+'/'+overlayName,overlay)) throw std::runtime_error("Cannot write overlay");
        }
        index_ << sequence_ << ',' << std::setprecision(17) << timestamp << ',' << view << ','
               << maskName << ',' << overlayName << ',' << inferMs << ',' << postMs << '\n';
        index_.flush();
        if (!index_) throw std::runtime_error("Cannot write mask index");
    }
    if (showMask_) MaskPreview::Show(bgr,mask,view);
    InferenceOutput output;
    output.b1Mask = mask;
    output.instances = std::move(raw.instances);
    return output;
}
cv::Mat DynamicDetector::Detect(const cv::Mat& image, bool rgb, double timestamp) {
    lastInferenceMs_ = lastPostMs_ = 0;
    lastRGBDInstances_.clear();
    if (!enabled_) return cv::Mat();
    InferenceOutput output = InferOne(image,rgb,"rgbd",timestamp);
    lastRGBDInstances_ = std::move(output.instances);
    ++sequence_;
    return output.b1Mask;
}
void DynamicDetector::WriteSummary(const std::string& path) const {
    cv::FileStorage out(path,cv::FileStorage::WRITE);
    if (!out.isOpened()) throw std::runtime_error("Cannot save module timing: " + path);
    out << "yolo_enabled" << int(enabled_) << "postprocess_enabled" << int(post_.Enabled())
        << "mask_feature_reject_enabled" << int(maskFeatureRejectEnabled_)
        << "b2_enabled" << int(b2_.enable) << "b2_evidence_only" << int(b2_.evidenceOnly)
        << "show_mask" << int(showMask_)
        << "dilate_radius" << post_.Radius() << "inference_calls" << double(inferenceCalls_)
        << "postprocess_calls" << double(postCalls_) << "inference_total_ms" << totalInferenceMs_
        << "postprocess_total_ms" << totalPostMs_;
}
StereoMasks DynamicDetector::DetectStereo(const cv::Mat& left, const cv::Mat& right, bool rgb, double timestamp) {
    lastInferenceMs_ = lastPostMs_ = 0;
    lastLeftInstances_.clear();
    lastRightInstances_.clear();
    if (!enabled_) return StereoMasks{};
    StereoMasks masks;
    InferenceOutput leftOutput = InferOne(left,rgb,"left",timestamp);
    InferenceOutput rightOutput = InferOne(right,rgb,"right",timestamp);
    masks.left = leftOutput.b1Mask;
    masks.right = rightOutput.b1Mask;
    lastLeftInstances_ = std::move(leftOutput.instances);
    lastRightInstances_ = std::move(rightOutput.instances);
    ++sequence_;
    return masks;
}
}
