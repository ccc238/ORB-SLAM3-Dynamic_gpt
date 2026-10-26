#ifndef ORB_SLAM3_DYNAMIC_DETECTOR_H
#define ORB_SLAM3_DYNAMIC_DETECTOR_H
#include "YOLOSeg.h"
#include "Deep/B4Config.h"
#include "Deep/DynamicFrontendConfig.h"
#include "Deep/MaskPostProcessor.h"
#include <fstream>
#include <limits>
namespace ORB_SLAM3 {
struct StereoMasks { cv::Mat left, right; };

// B3 is the future semantic--geometric fusion stage.  Keeping its settings
// alongside YOLO/B1/B2 gives every front-end semantic module one YAML parsing
// and validation path.  B3 remains evidence-only until a later feature-policy
// stage consumes its proposed actions.
struct B3Config {
    bool enable = false;
    bool evidenceOnly = true;
    bool personAsDynamicPrior = true;

    bool printFrameSummary = true;
    bool saveSummaryCsv = true;
    bool showOverlay = true;
};

// B2 deliberately contains configuration only at this stage. Evidence-only
// mode must not influence tracking or pose estimation.
struct B2Config {
    bool enable = false;
    bool evidenceOnly = true;

    bool candidateEnable = true;
    int maxFeaturesPerInstance = 300;
    int candidateErodeRadius = 3;
    int minBoundaryDistance = 3;

    bool associationEnable = true;
    bool useMutualMatch = true;
    float loweRatio = 0.75f;
    int maxHammingDistance = 64;
    float minInstanceIoU = 0.10f;
    float maxInstanceCenterDistancePixels = 100.0f;
    float minInstanceAssociationScore = 0.30f;

    bool geometryEnable = true;
    bool useDepthMedian3x3 = true;
    float minDepthMeters = 0.30f;
    float maxDepthMeters = 5.00f;
    bool useNormalizedResidual = true;
    // Evidence sufficiency is intentionally split by denominator.  A feature
    // correspondence is "matched" after descriptor/mutual filtering and
    // before its depth/geometry check; "valid" means a usable 3-D residual.
    int minMatchedFeatures = 30;
    float minMatchRatio = 0.15f;
    int minValidFeatures = 10;
    float minValidMatchRatio = 0.40f;
    int backgroundExclusionDilateRadius = 3;
    bool geometryDecisionEnable = false;
    std::string geometryDecisionMethod = "background_mad";
    float depthBinMeters = 0.50f;
    int minSamplesPerBin = 30;
    int minBackgroundSamples = 50;
    float sigmaFloorMeters = 0.01f;
    // Three-zone decision thresholds.  New YAML keys live directly below
    // B2.Geometry so moving/static gates are independently configurable.
    // Infinity is the legacy fallback for rawStaticThreshold: the old static
    // rule had no raw-residual upper bound, so this preserves old YAML runs.
    float rawMovingThreshold = 0.02f;
    float normMovingThreshold = 3.5f;
    float highRatioMovingThreshold = 0.60f;
    float rawStaticThreshold = std::numeric_limits<float>::infinity();
    float normStaticThreshold = 2.0f;
    float highRatioStaticThreshold = 0.20f;
    // Point-level cutoff defining P=count(e_i>=cutoff)/valid_count. Keep it
    // separate from the instance median threshold to preserve offline CSV P.
    float highResidualNormThreshold = 3.5f;

    bool debugEnable = true;
    // SaveCsv is the backward-compatible master switch.  The two following
    // switches select feature-level and per-instance summary streams.
    bool saveCsv = true;
    bool saveFeatureCsv = true;
    bool saveSummaryCsv = true;
    bool verbosePerFeature = false;
    bool printFrameSummary = true;
    bool showOverlay = true;
    std::string csvOutputParent = ".";
    std::string datasetName = "unknown";
    std::string yamlPath;
    std::string yamlSha256;
    std::string yoloModelPath;
    std::string yoloModelSha256;
};

class DynamicDetector {
public:
    explicit DynamicDetector(const std::string& settingsPath);
    // Accept the images used by Frame, before grayscale conversion. Disabled
    // paths return empty matrices before accessing/converting input images.
    cv::Mat Detect(const cv::Mat& image, bool inputIsRGB, double timestamp);
    StereoMasks DetectStereo(const cv::Mat& left, const cv::Mat& right,
                             bool inputIsRGB, double timestamp);
    // Raw, per-person outputs of the latest inference.  Each mask is in the
    // source image coordinate system and has not been B1-dilated.
    const std::vector<YOLOSegInstance>& LastRGBDInstances() const { return lastRGBDInstances_; }
    std::vector<YOLOSegInstance>& MutableLastRGBDInstances() { return lastRGBDInstances_; }
    const std::vector<YOLOSegInstance>& LastLeftInstances() const { return lastLeftInstances_; }
    const std::vector<YOLOSegInstance>& LastRightInstances() const { return lastRightInstances_; }
    bool Enabled() const { return enabled_; }
    bool MaskFeatureRejectEnabled() const { return maskFeatureRejectEnabled_; }
    const DynamicFrontendConfig& Frontend() const { return frontend_; }
    const B2Config& B2() const { return b2_; }
    const B3Config& B3() const { return b3_; }
    const B4Config& B4() const { return b4_; }
    bool PostProcessEnabled() const { return post_.Enabled(); }
    int PostProcessRadius() const { return post_.Radius(); }
    size_t InferenceCalls() const { return inferenceCalls_; }
    size_t PostProcessCalls() const { return postCalls_; }
    double LastInferenceMs() const { return lastInferenceMs_; }
    double LastPostProcessMs() const { return lastPostMs_; }
    void WriteSummary(const std::string& path) const;
private:
    struct InferenceOutput {
        cv::Mat b1Mask;
        std::vector<YOLOSegInstance> instances;
    };
    InferenceOutput InferOne(const cv::Mat& image, bool inputIsRGB,
                             const char* view, double timestamp);
    bool enabled_ = false, maskFeatureRejectEnabled_ = true;
    bool saveMask_ = false, saveOverlay_ = false, logTiming_ = false;
    bool showMask_ = false;
    B2Config b2_;
    B3Config b3_;
    B4Config b4_;
    DynamicFrontendConfig frontend_;
    MaskPostProcessor post_;
    std::unique_ptr<YOLOSeg> segmentor_;
    std::string outputDirectory_;
    std::ofstream index_;
    size_t sequence_ = 0, inferenceCalls_ = 0, postCalls_ = 0;
    double lastInferenceMs_ = 0, lastPostMs_ = 0;
    double totalInferenceMs_ = 0, totalPostMs_ = 0;
    std::vector<YOLOSegInstance> lastRGBDInstances_, lastLeftInstances_, lastRightInstances_;
};
}
#endif
