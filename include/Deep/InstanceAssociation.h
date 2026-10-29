#ifndef ORB_SLAM3_INSTANCE_ASSOCIATION_H
#define ORB_SLAM3_INSTANCE_ASSOCIATION_H
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
namespace ORB_SLAM3 {
enum class AssociationStatus { NEW_TRACK, CONTINUOUS_CONFIRMED, REACQUIRED, AMBIGUOUS };
inline const char* AssociationStatusName(AssociationStatus s) {
    switch(s) {
    case AssociationStatus::CONTINUOUS_CONFIRMED:return "CONTINUOUS_CONFIRMED";
    case AssociationStatus::REACQUIRED:return "REACQUIRED";
    case AssociationStatus::AMBIGUOUS:return "AMBIGUOUS";
    default:return "NEW_TRACK";
    }
}
struct InstanceAssociationResult {
    int instanceId=-1, trackId=-1;
    long previousMatchedFrame=-1;
    double previousMatchedTimestamp=std::numeric_limits<double>::quiet_NaN();
    AssociationStatus associationStatus=AssociationStatus::NEW_TRACK;
    double associationCost=std::numeric_limits<double>::quiet_NaN();
    double rowMargin=std::numeric_limits<double>::quiet_NaN();
    double columnMargin=std::numeric_limits<double>::quiet_NaN();
    bool evaluated=false, allowStateInheritance=false, allowGeometryPair=false, resetEvidence=false;
    std::string reason="NOT_ASSOCIATED", prediction="NONE", stateResetReason="NONE";
    int flowSupport=0;
    bool depthAvailable=false;
};
// Initial engineering values, NOT accuracy-validated thresholds. Mode owns
// the enabled features to make A0..A3 differences explicit and reproducible.
struct InstanceAssociationConfig {
    int mode=3; // 0 legacy greedy; 1 global; 2 prediction/ambiguity; 3 controlled inheritance
    double maxCost=.65, unmatchedTrackCost=.40, unmatchedInstanceCost=.40;
    double minMargin=.08, maxCenterDistance=1.0, maxScaleRatio=4.0;
    double minMaskIoU=.05, minBoxIoU=.10, maxPredictionSeconds=.75;
    double maxDepthDifference=.60, maxDepthRelative=.30, maxDepthMAD=.20;
    double maskWeight=.40, boxWeight=.20, centerWeight=.25, flowWeight=.15, depthWeight=.15;
    double mergeCoverage=.65, flowFBError=1.5, flowQuality=.01;
    int minEvidenceTerms=2, flowMaxCorners=80, flowMinSupport=8, safeErodeRadius=3, minDepthSamples=20;
    bool saveCsv=true, printSummary=true;
};
inline void ValidateInstanceAssociation(const InstanceAssociationConfig& c) {
    if(c.mode<0 || c.mode>3) throw std::runtime_error("InstanceAssociation.Mode must be in [0,3]");
    for(double v:{c.maxCost,c.unmatchedTrackCost,c.unmatchedInstanceCost,c.minMargin,c.maxCenterDistance,
        c.maxScaleRatio,c.maxPredictionSeconds,c.maxDepthDifference,c.maxDepthRelative,c.maxDepthMAD,
        c.mergeCoverage,c.flowFBError,c.flowQuality})
        if(!std::isfinite(v)||v<=0) throw std::runtime_error("InstanceAssociation thresholds must be finite and positive");
    for(double v:{c.maskWeight,c.boxWeight,c.centerWeight,c.flowWeight,c.depthWeight,c.minMaskIoU,c.minBoxIoU})
        if(!std::isfinite(v)||v<0) throw std::runtime_error("InstanceAssociation weights/IoU must be finite and nonnegative");
    if(c.maxCost>=c.unmatchedTrackCost+c.unmatchedInstanceCost || c.maxCost>1 ||
       c.maxScaleRatio<1 || c.minMargin>1 || c.mergeCoverage>1 || c.flowQuality>=1 ||
       c.minMaskIoU>1 || c.minBoxIoU>1 || c.minEvidenceTerms<2 || c.minEvidenceTerms>3 ||
       c.safeErodeRadius<0 || c.safeErodeRadius>64 || c.flowMinSupport<3 ||
       c.flowMaxCorners<c.flowMinSupport || c.flowMaxCorners>1000 || c.minDepthSamples<1 ||
       c.maskWeight+c.boxWeight+c.centerWeight<=0)
        throw std::runtime_error("InstanceAssociation invalid costs, weights, prediction or evidence limits");
}
struct InstanceAssociationEvent { int trackId=-1; std::string type,reason; };
}
#endif
