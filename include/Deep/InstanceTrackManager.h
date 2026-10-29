#ifndef ORB_SLAM3_INSTANCE_TRACK_MANAGER_H
#define ORB_SLAM3_INSTANCE_TRACK_MANAGER_H

#include "Deep/B4Config.h"
#include "YOLOSeg.h"
#include <unordered_map>

namespace ORB_SLAM3 {

class InstanceTrackManager {
public:
    // Compatibility entry for old synthetic/size-only callers. Production
    // Tracking always supplies the explicitly parsed association config.
    explicit InstanceTrackManager(const B4Config& config) : mConfig(config) { mAssociation.mode=0; }
    InstanceTrackManager(const B4Config& config,
        const InstanceAssociationConfig& association) : mConfig(config), mAssociation(association) {
        ValidateInstanceAssociation(association);
    }
    void Associate(std::vector<YOLOSegInstance>& instances, const cv::Size& imageSize);
    void Associate(std::vector<YOLOSegInstance>& instances, const cv::Size& imageSize,
        long frameId,double timestamp,const cv::Mat& gray=cv::Mat(),const cv::Mat& depthMeters=cv::Mat());
    void ValidateInputTime(long frameId,double timestamp) const;
    void Reset(); // IDs never reused within this manager session.
    const std::vector<InstanceAssociationEvent>& Events() const {return mEvents;}
    double ProcessingMs() const {return mProcessingMs;}
    int SequenceId() const {return mSequenceId;}
    static std::vector<int> SolveAssignment(const std::vector<std::vector<double>>& squareCosts);

private:
    struct Track {
        int id = -1;
        int classId = -1;
        cv::Mat mask;
        cv::Rect boundingBox;
        cv::Point2f center;
        int missedFrames = 0;
        long lastMatchedFrame=-1;
        double lastMatchedTimestamp=0;
        cv::Point2f velocity;
        cv::Mat gray;
        float depth=-1;
        bool ambiguousTemplate=false;
    };
    static float MaskIoU(const cv::Mat& a, const cv::Mat& b);
    static float BoxIoU(const cv::Rect& a, const cv::Rect& b);
    static float CenterDistance(const cv::Point2f& a, const cv::Point2f& b,
                                const cv::Size& imageSize);
    B4Config mConfig;
    InstanceAssociationConfig mAssociation;
    std::vector<InstanceAssociationEvent> mEvents;
    double mProcessingMs=0, mLastTimestamp=0;
    long mLastFrame=-1, mLegacyFrame=0;
    int mSequenceId=0;
    bool mHasInput=false;
    void AssociateLegacy(std::vector<YOLOSegInstance>& instances,const cv::Size&,long,double);
    int mNextTrackId = 0;
    std::unordered_map<int, Track> mTracks;
};
}
#endif
