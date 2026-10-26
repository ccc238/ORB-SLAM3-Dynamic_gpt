#ifndef ORB_SLAM3_INSTANCE_TRACK_MANAGER_H
#define ORB_SLAM3_INSTANCE_TRACK_MANAGER_H

#include "Deep/B4Config.h"
#include "YOLOSeg.h"
#include <unordered_map>

namespace ORB_SLAM3 {

class InstanceTrackManager {
public:
    explicit InstanceTrackManager(const B4Config& config) : mConfig(config) {}
    void Associate(std::vector<YOLOSegInstance>& instances, const cv::Size& imageSize);

private:
    struct Track {
        int id = -1;
        int classId = -1;
        cv::Mat mask;
        cv::Rect boundingBox;
        cv::Point2f center;
        int missedFrames = 0;
    };
    static float MaskIoU(const cv::Mat& a, const cv::Mat& b);
    static float BoxIoU(const cv::Rect& a, const cv::Rect& b);
    static float CenterDistance(const cv::Point2f& a, const cv::Point2f& b,
                                const cv::Size& imageSize);
    B4Config mConfig;
    int mNextTrackId = 0;
    std::unordered_map<int, Track> mTracks;
};
}
#endif
