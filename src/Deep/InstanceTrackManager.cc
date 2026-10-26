#include "Deep/InstanceTrackManager.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace ORB_SLAM3 {
float InstanceTrackManager::MaskIoU(const cv::Mat& a, const cv::Mat& b) {
    if (a.empty() || b.empty() || a.type()!=CV_8UC1 || b.type()!=CV_8UC1 || a.size()!=b.size()) return 0.f;
    cv::Mat inter, uni; cv::bitwise_and(a,b,inter); cv::bitwise_or(a,b,uni);
    const int n=cv::countNonZero(uni); return n ? float(cv::countNonZero(inter))/n : 0.f;
}
float InstanceTrackManager::BoxIoU(const cv::Rect& a, const cv::Rect& b) {
    const int n=a.area()+b.area()-(a&b).area(); return n ? float((a&b).area())/n : 0.f;
}
float InstanceTrackManager::CenterDistance(const cv::Point2f& a, const cv::Point2f& b,
                                           const cv::Size& s) {
    const float dx=a.x-b.x, dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy)/std::max(1.f, std::sqrt(float(s.width*s.width+s.height*s.height)));
}
void InstanceTrackManager::Associate(std::vector<YOLOSegInstance>& instances, const cv::Size& imageSize) {
    for (auto& entry : mTracks) ++entry.second.missedFrames;
    struct Pair { int track; size_t instance; float score; };
    std::vector<Pair> candidates;
    for (const auto& entry : mTracks) for (size_t i=0; i<instances.size(); ++i) {
        const Track& t=entry.second; const YOLOSegInstance& in=instances[i];
        if (t.classId != in.classId) continue;
        const float mask=MaskIoU(t.mask,in.mask), box=BoxIoU(t.boundingBox,in.boundingBox);
        const cv::Point2f center(in.boundingBox.x+.5f*in.boundingBox.width,in.boundingBox.y+.5f*in.boundingBox.height);
        const float distance=CenterDistance(t.center,center,imageSize);
        if (!(mask>=mConfig.minMaskIoU || box>=mConfig.minBoxIoU || distance<=mConfig.maxCenterDistance)) continue;
        candidates.push_back({entry.first,i,.50f*mask+.25f*box+.25f*(1.f-std::min(1.f,distance))});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Pair& a,const Pair& b){return a.score>b.score;});
    std::vector<int> assignment(instances.size(),-1); std::unordered_set<int> used;
    for (const Pair& p:candidates) if (!used.count(p.track) && assignment[p.instance]<0) { assignment[p.instance]=p.track; used.insert(p.track); }
    for (size_t i=0;i<instances.size();++i) {
        int id=assignment[i]; if(id<0) { id=mNextTrackId++; mTracks[id].id=id; }
        YOLOSegInstance& in=instances[i]; Track& t=mTracks[id];
        in.trackId=id; t.classId=in.classId; t.mask=in.mask.clone(); t.boundingBox=in.boundingBox;
        t.center=cv::Point2f(in.boundingBox.x+.5f*in.boundingBox.width,in.boundingBox.y+.5f*in.boundingBox.height); t.missedFrames=0;
    }
    for(auto it=mTracks.begin();it!=mTracks.end();) {
        if(it->second.missedFrames>mConfig.trackTTL) it=mTracks.erase(it); else ++it;
    }
}
}
