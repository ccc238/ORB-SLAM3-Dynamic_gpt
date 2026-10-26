#include "Deep/MaskPreview.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <map>
#include <mutex>
#include <set>
#include <string>

namespace ORB_SLAM3 {
namespace MaskPreview {
namespace {
std::atomic<bool> stopRequested{false};
std::atomic<bool> viewerMode{false};
std::mutex pendingMutex;
struct PreviewFrame { cv::Mat image, mask; };
// Bounded latest-frame mailbox: slow display never queues every input frame.
std::map<std::string,PreviewFrame> pending;
// Accessed only by the display owner (Viewer OR Tracking, never both).
std::set<std::string> windows;
void Render(const cv::Mat& bgr, const cv::Mat& mask, const std::string& view) {
    cv::Mat colored=bgr.clone(),overlay;
    colored.setTo(cv::Scalar(0,0,255),mask);
    cv::addWeighted(bgr,.65,colored,.35,0,overlay);
    cv::putText(overlay,"Person excluded (red) | Q / Esc: stop",
                cv::Point(10,24),cv::FONT_HERSHEY_SIMPLEX,.55,cv::Scalar(0,255,255),1,cv::LINE_AA);
    const std::string prefix=std::string("YOLO person - ")+view;
    const std::string maskName=prefix+" - mask (white=exclude)";
    const std::string overlayName=prefix+" - overlay";
    for (const auto& name : {maskName,overlayName}) {
        if (windows.insert(name).second) {
            cv::namedWindow(name,cv::WINDOW_NORMAL);
            cv::resizeWindow(name,bgr.cols,bgr.rows);
        }
    }
    cv::imshow(maskName,mask);
    cv::imshow(overlayName,overlay);
}
}
void Reset() {
    stopRequested.store(false);
    std::lock_guard<std::mutex> lock(pendingMutex);
    pending.clear();
}
void SetViewerMode(bool enabled) { viewerMode.store(enabled); }
bool ViewerMode() { return viewerMode.load(); }
void RequestStop() { stopRequested.store(true); }
bool StopRequested() { return stopRequested.load(); }
void HandleKey(int key) {
    key &= 0xff;
    if (key==27 || key=='q' || key=='Q') RequestStop();
}
void Show(const cv::Mat& bgr, const cv::Mat& mask, const char* view) {
    CV_Assert(bgr.type()==CV_8UC3 && mask.type()==CV_8UC1 && bgr.size()==mask.size());
    if (ViewerMode()) {
        std::lock_guard<std::mutex> lock(pendingMutex);
        pending[view]=PreviewFrame{bgr.clone(),mask.clone()};
        return; // No HighGUI calls from Tracking when Viewer owns the display.
    }
    Render(bgr,mask,view);
    HandleKey(cv::waitKey(1));
}
void RenderPending() {
    std::map<std::string,PreviewFrame> latest;
    {
        std::lock_guard<std::mutex> lock(pendingMutex);
        latest.swap(pending);
    }
    for (const auto& entry : latest)
        Render(entry.second.image,entry.second.mask,entry.first);
}
void Close() {
    for (const auto& name : windows) {
        try { cv::destroyWindow(name); } catch(const cv::Exception&) { /* already closed */ }
    }
    if (!windows.empty()) cv::waitKey(1);
    windows.clear();
}
}
}
