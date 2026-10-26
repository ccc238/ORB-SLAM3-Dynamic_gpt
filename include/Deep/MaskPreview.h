#ifndef ORB_SLAM3_MASK_PREVIEW_H
#define ORB_SLAM3_MASK_PREVIEW_H
#include <opencv2/core.hpp>
namespace ORB_SLAM3 {
// With Viewer enabled, Tracking only publishes images; the Viewer thread
// owns ALL HighGUI windows. Otherwise the Tracking caller owns the preview.
namespace MaskPreview {
void Reset();
void SetViewerMode(bool enabled);
bool ViewerMode();
void Show(const cv::Mat& bgr, const cv::Mat& mask, const char* view);
void RenderPending();
void HandleKey(int key);
void RequestStop();
bool StopRequested();
void Close();
}
}
#endif
