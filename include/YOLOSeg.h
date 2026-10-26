#ifndef ORB_SLAM3_YOLOSEG_H
#define ORB_SLAM3_YOLOSEG_H

#include <opencv2/core.hpp>
#include <onnxruntime_cxx_api.h>
#include <memory>
#include <string>
#include <vector>

namespace ORB_SLAM3 {

struct YOLOSegOptions {
    int inputWidth = 640;
    int inputHeight = 640;
    int numClasses = 80;
    int intraOpThreads = 2;
    int maxDetections = 100;
    float confidence = 0.25f;
    float nmsIoU = 0.45f;
    float maskThreshold = 0.50f;
    std::vector<int> dynamicClasses = {0};
};

// Original-image coordinates.  mask is CV_8UC1, 255 inside exactly this
// detected person instance; it is deliberately never post-processed here.
struct YOLOSegInstance {
    cv::Mat mask;
    cv::Rect boundingBox;
    float confidence = 0.0f;
    int classId = -1;
    // Assigned once by InstanceTrackManager immediately after inference.
    // All later B2/B3/B4 stages consume this shared identity.
    int trackId = -1;
};

struct YOLOSegResult {
    // Union of the unmodified instance masks (before any B1 dilation).
    cv::Mat personMask;
    std::vector<YOLOSegInstance> instances;
};

class YOLOSeg {
public:
    YOLOSeg(const std::string& modelPath,
            const YOLOSegOptions& options);

    // Input: unmodified CV_8UC3 BGR image.
    // Output: original-size CV_8UC1, 255=exclude, 0=keep.
    cv::Mat infer(const cv::Mat& bgr);

    // Same inference, retaining every selected person instance.  B1 uses
    // personMask; B2 must use instances, never a dilated B1 mask.
    YOLOSegResult inferInstances(const cv::Mat& bgr);

private:
    struct Letterbox {
        cv::Mat blob;
        int left;
        int top;
        int resizedWidth;
        int resizedHeight;
        float scaleX;
        float scaleY;
    };
    struct Detection {
        cv::Rect box;
        int classId;
        float confidence;
        std::vector<float> coefficients;
    };

    Letterbox preprocess(const cv::Mat& bgr) const;
    bool isDynamicClass(int classId) const;

    YOLOSegOptions options_;
    Ort::Env env_;
    Ort::SessionOptions sessionOptions_;
    std::unique_ptr<Ort::Session> session_;
    std::string inputName_;
    std::vector<std::string> outputNames_;
};

} // namespace ORB_SLAM3
#endif
