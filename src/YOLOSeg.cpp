#include "YOLOSeg.h"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <utility>

namespace ORB_SLAM3 {

YOLOSeg::YOLOSeg(const std::string& modelPath,
                 const YOLOSegOptions& options)
    : options_(options), env_(ORT_LOGGING_LEVEL_WARNING, "YOLO11Seg")
{
    if (options_.inputWidth <= 0 || options_.inputHeight <= 0 ||
        options_.numClasses <= 0 || options_.intraOpThreads <= 0 ||
        options_.maxDetections <= 0 ||
        !(options_.confidence > 0 && options_.confidence < 1) ||
        !(options_.nmsIoU > 0 && options_.nmsIoU < 1) ||
        !(options_.maskThreshold > 0 && options_.maskThreshold < 1))
        throw std::runtime_error("Invalid YOLO options");
    for (int id : options_.dynamicClasses)
        if (id < 0 || id >= options_.numClasses)
            throw std::runtime_error("YOLO dynamic class id out of range");

    sessionOptions_.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);
    sessionOptions_.SetIntraOpNumThreads(options_.intraOpThreads);
    // CPU baseline. Installing a GPU package alone does not enable CUDA.
    session_.reset(new Ort::Session(
        env_, modelPath.c_str(), sessionOptions_));
    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 2)
        throw std::runtime_error(
            "Expected one input and two raw segmentation outputs; export nms=False");

    const auto inputInfo = session_->GetInputTypeInfo(0);
    const auto tensorInfo = inputInfo.GetTensorTypeAndShapeInfo();
    const auto shape = tensorInfo.GetShape();
    const std::vector<int64_t> expected = {
        1, 3, options_.inputHeight, options_.inputWidth};
    if (tensorInfo.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        shape != expected)
        throw std::runtime_error(
            "Expected fixed batch=1 FP32 NCHW input matching configured image size");

    Ort::AllocatorWithDefaultOptions allocator;
    auto inputName = session_->GetInputNameAllocated(0, allocator);
    inputName_ = inputName.get();
    for (size_t i = 0; i < session_->GetOutputCount(); ++i) {
        auto name = session_->GetOutputNameAllocated(i, allocator);
        outputNames_.emplace_back(name.get());
    }

    int predictions = 0, prototypes = 0, maskChannels = 0;
    std::vector<int64_t> detectionShape;
    for (size_t i = 0; i < 2; ++i) {
        const auto type = session_->GetOutputTypeInfo(i);
        const auto tensor = type.GetTensorTypeAndShapeInfo();
        const auto dims = tensor.GetShape();
        if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            throw std::runtime_error("YOLO outputs must be FP32");
        std::cout << "[YOLO] output " << outputNames_[i] << " [";
        for (auto dim : dims) std::cout << dim << ' ';
        std::cout << "]" << std::endl;
        if (dims.size() == 3 && dims[0] == 1) {
            ++predictions; detectionShape = dims;
        } else if (dims.size() == 4 && dims[0] == 1 &&
                   dims[1] > 0 && dims[2] > 0 && dims[3] > 0) {
            ++prototypes; maskChannels = static_cast<int>(dims[1]);
        }
    }
    const int channels = 4 + options_.numClasses + maskChannels;
    if (predictions != 1 || prototypes != 1 ||
        (detectionShape[1] != channels && detectionShape[2] != channels))
        throw std::runtime_error("Unsupported YOLO raw segmentation output layout/class count");
    std::cout << "[YOLO] input " << inputName_ << " [1 3 "
              << options_.inputHeight << ' ' << options_.inputWidth
              << "] FP32 ORT=" << OrtGetApiBase()->GetVersionString() << std::endl;
    std::cout << "[YOLO] loaded " << modelPath << " using CPU" << std::endl;
}

YOLOSeg::Letterbox YOLOSeg::preprocess(const cv::Mat& bgr) const
{
    if (bgr.empty() || bgr.type() != CV_8UC3)
        throw std::runtime_error("YOLO input must be nonempty CV_8UC3 BGR");
    const float ratio = std::min(
        options_.inputWidth / static_cast<float>(bgr.cols),
        options_.inputHeight / static_cast<float>(bgr.rows));
    Letterbox result;
    result.resizedWidth = std::min(options_.inputWidth,
        std::max(1, cvRound(bgr.cols * ratio)));
    result.resizedHeight = std::min(options_.inputHeight,
        std::max(1, cvRound(bgr.rows * ratio)));
    result.left = (options_.inputWidth - result.resizedWidth) / 2;
    result.top = (options_.inputHeight - result.resizedHeight) / 2;
    result.scaleX = result.resizedWidth / static_cast<float>(bgr.cols);
    result.scaleY = result.resizedHeight / static_cast<float>(bgr.rows);
    cv::Mat resized, padded;
    cv::resize(bgr, resized,
        cv::Size(result.resizedWidth, result.resizedHeight));
    cv::copyMakeBorder(resized, padded, result.top,
        options_.inputHeight - result.resizedHeight - result.top,
        result.left, options_.inputWidth - result.resizedWidth - result.left,
        cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    // BGR -> RGB, HWC -> NCHW, uint8 -> float32 / 255.
    result.blob = cv::dnn::blobFromImage(
        padded, 1.0 / 255.0,
        cv::Size(options_.inputWidth, options_.inputHeight),
        cv::Scalar(), true, false, CV_32F);
    return result;
}

bool YOLOSeg::isDynamicClass(int classId) const
{
    return std::find(options_.dynamicClasses.begin(),
                     options_.dynamicClasses.end(), classId)
           != options_.dynamicClasses.end();
}

cv::Mat YOLOSeg::infer(const cv::Mat& bgr)
{
    return inferInstances(bgr).personMask;
}

YOLOSegResult YOLOSeg::inferInstances(const cv::Mat& bgr)
{
    const Letterbox prep = preprocess(bgr);
    const std::vector<int64_t> inputShape = {
        1, 3, options_.inputHeight, options_.inputWidth};
    auto memory = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);
    auto input = Ort::Value::CreateTensor<float>(
        memory, const_cast<float*>(prep.blob.ptr<float>()),
        prep.blob.total(), inputShape.data(), inputShape.size());
    const char* inputNames[] = {inputName_.c_str()};
    const char* outputNames[] = {
        outputNames_[0].c_str(), outputNames_[1].c_str()};
    auto outputs = session_->Run(Ort::RunOptions{nullptr},
        inputNames, &input, 1, outputNames, 2);

    int predictionIndex = -1, prototypeIndex = -1;
    std::vector<int64_t> predictionShape, prototypeShape;
    for (int i = 0; i < 2; ++i) {
        if (!outputs[i].IsTensor())
            throw std::runtime_error("YOLO output is not a tensor");
        const auto info = outputs[i].GetTensorTypeAndShapeInfo();
        if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            throw std::runtime_error("YOLO output must be FP32");
        const auto shape = info.GetShape();
        if (shape.size() == 3) {
            predictionIndex = i;
            predictionShape = shape;
        } else if (shape.size() == 4) {
            prototypeIndex = i;
            prototypeShape = shape;
        }
    }
    if (predictionIndex < 0 || prototypeIndex < 0 ||
        predictionShape[0] != 1 || prototypeShape[0] != 1)
        throw std::runtime_error("Unexpected YOLO segmentation output layout");
    const int nm = static_cast<int>(prototypeShape[1]);
    const int ph = static_cast<int>(prototypeShape[2]);
    const int pw = static_cast<int>(prototypeShape[3]);
    if (nm <= 0 || ph <= 0 || pw <= 0)
        throw std::runtime_error("Invalid YOLO prototype shape");
    const int channels = 4 + options_.numClasses + nm;
    const bool channelFirst = predictionShape[1] == channels;
    if (!channelFirst && predictionShape[2] != channels)
        throw std::runtime_error(
            "Unexpected detection channels; check class count and nms=False export");
    const int count = static_cast<int>(
        channelFirst ? predictionShape[2] : predictionShape[1]);
    if (count <= 0)
        throw std::runtime_error("Invalid YOLO prediction count");
    const float* predictions = outputs[predictionIndex].GetTensorData<float>();
    auto value = [&](int row, int channel) {
        return channelFirst ? predictions[channel * count + row]
                            : predictions[row * channels + channel];
    };

    std::vector<Detection> candidates;
    std::map<int, std::vector<int> > groups;
    const cv::Rect content(prep.left, prep.top,
                          prep.resizedWidth, prep.resizedHeight);
    for (int i = 0; i < count; ++i) {
        int classId = -1;
        float confidence = 0.0f;
        for (int c = 0; c < options_.numClasses; ++c) {
            const float score = value(i, 4 + c);
            if (score > confidence) {
                confidence = score;
                classId = c;
            }
        }
        if (confidence < options_.confidence || !isDynamicClass(classId))
            continue;
        const float cx = value(i, 0), cy = value(i, 1);
        const float w = value(i, 2), h = value(i, 3);
        if (!std::isfinite(cx) || !std::isfinite(cy) ||
            !std::isfinite(w) || !std::isfinite(h) || w <= 0 || h <= 0)
            continue;
        const int x1 = static_cast<int>(std::floor(std::max(
            0.0f, std::min(float(options_.inputWidth), cx - w / 2))));
        const int y1 = static_cast<int>(std::floor(std::max(
            0.0f, std::min(float(options_.inputHeight), cy - h / 2))));
        const int x2 = static_cast<int>(std::ceil(std::max(
            0.0f, std::min(float(options_.inputWidth), cx + w / 2))));
        const int y2 = static_cast<int>(std::ceil(std::max(
            0.0f, std::min(float(options_.inputHeight), cy + h / 2))));
        if (x2 <= x1 || y2 <= y1)
            continue;
        Detection detection;
        detection.box = cv::Rect(x1, y1, x2 - x1, y2 - y1) & content;
        if (detection.box.empty())
            continue;
        detection.classId = classId;
        detection.confidence = confidence;
        for (int m = 0; m < nm; ++m)
            detection.coefficients.push_back(
                value(i, 4 + options_.numClasses + m));
        groups[classId].push_back(static_cast<int>(candidates.size()));
        candidates.push_back(detection);
    }

    // Per-class NMS avoids suppressing different selected object classes.
    std::vector<int> kept;
    for (const auto& group : groups) {
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        for (int index : group.second) {
            boxes.push_back(candidates[index].box);
            scores.push_back(candidates[index].confidence);
        }
        std::vector<int> selected;
        cv::dnn::NMSBoxes(boxes, scores,
            options_.confidence, options_.nmsIoU, selected);
        for (int index : selected)
            kept.push_back(group.second[index]);
    }
    std::sort(kept.begin(), kept.end(), [&](int a, int b) {
        return candidates[a].confidence > candidates[b].confidence;
    });
    if (kept.size() > static_cast<size_t>(options_.maxDetections))
        kept.resize(options_.maxDetections);

    YOLOSegResult result;
    result.personMask = cv::Mat::zeros(bgr.size(), CV_8UC1);
    const float* prototypeData = outputs[prototypeIndex].GetTensorData<float>();
    cv::Mat prototypes(nm, ph * pw, CV_32F,
                       const_cast<float*>(prototypeData));
    const float logitThreshold = std::log(
        options_.maskThreshold / (1.0f - options_.maskThreshold));
    for (int index : kept) {
        const Detection& detection = candidates[index];
        cv::Mat coefficients(1, nm, CV_32F);
        for (int m = 0; m < nm; ++m)
            coefficients.at<float>(0, m) = detection.coefficients[m];
        cv::Mat flat = coefficients * prototypes;
        cv::Mat logits = flat.reshape(1, ph);
        cv::Mat networkMask, originalLogits;
        cv::resize(logits, networkMask,
            cv::Size(options_.inputWidth, options_.inputHeight),
            0, 0, cv::INTER_LINEAR);
        cv::resize(networkMask(content), originalLogits,
            bgr.size(), 0, 0, cv::INTER_LINEAR);
        cv::Mat binary;
        cv::compare(originalLogits, logitThreshold, binary, cv::CMP_GT);

        const int x1 = std::max(0, static_cast<int>(std::floor(
            (detection.box.x - prep.left) / prep.scaleX)));
        const int y1 = std::max(0, static_cast<int>(std::floor(
            (detection.box.y - prep.top) / prep.scaleY)));
        const int x2 = std::min(bgr.cols, static_cast<int>(std::ceil(
            (detection.box.x + detection.box.width - prep.left) / prep.scaleX)));
        const int y2 = std::min(bgr.rows, static_cast<int>(std::ceil(
            (detection.box.y + detection.box.height - prep.top) / prep.scaleY)));
        if (x2 <= x1 || y2 <= y1)
            continue;
        const cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
        cv::Mat instance = cv::Mat::zeros(bgr.size(), CV_8UC1);
        binary(roi).copyTo(instance(roi));
        YOLOSegInstance person;
        person.mask = instance;
        person.boundingBox = roi;
        person.confidence = detection.confidence;
        person.classId = detection.classId;
        result.instances.push_back(std::move(person));
        cv::bitwise_or(result.personMask, instance, result.personMask);
    }
    return result;
}

} // namespace ORB_SLAM3
