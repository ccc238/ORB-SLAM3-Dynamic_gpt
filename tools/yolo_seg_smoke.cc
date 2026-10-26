#include "Deep/DynamicDetector.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc < 3) { std::cerr << "Usage: yolo_seg_smoke settings.yaml image [output_prefix] | --config-only settings.yaml\n"; return 1; }
    try {
        const bool configOnly = std::string(argv[1]) == "--config-only";
        ORB_SLAM3::DynamicDetector detector(argv[configOnly ? 2 : 1]);
        if (configOnly) {
            const auto& b2 = detector.B2();
            std::cout << "[B2 THRESHOLDS] RD=" << b2.rawMovingThreshold
                      << " ED=" << b2.normMovingThreshold << " PD=" << b2.highRatioMovingThreshold
                      << " RS=" << b2.rawStaticThreshold << " ES=" << b2.normStaticThreshold
                      << " PS=" << b2.highRatioStaticThreshold
                      << " point_cutoff=" << b2.highResidualNormThreshold << '\n';
            return 0;
        }
        const cv::Mat image = cv::imread(argv[2]);
        if (image.empty()) throw std::runtime_error("Cannot read image");
        const cv::Mat mask = detector.Detect(image,false,0.0);
        if (!detector.Enabled()) {
            if (!mask.empty() || detector.InferenceCalls()!=0) throw std::runtime_error("Disabled path did work");
            // Unsupported inputs must also be ignored by the disabled branch.
            if (!detector.Detect(cv::Mat(3,3,CV_64F),true,0).empty()) throw std::runtime_error("Disabled conversion");
            std::cout << "PASS disabled: empty mask, zero inference; invalid image type skipped\n";
            return 0;
        }
        if (mask.size()!=image.size() || mask.type()!=CV_8UC1) throw std::runtime_error("Invalid mask");
        cv::Mat invalid = (mask!=0) & (mask!=255);
        if (cv::countNonZero(invalid)) throw std::runtime_error("Nonbinary mask");
        const auto instances = detector.LastRGBDInstances();
        if (instances.empty()) throw std::runtime_error("Smoke image must produce person instances");
        cv::Mat rawUnion = cv::Mat::zeros(image.size(),CV_8UC1);
        for (const auto& person : instances) {
            if (person.mask.type()!=CV_8UC1 || person.mask.size()!=image.size() ||
                (person.boundingBox & cv::Rect(0,0,image.cols,image.rows)) != person.boundingBox ||
                person.confidence <= 0.f)
                throw std::runtime_error("Invalid raw person instance");
            cv::bitwise_or(rawUnion,person.mask,rawUnion);
        }
        ORB_SLAM3::MaskPostProcessor b1Post(detector.PostProcessEnabled(),detector.PostProcessRadius());
        if (cv::norm(mask,b1Post.Apply(rawUnion),cv::NORM_INF)!=0)
            throw std::runtime_error("B1 mask is not union-then-dilate of raw instances");
        const auto pair = detector.DetectStereo(image,cv::Mat::zeros(image.size(),image.type()),false,1);
        if (pair.left.data==pair.right.data || cv::norm(pair.left,mask,cv::NORM_INF)!=0)
            throw std::runtime_error("Stereo masks are shared or nondeterministic");
        if (cv::countNonZero(mask)==0 || cv::countNonZero(pair.right)!=0)
            throw std::runtime_error("Smoke image must contain person; blank right must have none");
        if (!detector.LastRightInstances().empty())
            throw std::runtime_error("Blank right image produced person instances");
        if (detector.PostProcessCalls() != (detector.PostProcessEnabled() ? 3u : 0u))
            throw std::runtime_error("Postprocess not exactly once per view");
        cv::Mat dot=cv::Mat::zeros(19,19,CV_8UC1); dot.at<uchar>(9,9)=255;
        ORB_SLAM3::MaskPostProcessor post(true,3), disabled(false,3);
        cv::Mat expected;
        cv::dilate(dot,expected,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(7,7)));
        if (cv::norm(post.Apply(dot),expected,cv::NORM_INF)!=0 ||
            cv::norm(disabled.Apply(dot),dot,cv::NORM_INF)!=0 || !post.Apply(cv::Mat()).empty())
            throw std::runtime_error("Postprocessor regression");
        if(argc==4) {
            cv::Mat painted=image.clone(),overlay; painted.setTo(cv::Scalar(0,0,255),mask);
            cv::addWeighted(image,.65,painted,.35,0,overlay);
            if(!cv::imwrite(std::string(argv[3])+"_mask.png",mask) ||
               !cv::imwrite(std::string(argv[3])+"_overlay.png",overlay)) throw std::runtime_error("Cannot save smoke images");
        }
        std::cout << "PASS size=" << mask.cols << 'x' << mask.rows
                  << " excluded=" << cv::countNonZero(mask)
                  << " raw_instances=" << instances.size()
                  << " stereo_right_blank=" << cv::countNonZero(pair.right)
                  << " inference_calls=" << detector.InferenceCalls()
                  << " post_calls=" << detector.PostProcessCalls() << '\n';
    } catch(const std::exception& e) { std::cerr << "ERROR: " << e.what() << '\n'; return 2; }
    return 0;
}
