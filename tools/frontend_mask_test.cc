#include "Frame.h"
#include "ORBextractor.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdexcept>

void Require(bool condition,const char* message) { if(!condition) throw std::runtime_error(message); }
int main(int argc, char** argv) {
 try {
    if (argc==3) {
        cv::Mat raw=cv::imread(argv[1],cv::IMREAD_GRAYSCALE);
        cv::Mat dilated=cv::imread(argv[2],cv::IMREAD_GRAYSCALE), expected;
        Require(!raw.empty()&&!dilated.empty(),"Cannot read smoke masks");
        cv::dilate(raw,expected,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(7,7)));
        Require(cv::norm(dilated,expected,cv::NORM_INF)==0,"YOLO+post differs from exactly one radius-3 dilation");
    }
    cv::Mat image(480,640,CV_8UC1); cv::RNG rng(20261019); rng.fill(image,cv::RNG::UNIFORM,0,256);
    ORB_SLAM3::ORBextractor extractor(1000,1.2,8,20,7);
    cv::Mat mask=cv::Mat::zeros(image.size(),CV_8UC1); mask.colRange(0,320).setTo(255);
    std::vector<cv::KeyPoint> base,keys; cv::Mat desc; std::vector<int> overlap{0,0};
    extractor(image,cv::Mat(),base,desc,overlap);
    Require(!base.empty(),"Synthetic image must have features");
    extractor(image,mask,keys,desc,overlap);
    Require(!keys.empty(),"Unmasked half must retain features");
    for(const auto& k:keys) Require(mask.at<uchar>(cvRound(k.pt.y),cvRound(k.pt.x))==0,"Feature inside excluded region");
    extractor(image,cv::Mat(image.size(),CV_8UC1,cv::Scalar(255)),keys,desc,overlap);
    Require(keys.empty()&&desc.empty(),"Full mask must yield no features/descriptors");
    extractor(image,cv::Mat(),keys,desc,overlap);
    Require(keys.size()==base.size(),"Empty mask must preserve feature count");
    for(size_t i=0;i<keys.size();++i) Require(keys[i].pt==base[i].pt,"Empty mask changes features");
    ORB_SLAM3::ORBextractor right(1000,1.2,8,20,7);
    ORB_SLAM3::Pinhole pin({500,500,320,240});
    cv::Mat K=(cv::Mat_<float>(3,3)<<500,0,320,0,500,240,0,0,1);
    cv::Mat distortion=cv::Mat::zeros(4,1,CV_32F), blank=cv::Mat::zeros(image.size(),CV_8UC1);
    ORB_SLAM3::Frame frame(image,blank,0,&extractor,&right,nullptr,K,distortion,40,4,&pin);
    Require(frame.N>0,"Left features lost when right has none");
    for(float z:frame.mvDepth) Require(z<0,"No-right match must have invalid depth");
    // Both sides have descriptors, but all Hamming distances are 256: exercise
    // the empty-distance-median guard independently from the empty-side guard.
    frame.mvKeysRight=frame.mvKeys;
    frame.mDescriptors.setTo(0);
    frame.mDescriptorsRight=cv::Mat(frame.N,32,CV_8UC1,cv::Scalar(255));
    frame.ComputeStereoMatches();
    for(float z:frame.mvDepth) Require(z<0,"No descriptor match must have invalid depth");
    cv::Mat all(image.size(),CV_8UC1,cv::Scalar(255));
    ORB_SLAM3::Frame frame2(image,image,0,&extractor,&right,nullptr,K,distortion,40,4,&pin,nullptr,ORB_SLAM3::IMU::Calib(),cv::Mat(),all);
    Require(!frame2.mvKeys.empty()&&frame2.mvKeysRight.empty(),"Right mask must be independent from left");
    ORB_SLAM3::Frame frame3(image,image,0,&extractor,&right,nullptr,K,distortion,40,4,&pin,nullptr,ORB_SLAM3::IMU::Calib(),all,cv::Mat());
    Require(frame3.mvKeys.empty()&&!frame3.mvKeysRight.empty(),"Left mask must be independent from right");
    ORB_SLAM3::KannalaBrandt8 leftFish({500,500,320,240,0,0,0,0}), rightFish({500,500,320,240,0,0,0,0});
    leftFish.mvLappingArea={0,640}; rightFish.mvLappingArea={0,640};
    Sophus::SE3f Tlr(Sophus::SO3f(),Eigen::Vector3f(.08,0,0));
    ORB_SLAM3::Frame fish(image,image,0,&extractor,&right,nullptr,K,distortion,40,4,&leftFish,&rightFish,Tlr,nullptr,ORB_SLAM3::IMU::Calib(),all,cv::Mat());
    Require(fish.Nleft==0&&fish.Nright>0&&fish.mDescriptors.rows==fish.Nright,"Fisheye empty-left concatenation");
    ORB_SLAM3::Frame fishRight(image,image,0,&extractor,&right,nullptr,K,distortion,40,4,&leftFish,&rightFish,Tlr,nullptr,ORB_SLAM3::IMU::Calib(),cv::Mat(),all);
    Require(fishRight.Nright==0&&fishRight.mDescriptors.rows==fishRight.Nleft,"Fisheye empty-right concatenation");
    Require(frame.SemanticMetadataAligned() && frame2.SemanticMetadataAligned() &&
            frame3.SemanticMetadataAligned() && fish.SemanticMetadataAligned() && fishRight.SemanticMetadataAligned(),
            "Stereo/fisheye semantic arrays must follow total descriptor index count");
    std::cout<<"PASS FAST mask coordinates, empty/full masks, independent stereo masks, pinhole empty matches, fisheye empty-side guards, semantic array lengths\n";
 } catch(const std::exception& e) { std::cerr<<"ERROR: "<<e.what()<<'\n'; return 1; }
 return 0;
}
