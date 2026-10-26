#include "System.h"
#include "Deep/MaskPreview.h"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

struct InputFrame { double t; std::string left, right; };
int main(int argc,char** argv) {
    if(argc!=9) {
        std::cerr << "Usage: ablation_sequence rgbd|stereo vocabulary settings sequence association max_frames viewer pace\n";
        return 1;
    }
    try {
        const bool stereo=std::string(argv[1])=="stereo";
        if(!stereo && std::string(argv[1])!="rgbd") throw std::runtime_error("Invalid sensor mode");
        const int limit=std::stoi(argv[6]);
        std::ifstream list(argv[5]);
        if(!list) throw std::runtime_error("Cannot read associations");
        std::vector<InputFrame> frames;
        std::string line;
        while(std::getline(list,line)) {
            if(line.empty()||line[0]=='#') continue;
            InputFrame f; double other;
            std::istringstream in(line);
            if(!(in>>f.t>>f.left>>other>>f.right)) throw std::runtime_error("Invalid association line: "+line);
            frames.push_back(f);
            if(limit>0 && int(frames.size())>=limit) break;
        }
        if(frames.empty()) throw std::runtime_error("No frames");
        cv::FileStorage config(argv[3],cv::FileStorage::READ);
        const bool showMasks=!config["Yolo.ShowMask"].empty() && int(config["Yolo.ShowMask"])!=0;
        const bool frameLog=!config["Experiment.FrameLog"].empty() && int(config["Experiment.FrameLog"])!=0;
        std::ofstream timings;
        if(frameLog) { timings.open("frame_timing.csv"); timings<<"frame,timestamp,track_ms,state\n"; }
        ORB_SLAM3::System slam(argv[2],argv[3],stereo?ORB_SLAM3::System::STEREO:ORB_SLAM3::System::RGBD,std::stoi(argv[7])!=0);
        std::vector<double> times;
        size_t lost=0, recentLost=0, ok=0;
        bool previewStopped=false;
        try {
            for(size_t i=0;i<frames.size();++i) {
                const auto& f=frames[i];
                const std::string root=std::string(argv[4])+"/";
                cv::Mat left=cv::imread(root+f.left,stereo?cv::IMREAD_GRAYSCALE:cv::IMREAD_UNCHANGED);
                cv::Mat right=cv::imread(root+f.right,stereo?cv::IMREAD_GRAYSCALE:cv::IMREAD_UNCHANGED);
                if(left.empty()||right.empty()) throw std::runtime_error("Cannot read image pair: "+f.left+" "+f.right);
                const auto start=std::chrono::steady_clock::now();
                if(stereo) slam.TrackStereo(left,right,f.t);
                else slam.TrackRGBD(left,right,f.t);
                const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
                times.push_back(ms);
                const int state=slam.GetTrackingState();
                lost+=state==4; recentLost+=state==3; ok+=state==2;
                if(frameLog) timings<<i<<','<<std::setprecision(17)<<f.t<<','<<ms<<','<<state<<'\n';
                if(showMasks && ORB_SLAM3::MaskPreview::StopRequested()) {
                    previewStopped=true;
                    std::cout<<"[PREVIEW] Q/Esc: stopping after current frame"<<std::endl;
                    break;
                }
                if(std::stoi(argv[8])!=0 && i+1<frames.size()) {
                    const double wait=(frames[i+1].t-f.t)*1000-ms;
                    if(wait>0) std::this_thread::sleep_for(std::chrono::duration<double,std::milli>(wait));
                }
            }
        } catch(...) { slam.Shutdown(); ORB_SLAM3::MaskPreview::Close(); throw; }
        slam.Shutdown();
        ORB_SLAM3::MaskPreview::Close();
        if(ok) { slam.SaveTrajectoryTUM("CameraTrajectory.txt"); slam.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt"); }
        std::sort(times.begin(),times.end()); double total=0;
        for(double value:times)total+=value;
        cv::FileStorage stats("tracking_timing.yaml",cv::FileStorage::WRITE);
        stats<<"frames"<<int(times.size())<<"track_mean_ms"<<total/times.size()
             <<"track_median_ms"<<times[times.size()/2]<<"ok_frames"<<int(ok)
             <<"recently_lost_frames"<<int(recentLost)<<"lost_frames"<<int(lost);
        stats<<"preview_user_stopped"<<int(previewStopped);
        std::cout<<"[RUN] frames="<<times.size()<<" OK="<<ok<<" RECENTLY_LOST="<<recentLost<<" LOST="<<lost<<std::endl;
        if(!ok) throw std::runtime_error("No initialized trajectory; no trajectory exported");
    } catch(const std::exception& e) { std::cerr<<"ERROR: "<<e.what()<<std::endl; return 2; }
    return 0;
}
