#include "System.h"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
// Exercises the actual System -> Tracking reset boundary, including a
// restarted acquisition timeline. Identity correctness is audited by Python.
int main(int argc,char** argv) {
    if(argc!=5) {std::cerr<<"vocabulary config dataset association\n";return 2;}
    try {
        struct Pair {double time;std::string rgb,depth;};std::vector<Pair> frames;
        std::ifstream in(argv[4]);std::string line;
        while(std::getline(in,line)&&frames.size()<3) {
            if(line.empty()||line[0]=='#')continue;
            Pair f;double depthTime;std::istringstream row(line);
            if(!(row>>f.time>>f.rgb>>depthTime>>f.depth))throw std::runtime_error("bad association");
            frames.push_back(f);
        }
        if(frames.size()!=3)throw std::runtime_error("need three image pairs");
        ORB_SLAM3::System slam(argv[1],argv[2],ORB_SLAM3::System::RGBD,false);
        for(int sequence=0;sequence<3;++sequence) {
            if(sequence==1)slam.ResetActiveMap();
            if(sequence==2)slam.Reset();
            for(const auto& f:frames) {
                auto rgb=cv::imread(std::string(argv[3])+"/"+f.rgb,cv::IMREAD_UNCHANGED);
                auto depth=cv::imread(std::string(argv[3])+"/"+f.depth,cv::IMREAD_UNCHANGED);
                if(rgb.empty()||depth.empty())throw std::runtime_error("missing input");
                slam.TrackRGBD(rgb,depth,f.time);
            }
        }
        slam.Shutdown();std::cout<<"PASS actual System ResetActiveMap/Reset with restarted timestamps\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
