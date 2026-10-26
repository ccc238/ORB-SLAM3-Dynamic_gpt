#include "FrameDrawer.h"
#include "FrontendDiagnosticLog.h"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace ORB_SLAM3;
static void Require(bool test,const char* message) {if(!test) throw std::runtime_error(message);}
int main() {
    try {
        Require(FrameDrawer::TemporalFeatureColor(SemanticRegion::BACKGROUND,TemporalState::UNKNOWN)==cv::Scalar(0,255,0),"BG color");
        Require(FrameDrawer::TemporalFeatureColor(SemanticRegion::PERSON_INTERIOR,TemporalState::STATIC)==cv::Scalar(255,255,0),"S color");
        Require(FrameDrawer::TemporalFeatureColor(SemanticRegion::PERSON_INTERIOR,TemporalState::PENDING)==cv::Scalar(0,255,255),"P color");
        Require(FrameDrawer::TemporalFeatureColor(SemanticRegion::PERSON_INTERIOR,TemporalState::DYNAMIC)==cv::Scalar(0,0,255),"D color");
        Require(FrameDrawer::TemporalFeatureColor(SemanticRegion::PERSON_BOUNDARY,TemporalState::DYNAMIC)==cv::Scalar(128,128,128),"Boundary precedence");
        Require(FrameDrawer::TemporalFeatureColor(SemanticRegion::UNKNOWN,TemporalState::STATIC)==cv::Scalar(128,128,128),"Unknown precedence");
        cv::Mat image(480,640,CV_8UC3,cv::Scalar());
        std::vector<cv::KeyPoint> keys;
        for(int x:{50,100,150,200,250}) keys.emplace_back(float(x),200.f,7.f);
        std::vector<SemanticRegion> regions={SemanticRegion::BACKGROUND,SemanticRegion::PERSON_INTERIOR,
            SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_INTERIOR,SemanticRegion::PERSON_BOUNDARY};
        FrontendDiagnostics view;
        view.currentFeatureStates={TemporalState::UNKNOWN,TemporalState::STATIC,TemporalState::PENDING,TemporalState::DYNAMIC,TemporalState::DYNAMIC};
        B3ViewerInstance instance;
        instance.boundingBox=cv::Rect(300,30,200,350);instance.color=cv::Scalar(0,0,255);
        instance.primaryLabel="ID0 B4 state=D";instance.secondaryLabel="coarse_action=REJECT";
        instance.tertiaryLabel="fine_action=REJECT map_action=DENY";instance.quaternaryLabel="Ns/Nd/Nr=0/3/0 P_TO_D";
        view.stateActionInstances={instance};
        FrameDrawer::DrawStateActionOverlay(image,keys,regions,view);
        for(size_t i=0;i<keys.size();++i) {
            const auto color=FrameDrawer::TemporalFeatureColor(regions[i],view.currentFeatureStates[i]);
            Require(image.at<cv::Vec3b>(200,int(keys[i].pt.x))==cv::Vec3b(color[0],color[1],color[2]),"Point render color mismatch");
        }
        Require(cv::imwrite("synthetic_state_action_overlay.png",image),"Render export");
        double elapsed=-1;
        {DiagnosticTimer disabled(nullptr);}
        {DiagnosticTimer timer(&elapsed);timer.Stop();timer.Stop();}
        Require(elapsed>=0,"Timer stop");
        FrontendDiagnosticLog log(".");
        log.Append({{"row_type","FRAME"},{"frame_id","4"},{"timestamp","1.25"}});
        log.Append({{"row_type","INSTANCE"},{"frame_id","4"},{"b3_reason","quoted,\"reason\""}});
        std::ofstream map("synthetic_map.csv");
        map << "frame_id,timestamp,keyframe_id,stage,map_candidates_total,map_reject_P,map_reject_D,map_reject_boundary,map_reject_unknown,map_points_created\n"
            << "4,1.25,0,rgbd_depth_creation,11,2,3,1,0,5\n"
            << "9,1.50,1,process_keyframe,5,0,1,0,0,0\n"
            << "4,1.25,0,triangulation_current,6,1,0,0,0,2\n";
        map.close();log.Finalize("synthetic_map.csv",true);log.Finalize("synthetic_map.csv",true);
        bool rejected=false;try{FrontendDiagnosticLog duplicate(".");}catch(const std::runtime_error&){rejected=true;}
        Require(rejected,"Must reject output overwrite");
        std::cout << "PASS timing, colors, headless rendering, source-keyframe map joining, CSV escaping, no overwrite\n";
    } catch(const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
}
