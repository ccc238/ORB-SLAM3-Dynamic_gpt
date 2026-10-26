#include "MapAdmission.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "ORBmatcher.h"
#include "Pinhole.h"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <opencv2/core.hpp>
#include <sstream>
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3;
static void Require(bool value,const char* why) { if(!value) throw std::runtime_error(why); }
int main() {
    try {
        cv::Mat gray(240,320,CV_8UC1),depth(240,320,CV_32F,cv::Scalar(2.f));
        cv::randu(gray,0,255);
        ORBextractor extractor(300,1.2f,4,20,7);
        Pinhole camera({250,250,160,120});
        cv::Mat K=(cv::Mat_<float>(3,3)<<250,0,160,0,250,120,0,0,1),dist=cv::Mat::zeros(4,1,CV_32F);
        Frame frame(gray,depth,0,&extractor,nullptr,K,dist,25,4,&camera);
        frame.SetPose(Sophus::SE3f());
        Require(frame.N>6,"Synthetic features missing");
        frame.mbSemanticAnnotated=true;
        frame.mvSemanticRegion.assign(frame.N,SemanticRegion::BACKGROUND);
        for(int i=1;i<6;++i) {
            frame.mvSemanticRegion[i]=SemanticRegion::PERSON_INTERIOR;
            frame.mvInstanceId[i]=i;frame.mvTrackId[i]=i;
        }
        frame.mvCurrentB4State[1]=TemporalState::STATIC;
        frame.mvCurrentB4State[2]=TemporalState::PENDING;
        frame.mvCurrentB4State[3]=TemporalState::DYNAMIC;
        frame.mvSemanticRegion[4]=SemanticRegion::PERSON_BOUNDARY;
        frame.mvSemanticRegion[5]=SemanticRegion::UNKNOWN;
        Map map(0);
        KeyFrameDatabase db;
        MapPoint sentinel;
        frame.mvpMapPoints.assign(frame.N,&sentinel);
        const auto points=frame.mvpMapPoints;
        const auto outliers=frame.mvbOutlier;
        const cv::Mat descriptors=frame.mDescriptors.clone();
        PrepareFrameMapAdmission(frame,"");
        Require(frame.SemanticMetadataAligned(),"Frame map metadata alignment");
        Require(frame.mvpMapPoints==points && frame.mvbOutlier==outliers &&
            cv::norm(frame.mDescriptors,descriptors,cv::NORM_INF)==0,"Admission preparation changed tracking data");
        Frame copy(frame);
        Require(copy.mvAllowMapCreation==frame.mvAllowMapCreation && copy.mbMapAdmissionEnabled,"Frame copy lost gate");
        KeyFrame target(frame,&map,&db);
        Require(target.MapAdmissionMetadataAligned() && target.N==frame.N && target.mDescriptors.rows==frame.N,
            "KeyFrame metadata / descriptor index alignment");
        for(int i=0;i<6;++i) {
            const bool allow=i==0 || i==1;
            Require(target.AllowsMapCreation(i)==allow && bool(target.GetMapPoint(i))==allow,
                "KeyFrame snapshot leaked P/D/boundary/unknown observations");
        }
        Require(frame.mvpMapPoints==points,"KeyFrame sanitization changed source Frame");
        bool caught=false;
        try {target.AddMapPoint(&sentinel,2);} catch(const std::runtime_error&){caught=true;}
        Require(caught,"Indexed AddMapPoint did not catch bypass");
        caught=false;try {target.ReplaceMapPointMatch(3,&sentinel);}catch(const std::runtime_error&){caught=true;}
        Require(caught,"Indexed replacement did not catch bypass");
        // Round-trip new metadata; old v0 archives call the same legacy reset path.
        std::stringstream buffer;
        {boost::archive::binary_oarchive out(buffer);out << target;}
        KeyFrame restored;
        {boost::archive::binary_iarchive in(buffer);in >> restored;}
        Require(restored.MapAdmissionMetadataAligned() && restored.mbMapAdmissionEnabled &&
            restored.mvTrackId==target.mvTrackId && restored.mvAllowMapCreation==target.mvAllowMapCreation &&
            !restored.AllowsMapCreation(3) && restored.mMapAdmissionLogPath.empty(),"Serialized gate lost on reload");
        std::stringstream legacyBuffer;
        {boost::archive::binary_oarchive out(legacyBuffer);
         boost::serialization::access::serialize(out,target,0);}
        KeyFrame legacyRestored;
        {boost::archive::binary_iarchive in(legacyBuffer);
         boost::serialization::access::serialize(in,legacyRestored,0);}
        Require(!legacyRestored.mbMapAdmissionEnabled && legacyRestored.MapAdmissionMetadataAligned() &&
            legacyRestored.N==target.N && legacyRestored.AllowsMapCreation(3),"Legacy v0 payload compatibility defaults");
        // Real descriptor/projection tests for BOTH Fuse overloads.
        frame.mvpMapPoints.assign(frame.N,nullptr);
        frame.mbMapAdmissionEnabled=false;
        KeyFrame source(frame,&map,&db);
        Eigen::Vector3f position;frame.UnprojectStereo(0,position);
        MapPoint point(position,&source,&map);
        point.AddObservation(&source,0);source.AddMapPoint(&point,0);
        point.ComputeDistinctiveDescriptors();point.UpdateNormalAndDepth();
        frame.mvSemanticRegion.assign(frame.N,SemanticRegion::PERSON_INTERIOR);
        frame.mvInstanceId.assign(frame.N,0);frame.mvTrackId.assign(frame.N,0);
        frame.mvCurrentB4State.assign(frame.N,TemporalState::PENDING);
        PrepareFrameMapAdmission(frame,"");
        KeyFrame blocked(frame,&map,&db);
        ORBmatcher matcher;
        std::vector<MapPoint*> candidates{&point},replacements(1,nullptr);
        Sophus::Sim3f identity;
        Require(matcher.Fuse(&blocked,candidates)==0 && matcher.Fuse(&blocked,identity,candidates,3,replacements)==0 &&
            !point.IsInKeyFrame(&blocked),"Fusion reintroduced forbidden target observations");
        frame.mbMapAdmissionEnabled=false;
        KeyFrame legacy(frame,&map,&db);
        Require(matcher.Fuse(&legacy,candidates)>0 && point.IsInKeyFrame(&legacy),"Disabled gate changed original fusion");
        // Existing-observation replacement only transfers already admitted indices.
        MapPoint replacement(position,&source,&map);
        point.Replace(&replacement);
        for(const auto& obs:replacement.GetObservations()) {
            const int index=std::get<0>(obs.second);
            Require(index<0 || obs.first->AllowsMapCreation(index),"Replacement transferred a forbidden observation");
        }
        frame.mvAllowMapCreation.pop_back();caught=false;
        try {PrepareFrameMapAdmission(frame,"");}catch(const std::runtime_error&){caught=true;}
        Require(caught,"Misaligned admission metadata accepted");
        std::cout << "PASS snapshot/copy, indexed observation guards, P/D/boundary/unknown exclusion, "
            "new archive round-trip, legacy defaults, both Fuse overloads, replacement, disabled legacy fusion\n";
    } catch(const std::exception& error) {std::cerr << "FAIL " << error.what() << '\n';return 1;}
}
