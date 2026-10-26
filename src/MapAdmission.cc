#include "MapAdmission.h"
#include "FeaturePolicyEvaluator.h"
#include "KeyFrame.h"
#include "Atlas.h"
#include "Map.h"
#include <fstream>
#include <iostream>
#include <mutex>
#include <iomanip>
#include <stdexcept>

namespace ORB_SLAM3 {
namespace { std::mutex admissionOutputMutex; }
void PrepareFrameMapAdmission(Frame& frame, const std::string& logPath) {
    const auto policy=ComputeFeaturePolicies(frame,FeaturePolicyPhase::MAPPING,true);
    frame.mbMapAdmissionEnabled=true;
    frame.mMapAdmissionLogPath=logPath;
    frame.mvFeaturePolicy=policy;
    for(int i=0;i<frame.N;++i)
        frame.mvAllowMapCreation[i]=FeaturePolicyAllows(policy[i],FeaturePolicyPhase::MAPPING);
}
bool KeyFrame::MapAdmissionMetadataAligned() const {
    const size_t n=static_cast<size_t>(N);
    return mvSemanticRegion.size()==n && mvInstanceId.size()==n && mvTrackId.size()==n &&
        mvCurrentB4State.size()==n && mvFeaturePolicy.size()==n && mvAllowMapCreation.size()==n;
}
void KeyFrame::ResetLegacyMapAdmission() {
    mvSemanticRegion.assign(N,SemanticRegion::UNKNOWN);
    mvInstanceId.assign(N,-1);mvTrackId.assign(N,-1);
    mvCurrentB4State.assign(N,TemporalState::UNKNOWN);
    mvFeaturePolicy.assign(N,FeatureUsePolicy::USE_NORMAL);
    mvAllowMapCreation.assign(N,true);
    mbMapAdmissionEnabled=false;
    mMapAdmissionLogPath.clear();
}
bool KeyFrame::AllowsMapCreation(size_t index) const {
    if(!mbMapAdmissionEnabled) return true;
    if(index>=static_cast<size_t>(N) || !MapAdmissionMetadataAligned())
        throw std::runtime_error("KeyFrame map-admission metadata/index mismatch");
    const bool semanticAllowed=mvSemanticRegion[index]==SemanticRegion::BACKGROUND ||
        (mvSemanticRegion[index]==SemanticRegion::PERSON_INTERIOR && mvInstanceId[index]>=0 &&
         mvTrackId[index]>=0 && mvCurrentB4State[index]==TemporalState::STATIC);
    const bool policyAllowed=FeaturePolicyAllows(mvFeaturePolicy[index],FeaturePolicyPhase::MAPPING);
    if(mvAllowMapCreation[index]!=(semanticAllowed && policyAllowed))
        throw std::runtime_error("KeyFrame map-admission snapshot inconsistent");
    return mvAllowMapCreation[index];
}
MapAdmissionBatch::MapAdmissionBatch(const KeyFrame* keyframe,const char* name):kf(keyframe),stage(name) {}
bool MapAdmissionBatch::Check(size_t index) {
    if(!kf->mbMapAdmissionEnabled) return true;
    ++candidates;
    if(kf->AllowsMapCreation(index)) return true;
    switch(kf->mvFeaturePolicy[index]) {
    case FeatureUsePolicy::EXCLUDE_PENDING: ++pending;break;
    case FeatureUsePolicy::EXCLUDE_DYNAMIC: ++dynamic;break;
    case FeatureUsePolicy::EXCLUDE_BOUNDARY: ++boundary;break;
    default: ++unknown;break;
    }
    return false;
}
void MapAdmissionBatch::Created(size_t index) {
    if(!kf->mbMapAdmissionEnabled) return;
    if(!kf->AllowsMapCreation(index)) throw std::runtime_error("Forbidden feature created a MapPoint");
    ++created;
}
MapAdmissionBatch::~MapAdmissionBatch() {
    if(!kf->mbMapAdmissionEnabled || (!candidates && !created)) return;
    std::lock_guard<std::mutex> lock(admissionOutputMutex);
    std::cout << "[MAP ADMISSION] frame=" << kf->mnFrameId << " kf=" << kf->mnId << " stage=" << stage
        << " map_candidates_total=" << candidates << " map_reject_P=" << pending
        << " map_reject_D=" << dynamic << " map_reject_boundary=" << boundary
        << " map_reject_unknown=" << unknown << " map_points_created=" << created << std::endl;
    if(kf->mMapAdmissionLogPath.empty()) return;
    std::ofstream out(kf->mMapAdmissionLogPath,std::ios::app);
    if(!out) { std::cerr << "Cannot write map admission CSV: " << kf->mMapAdmissionLogPath << std::endl; std::terminate(); }
    if(out.tellp()==0) out << "frame_id,timestamp,keyframe_id,stage,map_candidates_total,map_reject_P,map_reject_D,map_reject_boundary,map_reject_unknown,map_points_created\n";
    out << kf->mnFrameId << ',' << std::setprecision(17) << kf->mTimeStamp << ',' << kf->mnId << ',' << stage
        << ',' << candidates << ',' << pending << ',' << dynamic << ',' << boundary << ',' << unknown << ',' << created << '\n';
    out.flush();
    if(!out) { std::cerr << "Map admission CSV write failed" << std::endl; std::terminate(); }
}
void SaveMapAdmissionAudit(Atlas& atlas,const std::string& filename) {
    long keyframes=0,features=0,forward=0,reverse=0,violations=0,unaligned=0;
    for(auto* map:atlas.GetAllMaps()) {
        for(auto* kf:map->GetAllKeyFrames()) {
            if(!kf->mbMapAdmissionEnabled) continue;
            ++keyframes;
            if(!kf->MapAdmissionMetadataAligned()) {++unaligned;continue;}
            const auto points=kf->GetMapPointMatches();
            if(points.size()!=static_cast<size_t>(kf->N)) {++unaligned;continue;}
            features+=kf->N;
            for(size_t i=0;i<points.size();++i) if(points[i] && !points[i]->isBad()) {
                ++forward;
                if(!kf->AllowsMapCreation(i)) ++violations;
            }
        }
        for(auto* point:map->GetAllMapPoints()) if(!point->isBad())
            for(const auto& obs:point->GetObservations()) {
                auto* kf=obs.first;
                if(!kf->mbMapAdmissionEnabled) continue;
                for(int index:{std::get<0>(obs.second),std::get<1>(obs.second)}) if(index>=0) {
                    ++reverse;
                    if(!kf->AllowsMapCreation(static_cast<size_t>(index))) ++violations;
                }
            }
    }
    std::ofstream out(filename);
    if(!out) throw std::runtime_error("Cannot create map admission audit: "+filename);
    out << "active_keyframes,feature_slots,forward_observations,reverse_observations,forbidden_observations,unaligned_keyframes\n"
        << keyframes << ',' << features << ',' << forward << ',' << reverse << ',' << violations << ',' << unaligned << '\n';
    out.close();
    if(!out || violations || unaligned) throw std::runtime_error("Map admission final observation audit failed");
}
}
