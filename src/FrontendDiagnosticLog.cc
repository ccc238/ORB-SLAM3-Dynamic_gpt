#include "FrontendDiagnosticLog.h"
#include <sstream>
#include <stdexcept>
#include <array>
#include <algorithm>
namespace ORB_SLAM3 {
namespace {
void Write(std::ostream& out,const DiagnosticRow& row) {
    bool first=true;
    for(const auto& key:FrontendDiagnosticLog::Columns()) {
        if(!first) out << ',';
        first=false;
        auto found=row.find(key);
        if(found==row.end()) continue; // Missing/not-run is NA, never fabricated zero.
        out << '"';
        for(char ch:found->second) { if(ch=='"') out << '"'; out << ch; }
        out << '"';
    }
    out << '\n';
}
void Header(std::ostream& out) {
    bool first=true;
    for(const auto& key:FrontendDiagnosticLog::Columns()) {
        if(!first) out << ',';
        first=false; out << key;
    }
    out << '\n';
}
std::ofstream Fresh(const std::string& path) {
    if(std::ifstream(path).good()) throw std::runtime_error("Refusing to overwrite diagnostics: "+path);
    std::ofstream out(path);
    if(!out) throw std::runtime_error("Cannot create diagnostics: "+path);
    return out;
}
}
const std::vector<std::string>& FrontendDiagnosticLog::Columns() {
    static const std::vector<std::string> columns={
        "row_type","frame_id","timestamp","tracking_state","instance_id","track_id","semantic_region","feature_count",
        "previous_b4_state","current_b4_state","Ns","Nd","Nr","missed_frames","b4_transition_reason",
        "b2_raw_r3d","b2_norm_r3d","b2_high_ratio","b2_reliable","b2_reason",
        "b3_fusion_category","b3_reliable","b3_reason","allow_coarse","allow_fine","allow_map",
        "coarse_action","fine_action","map_action",
        "coarse_matches_before","coarse_rejected","coarse_matches_after","coarse_inliers","coarse_attempts",
        "fine_matches_before","fine_reject_P","fine_reject_D","fine_reject_boundary","fine_reject_unknown",
        "fine_matches_after","fine_inliers","map_reject_P","map_reject_D","map_reject_boundary","map_reject_unknown",
        "map_points_created","map_candidates_total","map_counts_scope","map_counts_pending",
        "fallback","fallback_reason","yolo_ms","orb_ms","coarse_ms","b2_ms","b3_ms","b4_ms","fine_ms",
        "final_orb","raw_fast","b1_mask_rejected","person_main_orb","pose_inliers",
        "mode","keyframe_created","coarse_filter_enabled","fine_policy_enabled","map_admission_enabled"};
    return columns;
}
FrontendDiagnosticLog::FrontendDiagnosticLog(const std::string& parent) : prefix(parent+"/dynamic_frontend") {
    online=Fresh(prefix+"_online.csv"); Header(online);
    auto meta=Fresh(prefix+"_schema.json");
    meta << R"({"version":1,"missing":"empty = not measured/not applicable; -1 in timing = not executed","rows":{"FRAME":"one per image; frame counters and times only","INSTANCE":"one per B4 track; TTL-lost has NOT_VISIBLE and no stale B2/B3 metrics","FEATURE_GROUP":"disjoint main Frame feature groups, feature_count sums to N"},"allow":"policy proposal, NOT proof of application; coarse uses previous state, fine/map current state","actions":"DISABLED, PROPOSE_PASS/REJECT/DENY, PASS/REJECT/DENY, NOT_RUN, RESTORED_FALLBACK; map is indexed feature eligibility, not a promise to insert a KeyFrame","coarse_counts":"last original initial-pose branch attempt; coarse_attempts records number of attempts","fine_counts":"second filter after SearchLocalPoints, immediately before optimization","map_counts":"final file joins asynchronous events by SOURCE_KEYFRAME_FRAME; event checks are not unique features; frame rows only","times_ms":{"orb":"main RGB-D ExtractORB, excludes B2 candidate extraction","coarse":"initial tracking branches, excludes B2/B3/B4 and TrackLocalMap","b2":"Process only, excludes candidates and snapshot preparation","b3":"fusion function","b4":"state machine function","fine":"whole TrackLocalMap, including search/filter/optimization"},"online":"map values pending; dynamic_frontend.csv produced after thread joins; absent final file means shutdown not completed"})" << '\n';
    if(!meta) throw std::runtime_error("Cannot write diagnostic schema");
}
void FrontendDiagnosticLog::Append(const DiagnosticRow& row) {
    if(finalized) throw std::runtime_error("Diagnostics appended after finalization");
    Write(online,row); online.flush();
    if(!online) throw std::runtime_error("Cannot append frontend diagnostics");
    rows.push_back(row);
}
void FrontendDiagnosticLog::Finalize(const std::string& mapCsv,bool mapEnabled) {
    if(finalized) return;
    std::map<std::string,std::array<long long,6>> counts;
    if(mapEnabled) {
        std::ifstream in(mapCsv);
        const bool hasKeyFrames=std::any_of(rows.begin(),rows.end(),[](const DiagnosticRow& row) {
            auto found=row.find("keyframe_created");return found!=row.end() && found->second=="1";
        });
        if(!in && hasKeyFrames) throw std::runtime_error("Missing source-KeyFrame map log: "+mapCsv);
        std::string line; std::getline(in,line);
        if(in && line!="frame_id,timestamp,keyframe_id,stage,map_candidates_total,map_reject_P,map_reject_D,map_reject_boundary,map_reject_unknown,map_points_created")
            throw std::runtime_error("Unexpected map admission CSV schema");
        while(std::getline(in,line)) {
            std::vector<std::string> fields;
            std::istringstream stream(line); std::string cell;
            while(std::getline(stream,cell,',')) fields.push_back(cell);
            if(fields.size()!=10) throw std::runtime_error("Malformed map admission event");
            auto& totals=counts[fields[0]];
            for(int i=0;i<6;++i) totals[i]+=std::stoll(fields[i+4]);
        }
    }
    auto out=Fresh(prefix+".csv"); Header(out);
    const char* keys[]={"map_candidates_total","map_reject_P","map_reject_D","map_reject_boundary","map_reject_unknown","map_points_created"};
    for(auto row:rows) {
        if(row.at("row_type")=="FRAME") {
            const auto totals=counts[row.at("frame_id")];
            if(mapEnabled) for(int i=0;i<6;++i) row[keys[i]]=std::to_string(totals[i]);
            row["map_counts_pending"]="0";
        }
        Write(out,row);
    }
    out.flush();
    if(!out) throw std::runtime_error("Cannot finalize frontend diagnostics");
    finalized=true;
}
}
