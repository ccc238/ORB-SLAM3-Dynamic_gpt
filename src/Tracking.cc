/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"
#include "FrameSemanticAnnotator.h"
#include "FeaturePolicyEvaluator.h"
#include "CoarseFeatureFilter.h"
#include "MapAdmission.h"

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "Optimizer.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "GeometricTools.h"

#include <iostream>
#include <cstring>
#include <iomanip>
#include <unordered_set>

#include <mutex>
#include <chrono>


using namespace std;

namespace ORB_SLAM3
{
namespace {
cv::Mat BackgroundExclusionMask(const cv::Size& size,
    const std::vector<YOLOSegInstance>& persons, int radius)
{
    cv::Mat excluded = cv::Mat::zeros(size,CV_8UC1);
    for (const auto& person : persons) {
        CV_Assert(person.mask.size()==size && person.mask.type()==CV_8UC1);
        cv::bitwise_or(excluded,person.mask,excluded);
    }
    if (radius>0) {
        cv::Mat kernel=cv::Mat::zeros(2*radius+1,2*radius+1,CV_8UC1);
        for (int dy=-radius;dy<=radius;++dy)
            for (int dx=-radius;dx<=radius;++dx)
                if (dx*dx+dy*dy<=radius*radius) kernel.at<uchar>(dy+radius,dx+radius)=255;
        cv::dilate(excluded,excluded,kernel);
    }
    return excluded;
}
}

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq):
    mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
    mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
    mbReadyToInitializate(false), mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr), mpLastKeyFrame(static_cast<KeyFrame*>(NULL))
{
    // Load camera parameters from settings file
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool b_parse_cam = ParseCamParamFile(fSettings);
        if(!b_parse_cam)
        {
            std::cout << "*Error with the camera parameters in the config file*" << std::endl;
        }

        // Load ORB parameters
        bool b_parse_orb = ParseORBParamFile(fSettings);
        if(!b_parse_orb)
        {
            std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
        }

        bool b_parse_imu = true;
        if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO || sensor==System::IMU_RGBD)
        {
            b_parse_imu = ParseIMUParamFile(fSettings);
            if(!b_parse_imu)
            {
                std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
            }

            mnFramesToResetIMU = mMaxFrames;
        }

        if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        mpDynamicDetector.reset(new DynamicDetector(strSettingPath));
        if (mpDynamicDetector->Frontend().useCoarsePoseForB2 && sensor != System::RGBD)
            throw std::runtime_error("TwoStage.UseCoarsePoseForB2 currently supports non-inertial RGB-D only");
        mB2Config = mpDynamicDetector->B2();
        mB3Config = mpDynamicDetector->B3();
        mB4Config = mpDynamicDetector->B4();
        if (mpDynamicDetector->Frontend().diagnosticSaveCsv)
            mpFrontendDiagnosticLog.reset(new FrontendDiagnosticLog(mB2Config.csvOutputParent));
        if (mpDynamicDetector->Frontend().useCoarsePoseForB2)
        {
            const std::string path = mB2Config.csvOutputParent + "/coarse_pose_quality_" +
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".csv";
            if (std::ifstream(path).good()) throw std::runtime_error("Coarse quality CSV already exists: " + path);
            mCoarseQualityCsv.open(path);
            if (!mCoarseQualityCsv) throw std::runtime_error("Cannot create coarse quality CSV: " + path);
            mCoarseQualityCsv << "frame_id,timestamp,match_count,inlier_count,inlier_ratio,median_reprojection_error,"
                "background_features,background_inliers,just_relocalized,initialized,new_map,relocalizing,"
                "just_recovered,initial_success,finite_pose,pure_vo,reliable,fallback,fallback_reason,"
                "min_coarse_inliers,min_coarse_inlier_ratio\n";
        }
        const auto& policyConfig = mpDynamicDetector->Frontend().featurePolicy;
        if(mpDynamicDetector->Frontend().mapAdmissionEnable) {
            mMapAdmissionLogPath=mB2Config.csvOutputParent+"/map_admission_"+
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count())+".csv";
            if(std::ifstream(mMapAdmissionLogPath).good())
                throw std::runtime_error("Map admission CSV already exists: "+mMapAdmissionLogPath);
        }
        if (mpDynamicDetector->Frontend().finePolicyEnable)
        {
            const std::string path=mB2Config.csvOutputParent+"/fine_feature_policy_"+
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count())+".csv";
            if(std::ifstream(path).good()) throw std::runtime_error("Fine policy CSV already exists: "+path);
            mFinePolicyCsv.open(path);
            if(!mFinePolicyCsv) throw std::runtime_error("Cannot create fine policy CSV: "+path);
            const std::string keyframePath=mB2Config.csvOutputParent+"/keyframe_stage_"+
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count())+".csv";
            if(std::ifstream(keyframePath).good()) throw std::runtime_error("Keyframe stage CSV already exists: "+keyframePath);
            mKeyFrameStageCsv.open(keyframePath);
            if(!mKeyFrameStageCsv) throw std::runtime_error("Cannot create keyframe stage CSV: "+keyframePath);
            mKeyFrameStageCsv << "frame_id,timestamp,phase,semantic_frame_id,final_frame_id,coarse_fallback,"
                "coarse_reliable,evidence_complete,fine_completed,optimizer_executed,fine_success,fine_failed,"
                "semantic_policy_available,coarse_matches,coarse_inliers,final_inliers,current_inliers,decision_inliers,"
                "required_static_support,pose_finite,final_pose_unchanged,gate_allowed,reason,coarse_reason,"
                "need_evaluated,needed,create_attempted,created,bootstrap\n";
            mFinePolicyCsv << "frame_id,timestamp,phase,fine_matches_before,fine_reject_P,fine_reject_D,"
                "fine_reject_boundary,fine_reject_unknown,fine_matches_after,fine_inliers,fine_fallback,"
                "fallback_reason,P_at_optimizer,D_at_optimizer,forbidden_at_optimizer,excluded_features,"
                "required_static_support,optimizer_executed,keyframe_blocked\n";
        }
        if (mpDynamicDetector->Frontend().coarseFilterEnable)
        {
            const std::string path = mB2Config.csvOutputParent + "/coarse_feature_filter_" +
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".csv";
            if (std::ifstream(path).good()) throw std::runtime_error("Coarse filter CSV already exists: " + path);
            mCoarseFilterCsv.open(path);
            if (!mCoarseFilterCsv) throw std::runtime_error("Cannot create coarse filter CSV: " + path);
            mCoarseFilterCsv << "frame_id,timestamp,branch,coarse_matches_before,coarse_policy_rejected,"
                "coarse_matches_after,coarse_inliers,coarse_fallback,coarse_fallback_reason,"
                "coarse_policy_applied,restored_inliers,skip_reason\n";
        }
        if (policyConfig.enable && policyConfig.saveSummaryCsv)
        {
            const std::string path = mB2Config.csvOutputParent + "/feature_policy_summary_" +
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".csv";
            if (std::ifstream(path).good())
                throw std::runtime_error("FeaturePolicy CSV already exists; choose a new result directory: " + path);
            mFeaturePolicySummaryCsv.open(path);
            if (!mFeaturePolicySummaryCsv)
                throw std::runtime_error("Cannot create FeaturePolicy CSV: " + path);
            WriteFeaturePolicyCsvHeader(mFeaturePolicySummaryCsv);
        }
        if (mpDynamicDetector->Frontend().featureSemantics.enable &&
            sensor != System::RGBD && sensor != System::IMU_RGBD)
            throw std::runtime_error("FeatureSemantic annotation currently supports RGB-D input only");
        // One identity authority for YOLO instances, B2, B3, and B4.
        mpInstanceTrackManager.reset(new InstanceTrackManager(mB4Config));
        if (mB3Config.enable)
        {
            mpB3Fusion.reset(new B3Fusion(mB3Config.evidenceOnly,
                                           mB3Config.personAsDynamicPrior));
            if (mB3Config.saveSummaryCsv)
            {
                const std::string path = mB2Config.csvOutputParent +
                    "/b3_fusion_summary_" + std::to_string(
                        std::chrono::system_clock::now().time_since_epoch().count()) + ".csv";
                mB3SummaryCsv.open(path);
                if (!mB3SummaryCsv)
                    throw std::runtime_error("Cannot open B3 summary CSV: " + path);
                mB3SummaryCsv << "frame_id,timestamp,instance_id,track_id,class_id,class_name,confidence,"
                              << "semantic_dynamic_prior,geometry_reliable,geometry_state,geometry_reason,"
                              << "candidate_count,matched_count,valid_count,match_ratio,valid_match_ratio,"
                              << "raw_r3d_median,norm_r3d_median,high_ratio,bbox_x,bbox_y,bbox_width,bbox_height,"
                              << "fusion_category,fusion_reliable,"
                              << "fusion_reason,recommended_action,applied_action\n";
            }
        }
        if (mB4Config.enable)
        {
            mpB4StateMachine.reset(new B4TemporalStateMachine(mB4Config));
            if (mB4Config.saveSummaryCsv)
            {
                const std::string path = mB2Config.csvOutputParent +
                    "/b4_state_summary_" + std::to_string(
                        std::chrono::system_clock::now().time_since_epoch().count()) + ".csv";
                mB4SummaryCsv.open(path);
                if (!mB4SummaryCsv)
                    throw std::runtime_error("Cannot open B4 summary CSV: " + path);
                mB4SummaryCsv << "timestamp,frame_id,instance_id,track_id,"
                              << "b3_category,b3_reliable,b3_reason,previous_state,current_state,"
                              << "static_count,dynamic_count,recovery_count,unreliable_count,missed_frames,track_age,"
                              << "transition_reason,recommended_policy,applied_policy,evidence_only,"
                              << "has_reliable_history,evidence_age_seconds,counter_expired,counters_reset,"
                              << "static_state_expired,expiry_reason,expiry_previous_state,expiry_current_state,current_update_reason\n";
            }
        }
        if (mB2Config.enable)
        {
            if (sensor == System::RGBD || sensor == System::IMU_RGBD)
            {
                mpB2Processor.reset(new B2MotionEvidence(mB2Config));
                mpB2Processor->SetCameraCalibration(mK);
            }
            else
                std::cerr << "B2 RGB-D geometry is disabled for stereo input" << std::endl;
        }
    }
    initID = 0; lastID = 0;
    mbInitWith3KFs = false;
    mnNumDataset = 0;

    vector<GeometricCamera*> vpCams = mpAtlas->GetAllCameras();
    std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    for(GeometricCamera* pCam : vpCams)
    {
        std::cout << "Camera " << pCam->GetId();
        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            std::cout << " is pinhole" << std::endl;
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            std::cout << " is fisheye" << std::endl;
        }
        else
        {
            std::cout << " is unknown" << std::endl;
        }
    }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif
}

#ifdef REGISTER_TIMES
double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<int> v_values, double average)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

void Tracking::LocalMapStats2File()
{
    ofstream f;
    f.open("LocalMapTimeStats.txt");
    f << fixed << setprecision(6);
    f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
    for(int i=0; i<mpLocalMapper->vdLMTotal_ms.size(); ++i)
    {
        f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
          << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBASync_ms[i] << ","
          << mpLocalMapper->vdKFCullingSync_ms[i] <<  "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    f.open("LBA_Stats.txt");
    f << fixed << setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for(int i=0; i<mpLocalMapper->vdLBASync_ms.size(); ++i)
    {
        f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
          << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
          << mpLocalMapper->vnLBA_edges[i] << endl;
    }


    f.close();
}

void Tracking::TrackStats2File()
{
    ofstream f;
    f.open("SessionInfo.txt");
    f << fixed;
    f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
    f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

    f << "OpenCV version: " << CV_VERSION << endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << fixed << setprecision(6);

    f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

    for(int i=0; i<vdTrackTotal_ms.size(); ++i)
    {
        double stereo_rect = 0.0;
        if(!vdRectStereo_ms.empty())
        {
            stereo_rect = vdRectStereo_ms[i];
        }

        double resize_image = 0.0;
        if(!vdResizeImage_ms.empty())
        {
            resize_image = vdResizeImage_ms[i];
        }

        double stereo_match = 0.0;
        if(!vdStereoMatch_ms.empty())
        {
            stereo_match = vdStereoMatch_ms[i];
        }

        double imu_preint = 0.0;
        if(!vdIMUInteg_ms.empty())
        {
            imu_preint = vdIMUInteg_ms[i];
        }

        f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
          << vdPosePred_ms[i] <<  "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
    }

    f.close();
}

void Tracking::PrintTimeStats()
{
    // Save data in files
    TrackStats2File();
    LocalMapStats2File();


    ofstream f;
    f.open("ExecMean.txt");
    f << fixed;
    //Report the mean and std of each one
    std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    cout << "OpenCV version: " << CV_VERSION << endl;
    f << "OpenCV version: " << CV_VERSION << endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    if(!vdRectStereo_ms.empty())
    {
        average = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdResizeImage_ms.empty())
    {
        average = calcAverage(vdResizeImage_ms);
        deviation = calcDeviation(vdResizeImage_ms, average);
        std::cout << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
        f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if(!vdStereoMatch_ms.empty())
    {
        average = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdIMUInteg_ms.empty())
    {
        average = calcAverage(vdIMUInteg_ms);
        deviation = calcDeviation(vdIMUInteg_ms, average);
        std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    std::cout << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    std::cout << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    average = calcAverage(mpLocalMapper->vdKFInsert_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
    std::cout << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
    std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCreation_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
    std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLBA_ms);
    deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
    std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdKFCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
    std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLMTotal_ms);
    deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
    std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping LBA complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_edges);
    deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
    std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFopt);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
    std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
    std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_MPs);
    deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
    std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
    f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

    // Map complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Map complexity" << std::endl;
    std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
    std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBestMap = vpMaps[0];
    for(int i=1; i<vpMaps.size(); ++i)
    {
        if(pBestMap->GetAllKeyFrames().size() < vpMaps[i]->GetAllKeyFrames().size())
        {
            pBestMap = vpMaps[i];
        }
    }

    f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

    f << "---------------------------" << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdDataQuery_ms);
    deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdEstSim3_ms);
    deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdPRTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
    f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
    f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
    f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nLoop << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
    average = calcAverage(mpLoopClosing->vnLoopKFs);
    deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
    f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
    f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
    f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
    f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nMerges << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeKFs);
    deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeMPs);
    deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdGBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
    f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
    deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
    f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
    f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    f << "Numb abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAKFs);
    deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAMPs);
    deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f.close();

}

#endif

Tracking::~Tracking()
{
    //f_track_stats.close();

}

void Tracking::newParameterLoader(Settings *settings) {
    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    if(settings->needToUndistort()){
        mDistCoef = settings->camera1DistortionCoef();
    }
    else{
        mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    }

    //TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK = cv::Mat::eye(3,3,CV_32F);
    mK.at<float>(0,0) = mpCamera->getParameter(0);
    mK.at<float>(1,1) = mpCamera->getParameter(1);
    mK.at<float>(0,2) = mpCamera->getParameter(2);
    mK.at<float>(1,2) = mpCamera->getParameter(3);

    mK_.setIdentity();
    mK_(0,0) = mpCamera->getParameter(0);
    mK_(1,1) = mpCamera->getParameter(1);
    mK_(0,2) = mpCamera->getParameter(2);
    mK_(1,2) = mpCamera->getParameter(3);

    if((mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD) &&
        settings->cameraType() == Settings::KannalaBrandt){
        mpCamera2 = settings->camera2();
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = settings->Tlr();

        mpFrameDrawer->both = true;
    }

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD ){
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    if(mSensor==System::RGBD || mSensor==System::IMU_RGBD){
        mDepthMapFactor = settings->depthMapFactor();
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    //ORB parameters
    int nFeatures = settings->nFeatures();
    int nLevels = settings->nLevels();
    int fIniThFAST = settings->initThFAST();
    int fMinThFAST = settings->minThFAST();
    float fScaleFactor = settings->scaleFactor();

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    //IMU parameters
    Sophus::SE3f Tbc = settings->Tbc();
    mInsertKFsLost = settings->insertKFsWhenLost();
    mImuFreq = settings->imuFrequency();
    mImuPer = 0.001; //1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
    float Ng = settings->noiseGyro();
    float Na = settings->noiseAcc();
    float Ngw = settings->gyroWalk();
    float Naw = settings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];
    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        if(mImageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx = fx * mImageScale;
            fy = fy * mImageScale;
            cx = cx * mImageScale;
            cy = cy * mImageScale;
        }

        vector<float> vCamCalib{fx,fy,cx,cy};

        mpCamera = new Pinhole(vCamCalib);

        mpCamera = mpAtlas->AddCamera(mpCamera);

        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- Image scale: " << mImageScale << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5)
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;

        mK_.setIdentity();
        mK_(0,0) = fx;
        mK_(1,1) = fy;
        mK_(0,2) = cx;
        mK_(1,2) = cy;
    }
    else if(sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if(!b_miss_params)
        {
            if(mImageScale != 1.f)
            {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);
            mpCamera = mpAtlas->AddCamera(mpCamera);
            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- Image scale: " << mImageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;

            mK_.setIdentity();
            mK_(0,0) = fx;
            mK_(1,1) = fy;
            mK_(0,2) = cx;
            mK_(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD){
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            cv::Mat cvTlr;
            if(!node.empty())
            {
                cvTlr = node.mat();
                if(cvTlr.rows != 3 || cvTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params)
            {
                if(mImageScale != 1.f)
                {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;

                    leftLappingBegin = leftLappingBegin * mImageScale;
                    leftLappingEnd = leftLappingEnd * mImageScale;
                    rightLappingBegin = rightLappingBegin * mImageScale;
                    rightLappingEnd = rightLappingEnd * mImageScale;
                }

                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);
                mpCamera2 = mpAtlas->AddCamera(mpCamera2);

                mTlr = Converter::toSophus(cvTlr);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << mImageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << cvTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params)
        {
            return false;
        }

    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD )
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if(!node.empty() && node.isReal())
        {
            mbf = node.real();
            if(mImageScale != 1.f)
            {
                mbf *= mImageScale;
            }
        }
        else
        {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD)
    {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty()  && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf*mThDepth/fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }
        else
        {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }


    }

    if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal())
        {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    if(b_miss_params)
    {
        return false;
    }

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::Mat cvTbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty())
    {
        cvTbc = node.mat();
        if(cvTbc.rows != 4 || cvTbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }
    cout << endl;
    cout << "Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
    Eigen::Matrix<float,4,4,Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    Sophus::SE3f Tbc(eigTbc);

    node = fSettings["InsertKFsWhenLost"];
    mInsertKFsLost = true;
    if(!node.empty() && node.isInt())
    {
        mInsertKFsLost = (bool) node.operator int();
    }

    if(!mInsertKFsLost)
        cout << "Do not insert keyframes when lost visual tracking " << endl;



    float Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt())
    {
        mImuFreq = node.operator int();
        mImuPer = 0.001; //1.0 / (double) mImuFreq;
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.fastInit"];
    mFastInit = false;
    if(!node.empty())
    {
        mFastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
    }

    if(mFastInit)
        cout << "Fast IMU initialization. Acceleration is not checked \n";

    if(b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(mImuFreq);
    cout << endl;
    cout << "IMU frequency: " << mImuFreq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);


    return true;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

void Tracking::SetStepByStep(bool bSet)
{
    bStepByStep = bSet;
}

bool Tracking::GetStepByStep()
{
    return bStepByStep;
}



Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename)
{
    const StereoMasks masks = mpDynamicDetector->DetectStereo(imRectLeft, imRectRight, mbRGB, timestamp);
    const bool rejectMask = mpDynamicDetector->MaskFeatureRejectEnabled();
    //cout << "GrabImageStereo" << endl;

    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    mImRight = imRectRight;

    if(mImGray.channels()==3)
    {
        //cout << "Image with 3 channels" << endl;
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        //cout << "Image with 4 channels" << endl;
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
        }
    }

    if (imGrayRight.channels() == 3)
        cvtColor(imGrayRight,imGrayRight,mbRGB ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
    else if (imGrayRight.channels() == 4)
        cvtColor(imGrayRight,imGrayRight,mbRGB ? cv::COLOR_RGBA2GRAY : cv::COLOR_BGRA2GRAY);

    //cout << "Incoming frame creation" << endl;

    if (mSensor == System::STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera, nullptr, IMU::Calib(), rejectMask ? masks.left : cv::Mat(), rejectMask ? masks.right : cv::Mat());
    else if(mSensor == System::STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr, nullptr, IMU::Calib(), rejectMask ? masks.left : cv::Mat(), rejectMask ? masks.right : cv::Mat());
    else if(mSensor == System::IMU_STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib, rejectMask ? masks.left : cv::Mat(), rejectMask ? masks.right : cv::Mat());
    else if(mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr,&mLastFrame,*mpImuCalib, rejectMask ? masks.left : cv::Mat(), rejectMask ? masks.right : cv::Mat());

    //cout << "Incoming frame ended" << endl;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

    //cout << "Tracking start" << endl;
    Track();
    EmitFrontendDiagnostics();
    //cout << "Tracking end" << endl;

    return mCurrentFrame.GetPose();
}


Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename)
{
    if (mpB4StateMachine && mB4Config.evidenceExpiryEnable)
        mpB4StateMachine->ValidateTimestamp(timestamp);
    mDiagnosticStages=FrontendStageMetrics();
    mbCollectStageDiagnostics=mpDynamicDetector->Frontend().CollectDiagnostics();
    mbRGBDEvidenceReady = false;
    mbRGBDEvidenceProcessed = false;
    mbTwoStageFineCompleted = false;
    mCoarseFilterFallbackReason.clear();
    mLastFinePolicyReport=FineFeaturePolicyReport();
    mbFinePolicyApplied=false;
    mbFineOptimizationExecuted=false;
    mbFineTrackingFailed=false;
    mKeyFrameDecision=KeyFrameDecisionContext();
    const int incomingState = mState;
    mCoarsePoseQuality = CoarsePoseQuality();
    mCoarsePoseContext = CoarsePoseContext();
    mCoarsePoseContext.initialized = mState != NO_IMAGES_YET && mState != NOT_INITIALIZED;
    mCoarsePoseContext.just_recovered = mbCoarseRecoveryPending;
    mbCoarseRecoveryPending = false;
    if (mpDynamicDetector->Frontend().useCoarsePoseForB2)
    {
        Map* currentMap = mpAtlas->GetCurrentMap();
        const long mapId = currentMap ? static_cast<long>(currentMap->GetId()) : -1;
        mCoarsePoseContext.new_map = mbCreatedMap || (!mbOnlyTracking && mState == LOST) ||
            (mLastCoarseMapId >= 0 && (mapId != mLastCoarseMapId || mState == NO_IMAGES_YET));
        mLastCoarseMapId = mapId;
    }
    const cv::Mat dynamicMask = mpDynamicDetector->Detect(imRGB, mbRGB, timestamp);
    if (mpInstanceTrackManager)
    {
        mpInstanceTrackManager->Associate(mpDynamicDetector->MutableLastRGBDInstances(), imRGB.size());
        std::unordered_set<int> assignedIds;
        std::ostringstream ids;
        for (const auto& person : mpDynamicDetector->LastRGBDInstances())
        {
            if (person.trackId < 0 || !assignedIds.insert(person.trackId).second)
                throw std::runtime_error("Person track_id must be valid and unique before Frame construction");
            if (assignedIds.size() > 1) ids << ';';
            ids << person.trackId;
        }
        if (mB4Config.enable && mB4Config.printFrameSummary)
        {
            std::ostringstream association;
            association << "[INSTANCE TRACK] timestamp=" << std::setprecision(17) << timestamp
                        << " persons=" << assignedIds.size() << " track_ids=" << ids.str();
            cout << association.str() << endl;
        }
    }
    const DynamicFrontendConfig& frontend = mpDynamicDetector->Frontend();
    // Explicit modes own the extraction policy. In particular, evidence-only
    // mode keeps person features in the main Frame; its independent B2 ORB
    // candidates below are never inserted into the main feature arrays.
    // Old YAML without Mode retains exactly its former B1 switch behavior.
    const bool useB1FeatureMask = frontend.explicitMode ?
        frontend.mode == DynamicFrontendMode::B1_HARD_MASK :
        mpDynamicDetector->MaskFeatureRejectEnabled();
    const cv::Mat featureMask = useB1FeatureMask ? dynamicMask : cv::Mat();
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    B2CandidateFrame b2Frame;
    if (mpB2Processor)
        b2Frame = mpB2Processor->ExtractCandidates(
            mImGray, imDepth, mpDynamicDetector->LastRGBDInstances(),
            timestamp, *mpORBextractorLeft);

    if (mSensor == System::RGBD)
        mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera, nullptr, IMU::Calib(), featureMask, mbCollectStageDiagnostics);
    else if(mSensor == System::IMU_RGBD)
        mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib, featureMask, mbCollectStageDiagnostics);

    const auto temporalState = [](B4MotionState state) {
        return state == B4MotionState::STATIC ? TemporalState::STATIC :
            state == B4MotionState::DYNAMIC ? TemporalState::DYNAMIC : TemporalState::PENDING;
    };
    if (frontend.featureSemantics.enable)
    {
        std::unordered_map<int, TemporalState> previousStates;
        if (mpB4StateMachine)
            for (const auto& person : mpDynamicDetector->LastRGBDInstances())
            {
                B4MotionState state = B4MotionState::PENDING;
                mpB4StateMachine->GetUsableState(person.trackId, timestamp, state); // NEW/stale S starts P.
                previousStates[person.trackId] = temporalState(state);
            }
        AnnotateRGBDFeatures(mCurrentFrame, mImGray.size(), imDepth,
            mpDynamicDetector->LastRGBDInstances(), previousStates, frontend.featureSemantics);
    }
    // Compute the historical coarse proposal BEFORE Track(). It is NOT used
    // by matching/optimization. Fine/map proposals are computed only after B4.
    std::vector<FeatureUsePolicy> coarsePolicySnapshot;
    if (frontend.featurePolicy.enable)
        coarsePolicySnapshot = ComputeFeaturePolicies(mCurrentFrame,FeaturePolicyPhase::COARSE);

    // Read-only extraction diagnostics, before any Tracking association or
    // pose optimization. Raw masks (not the dilated B1 mask) and mvKeys share
    // the actual extraction-image coordinates. Never mark or remove a key here.
    int personMainOrb = 0;
    const auto& persons = mpDynamicDetector->LastRGBDInstances();
    for (const auto& person : persons)
        CV_Assert(person.mask.type() == CV_8UC1 && person.mask.size() == mImGray.size());
    for (const cv::KeyPoint& key : mCurrentFrame.mvKeys)
    {
        const int x = cvRound(key.pt.x), y = cvRound(key.pt.y);
        if (x < 0 || y < 0 || x >= mImGray.cols || y >= mImGray.rows)
            continue;
        for (const auto& person : persons)
            if (person.mask.at<uchar>(y,x) != 0)
            {
                ++personMainOrb;
                break; // An overlapping instance must not double-count a key.
            }
    }
    cout << "[DYNAMIC FRONTEND] frame_id=" << mCurrentFrame.mnId
         << " dynamic_frontend_mode=" << (frontend.explicitMode ? static_cast<int>(frontend.mode) : -1)
         << " raw_fast=" << mCurrentFrame.mnRawFastCandidatesLeft
         << " b1_mask_rejected=" << mCurrentFrame.mnMaskedRejectedLeft
         << " final_orb=" << mCurrentFrame.N
         << " person_main_orb=" << personMainOrb << endl;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    mPendingRGBDB2Frame = std::move(b2Frame);
    mPendingRGBDB2Frame.frameId = mCurrentFrame.mnId;
    if(frontend.finePolicyEnable && mCurrentFrame.mbSemanticAnnotated && mCurrentFrame.SemanticMetadataAligned())
        mKeyFrameDecision.semanticFrameId=static_cast<long>(mCurrentFrame.mnId);
    mRGBDCoarsePolicySnapshot = std::move(coarsePolicySnapshot);
    mbRGBDEvidenceReady = true;
    Track();
    if (mpDynamicDetector->Frontend().useCoarsePoseForB2 &&
        ((mState == OK && (incomingState == RECENTLY_LOST || incomingState == LOST)) ||
         mCoarsePoseQuality.just_relocalized))
        mbCoarseRecoveryPending = true;
    // Never recompute evidence with the refined pose in coarse-evidence mode.
    // Initialization and other early returns still advance evidence once, with
    // an invalid coarse pose, so they cannot drive a state transition.
    if (!mbRGBDEvidenceProcessed)
        RunRGBDGeometryEvidence(!frontend.useCoarsePoseForB2 && mCurrentFrame.HasPose(),
            frontend.useCoarsePoseForB2 ? "bootstrap_or_unavailable" : "final");
    if (frontend.useCoarsePoseForB2)
        cout << "[TWO STAGE] frame=" << mPendingRGBDB2Frame.frameId
             << " event=frame_complete fine_completed=" << int(mbTwoStageFineCompleted)
             << " evidence_updates=1 applied="
             << int(std::count(mCurrentFrame.mvDynamicExcluded.begin(),mCurrentFrame.mvDynamicExcluded.end(),true)>0)
             << " evidence_applied=0" << endl;
    if(frontend.finePolicyEnable) RecordKeyFrameStage("frame_summary");
    mbRGBDEvidenceReady = false;
    if(frontend.finePolicyEnable)
        for(auto& result:mCurrentB4Results)
            result.appliedPolicy=result.isLost ? "NONE_NO_CURRENT_INSTANCE" :
                mbFinePolicyApplied ? "FINE_"+result.recommendedPolicy : "NONE_FINE_NOT_RUN";
    UpdateDynamicLogs();
    EmitFrontendDiagnostics();

    return mCurrentFrame.GetPose();
}

void Tracking::ApplyFineFeaturePolicy(const char* phase)
{
    if(!mbRGBDEvidenceReady || !mpDynamicDetector->Frontend().finePolicyEnable) return;
    if(!mbRGBDEvidenceProcessed)
        throw std::runtime_error("Fine feature policy requires current-frame B2/B3/B4 completion");
    mLastFinePolicyReport=ApplyCurrentFineFeaturePolicy(mCurrentFrame);
    mbFinePolicyApplied=true;
    ReportFineFeaturePolicy(phase,-1,false,
        FineStaticSupportRequired(mCurrentFrame.mnId,mnLastRelocFrameId,mMaxFrames,mState==RECENTLY_LOST));
}

void Tracking::ReportFineFeaturePolicy(const char* phase,int inliers,bool failed,int requiredSupport)
{
    const auto& r=mLastFinePolicyReport;
    if(mbCollectStageDiagnostics) {
        if(std::strcmp(phase,"pre_pose_optimization")==0) {
            mDiagnosticStages.fineBefore=r.before; mDiagnosticStages.fineP=r.rejectPending;
            mDiagnosticStages.fineD=r.rejectDynamic; mDiagnosticStages.fineBoundary=r.rejectBoundary;
            mDiagnosticStages.fineUnknown=r.rejectUnknown; mDiagnosticStages.fineAfter=r.after;
        }
        if(std::strcmp(phase,"result")==0) mDiagnosticStages.fineInliers=inliers;
    }
    if(failed) mbFineTrackingFailed=true;
    if(std::strcmp(phase,"result")==0) {
        mKeyFrameDecision.finalFrameId=static_cast<long>(mCurrentFrame.mnId);
        mKeyFrameDecision.finalInliers=inliers;
        mKeyFrameDecision.requiredStaticSupport=requiredSupport;
        mKeyFrameDecision.fineSucceeded=!failed;
        if(mCurrentFrame.HasPose()) mFinalKeyFramePose=mCurrentFrame.GetPose();
    }
    const char* reason=failed ? "FINE_TRACKING_INSUFFICIENT_STATIC_SUPPORT" :
        inliers<0 ? "AWAITING_FINE_OPTIMIZATION" : "NA";
    cout << "[FINE POLICY] frame=" << mCurrentFrame.mnId << " phase=" << phase
         << " fine_matches_before=" << r.before << " fine_reject_P=" << r.rejectPending
         << " fine_reject_D=" << r.rejectDynamic << " fine_reject_boundary=" << r.rejectBoundary
         << " fine_reject_unknown=" << r.rejectUnknown << " fine_matches_after=" << r.after
         << " fine_inliers=" << inliers << " fine_fallback=" << int(failed)
         << " reason=" << reason << " P_at_optimizer=" << r.pendingAtOptimizer
         << " D_at_optimizer=" << r.dynamicAtOptimizer
         << " forbidden_at_optimizer=" << r.forbiddenAtOptimizer
         << " optimizer_executed=" << int(mbFineOptimizationExecuted)
         << " required_static_support=" << requiredSupport
         << " keyframe_blocked=" << int(failed) << endl;
    mFinePolicyCsv << std::setprecision(17) << mCurrentFrame.mnId << ',' << mCurrentFrame.mTimeStamp << ','
        << phase << ',' << r.before << ',' << r.rejectPending << ',' << r.rejectDynamic << ','
        << r.rejectBoundary << ',' << r.rejectUnknown << ',' << r.after << ',' << inliers << ','
        << int(failed) << ',' << reason << ',' << r.pendingAtOptimizer << ',' << r.dynamicAtOptimizer << ','
        << r.forbiddenAtOptimizer << ',' << r.excludedFeatures << ',' << requiredSupport << ','
        << int(mbFineOptimizationExecuted) << ',' << int(failed) << '\n';
    mFinePolicyCsv.flush();
    if(!mFinePolicyCsv) throw std::runtime_error("Cannot write fine feature policy CSV");
}

void Tracking::OptimizeNormalInitialPose(int& matches, const char* branch)
{
    // Never called by Relocalization(): its matching/optimizer path is unchanged.
    if (!mbRGBDEvidenceReady || !mpDynamicDetector->Frontend().coarseFilterEnable)
    {
        OptimizeInitialPoseWithDiagnostics();
        return;
    }
    const auto& config = mpDynamicDetector->Frontend();
    CoarseFilterReport report;
    std::string skip = "NA";
    if (!mCoarsePoseContext.initialized) skip = "NOT_INITIALIZED";
    else if (mCoarsePoseContext.new_map) skip = "NEW_MAP";
    else if (mCoarsePoseContext.relocalizing || mState != OK) skip = "RELOCALIZATION";
    else if (mCoarsePoseContext.just_recovered) skip = "JUST_RECOVERED";
    else if (mbOnlyTracking && mbVO) skip = "POSE_NOT_RELIABLE";
    else if (!mCoarseFilterFallbackReason.empty()) skip = "FRAME_ALREADY_FELL_BACK";
    if (skip != "NA")
    {
        report.before = std::count_if(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),
            [](const MapPoint* point){return point!=nullptr;});
        report.after = report.before;
        report.inliers = OptimizeInitialPoseWithDiagnostics();
    }
    else
    {
        const auto proposed = ComputeFeaturePolicies(mCurrentFrame,FeaturePolicyPhase::COARSE,true);
        if (proposed != mRGBDCoarsePolicySnapshot)
            throw std::runtime_error("Coarse filter must use frozen previous B4 state");
        const cv::Mat excluded = BackgroundExclusionMask(mImGray.size(),
            mpDynamicDetector->LastRGBDInstances(),mB2Config.backgroundExclusionDilateRadius);
        report = OptimizeWithCoarseFeatureFilter(mCurrentFrame,proposed,
            config.minCoarseInliers,config.minCoarseInliers,config.minCoarseInlierRatio,
            mbOnlyTracking ? 0 : 10,
            [&](){return OptimizeInitialPoseWithDiagnostics();},
            [&](){
                int mapped = 0;
                for (int i=0;i<mCurrentFrame.N;++i)
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i] &&
                        mCurrentFrame.mvpMapPoints[i]->Observations()>0) ++mapped;
                return mapped;
            },
            [&](){
                int background = 0;
                for (int i=0;i<mCurrentFrame.N;++i) {
                    const auto& pixel=mCurrentFrame.mvKeys[i].pt;
                    const int x=cvRound(pixel.x),y=cvRound(pixel.y);
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i] &&
                        x>=0 && y>=0 && x<excluded.cols && y<excluded.rows && !excluded.at<uchar>(y,x))
                        ++background;
                }
                return background>=config.minCoarseInliers ? std::string() :
                    std::string("COARSE_FILTER_INSUFFICIENT_BACKGROUND_INLIERS");
            });
        if (report.applied) matches -= report.rejected;
        if (report.fallback) mCoarseFilterFallbackReason = report.reason;
    }
    if(mbCollectStageDiagnostics) {
        ++mDiagnosticStages.coarseAttempts;
        mDiagnosticStages.coarseBefore=report.before;
        mDiagnosticStages.coarseRejected=report.rejected;
        mDiagnosticStages.coarseAfter=report.after;
        mDiagnosticStages.coarseInliers=report.inliers;
        mDiagnosticStages.coarseApplied=report.applied;
    }
    cout << "[COARSE FILTER] frame=" << mCurrentFrame.mnId << " branch=" << branch
         << " coarse_matches_before=" << report.before << " coarse_policy_rejected=" << report.rejected
         << " coarse_matches_after=" << report.after << " coarse_inliers=" << report.inliers
         << " coarse_fallback=" << int(report.fallback) << " coarse_fallback_reason=" << report.reason
         << " coarse_policy_applied=" << int(report.applied) << " restored_inliers=" << report.restoredInliers
         << " skip_reason=" << skip << endl;
    mCoarseFilterCsv << std::setprecision(17) << mCurrentFrame.mnId << ',' << mCurrentFrame.mTimeStamp << ','
        << branch << ',' << report.before << ',' << report.rejected << ',' << report.after << ','
        << report.inliers << ',' << int(report.fallback) << ',' << report.reason << ','
        << int(report.applied) << ',' << report.restoredInliers << ',' << skip << '\n';
    mCoarseFilterCsv.flush();
    if (!mCoarseFilterCsv) throw std::runtime_error("Cannot write coarse filter CSV");
}

int Tracking::OptimizeInitialPoseWithDiagnostics()
{
    const bool active = mbRGBDEvidenceReady && mpDynamicDetector->Frontend().useCoarsePoseForB2;
    if (!active) return Optimizer::PoseOptimization(&mCurrentFrame);
    mCoarsePoseQuality = CoarsePoseQuality();
    for (const auto* point : mCurrentFrame.mvpMapPoints)
        if (point) ++mCoarsePoseQuality.match_count;
    const int inliers = Optimizer::PoseOptimization(&mCurrentFrame); // Same single original call.
    mCoarsePoseQuality.inlier_count = inliers;
    std::vector<float> errors;
    if (mCurrentFrame.HasPose())
        for (int i = 0; i < mCurrentFrame.N; ++i)
        {
            auto* point = mCurrentFrame.mvpMapPoints[i];
            if (!point || mCurrentFrame.mvbOutlier[i]) continue;
            const Eigen::Vector3f cameraPoint = mCurrentFrame.GetPose() * point->GetWorldPos();
            if (!cameraPoint.allFinite() || cameraPoint.z() <= 0.f) continue;
            const Eigen::Vector2f projected = mpCamera->project(cameraPoint);
            const cv::Point2f pixel = mCurrentFrame.mvKeysUn[i].pt;
            const float error = (projected - Eigen::Vector2f(pixel.x,pixel.y)).norm();
            if (std::isfinite(error)) errors.push_back(error);
        }
    if (!errors.empty())
    {
        std::sort(errors.begin(),errors.end());
        const size_t mid = errors.size()/2;
        mCoarsePoseQuality.median_reprojection_error = errors.size()%2 ?
            errors[mid] : (errors[mid-1]+errors[mid])*.5f;
    }
    return inliers;
}

void Tracking::FinalizeCoarsePoseQuality(bool initialSuccess)
{
    const auto& config = mpDynamicDetector->Frontend();
    auto& context = mCoarsePoseContext;
    context.initial_success = initialSuccess;
    context.finite_pose = mCurrentFrame.HasPose() && mCurrentFrame.GetPose().matrix().allFinite();
    context.pure_vo = mbOnlyTracking && mbVO;
    mCoarsePoseQuality.just_relocalized = context.relocalizing && initialSuccess &&
        mnLastRelocFrameId == mPendingRGBDB2Frame.frameId;
    const cv::Mat excluded = BackgroundExclusionMask(mImGray.size(),
        mpDynamicDetector->LastRGBDInstances(),mB2Config.backgroundExclusionDilateRadius);
    context.background_features = context.background_inliers = 0;
    for (int i=0;i<mCurrentFrame.N;++i)
    {
        const auto& pixel=mCurrentFrame.mvKeys[i].pt;
        const int x=cvRound(pixel.x), y=cvRound(pixel.y);
        if (x<0 || y<0 || x>=excluded.cols || y>=excluded.rows || excluded.at<uchar>(y,x)) continue;
        ++context.background_features;
        if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
            ++context.background_inliers;
    }
    mCoarsePoseQuality=EvaluateCoarsePoseQuality(mCoarsePoseQuality,context,
        config.minCoarseInliers,config.minCoarseInlierRatio);
    if (!mCoarseFilterFallbackReason.empty())
    {
        mCoarsePoseQuality.reliable = false;
        mCoarsePoseQuality.reason = mCoarseFilterFallbackReason;
    }
    const auto& quality=mCoarsePoseQuality;
    cout << "[COARSE QUALITY] frame=" << mPendingRGBDB2Frame.frameId
         << " matches=" << quality.match_count << " inliers=" << quality.inlier_count
         << " inlier_ratio=" << quality.inlier_ratio
         << " median_reprojection_error=" << quality.median_reprojection_error
         << " background_features=" << context.background_features
         << " background_inliers=" << context.background_inliers
         << " reliable=" << int(quality.reliable)
         << " fallback_reason=" << (quality.reliable ? "NA" : quality.reason) << endl;
    mCoarseQualityCsv << std::setprecision(17) << mPendingRGBDB2Frame.frameId << ','
        << mPendingRGBDB2Frame.timestamp << ',' << quality.match_count << ',' << quality.inlier_count << ','
        << quality.inlier_ratio << ',' << quality.median_reprojection_error << ','
        << context.background_features << ',' << context.background_inliers << ','
        << int(quality.just_relocalized) << ',' << int(context.initialized) << ','
        << int(context.new_map) << ',' << int(context.relocalizing) << ','
        << int(context.just_recovered) << ',' << int(context.initial_success) << ','
        << int(context.finite_pose) << ',' << int(context.pure_vo) << ','
        << int(quality.reliable) << ',' << int(!quality.reliable) << ','
        << (quality.reliable ? "" : quality.reason) << ','
        << config.minCoarseInliers << ',' << config.minCoarseInlierRatio << '\n';
    mCoarseQualityCsv.flush();
    if (!mCoarseQualityCsv) throw std::runtime_error("Cannot write coarse pose quality CSV");
}

void Tracking::RunRGBDGeometryEvidence(bool poseReliable, const char* poseSource)
{
    if (!mbRGBDEvidenceReady || mbRGBDEvidenceProcessed)
        throw std::runtime_error("RGB-D evidence must run exactly once per input frame");
    if (mpDynamicDetector->Frontend().useCoarsePoseForB2 && mCoarsePoseQuality.reason.empty())
        FinalizeCoarsePoseQuality(false); // Initialization or Track early return.
    mbRGBDEvidenceProcessed = true;
    auto& b2Frame = mPendingRGBDB2Frame;
    const auto& coarsePolicySnapshot = mRGBDCoarsePolicySnapshot;
    const auto& frontend = mpDynamicDetector->Frontend();
    const double timestamp = mPendingRGBDB2Frame.timestamp;
    const auto temporalState = [](B4MotionState state) {
        return state == B4MotionState::STATIC ? TemporalState::STATIC :
            state == B4MotionState::DYNAMIC ? TemporalState::DYNAMIC : TemporalState::PENDING;
    };
    B2Result b2Result;
    bool hasB2Result = false;
    const bool verifyEvidence = bool(mpB2Processor);
    const auto mapPointsBeforeEvidence = verifyEvidence ?
        mCurrentFrame.mvpMapPoints : std::vector<MapPoint*>();
    const auto outliersBeforeEvidence = verifyEvidence ?
        mCurrentFrame.mvbOutlier : std::vector<bool>();
    const int featureCountBeforeEvidence = mCurrentFrame.N;
    const cv::Mat descriptorsBeforeEvidence = verifyEvidence ?
        mCurrentFrame.mDescriptors.clone() : cv::Mat();
    const bool hadPoseBeforeEvidence = verifyEvidence && mCurrentFrame.HasPose();
    Sophus::SE3f poseBeforeEvidence;
    if (hadPoseBeforeEvidence) poseBeforeEvidence = mCurrentFrame.GetPose();
    if (mpB2Processor)
    {
        b2Frame.poseSource = frontend.useCoarsePoseForB2 ? "coarse" : "final";
        // The saved input identity survives Atlas early returns.
        b2Frame.poseValid = poseReliable && mCurrentFrame.HasPose() &&
            mCurrentFrame.GetPose().matrix().allFinite();
        if (b2Frame.poseValid)
            b2Frame.Tcw = mCurrentFrame.GetPose();
        // Snapshot only Tracking-confirmed RGB-D inliers for B2's background
        // residual reference.  B2 filters raw person-mask pixels itself and
        // stores this data in its own previous-frame record.
        const size_t featureCount = std::min(mCurrentFrame.mvpMapPoints.size(),
            std::min(mCurrentFrame.mvbOutlier.size(),
                     std::min(mCurrentFrame.mvDepth.size(), mCurrentFrame.mvKeysUn.size())));
        b2Frame.backgroundObservations.reserve(featureCount);
        for (size_t i = 0; i < featureCount; ++i)
        {
            MapPoint* mapPoint = mCurrentFrame.mvpMapPoints[i];
            const float depth = mCurrentFrame.mvDepth[i];
            if (!mapPoint || mCurrentFrame.mvbOutlier[i] || !std::isfinite(depth) ||
                depth < mB2Config.minDepthMeters || depth > mB2Config.maxDepthMeters)
                continue;
            B2BackgroundObservation observation;
            observation.mapPointId = mapPoint->mnId;
            observation.pixel = mCurrentFrame.mvKeysUn[i].pt;
            observation.depthMeters = depth;
            b2Frame.backgroundObservations.push_back(observation);
        }
        const B2CandidateFrame* previous = mpB2Processor->HasPreviousFrame() ?
            &mpB2Processor->PreviousFrame() : nullptr;
        Sophus::SE3f relativePose;
        if (previous && previous->poseValid && b2Frame.poseValid)
            relativePose = b2Frame.Tcw * previous->Tcw.inverse();
        cout << "[B2 INPUT] frame=" << b2Frame.frameId
             << " previous_frame=" << (previous ? previous->frameId : -1)
             << " pose_source=" << b2Frame.poseSource
             << " current_pose_reliable=" << int(b2Frame.poseValid)
             << " pair_pose_reliable=" << int(previous && previous->poseValid && b2Frame.poseValid)
             << " explicit_relative=1" << endl;
        {
            DiagnosticTimer timer(mbCollectStageDiagnostics ? &mDiagnosticStages.b2Ms : nullptr);
            b2Result = mpB2Processor->Process(previous,b2Frame,relativePose,b2Frame.poseValid);
        }
        hasB2Result = true;
        const B2Result& result = b2Result;
        // Convert the immutable B2 result into a viewer-only snapshot. No B2
        // candidate is inserted into Frame::mvKeys or any SLAM container.
        mFrontendDiagnostics.b2OverlayEnabled = mB2Config.showOverlay;
        mFrontendDiagnostics.b2Points.clear();
        mFrontendDiagnostics.b2Candidates = 0;
        mFrontendDiagnostics.b2Matched = 0;
        mFrontendDiagnostics.b2Valid = 0;
        mFrontendDiagnostics.b2InstanceEvidenceSummaries.clear();
        mFrontendDiagnostics.b2ProcessingMs = result.processingMs;
        std::vector<float> b2RawResiduals, b2NormalizedResiduals;
        for (const B2PersonInstance& person : result.persons)
        {
            // These three totals deliberately use the same per-instance
            // denominators as B2's decision gate: queried candidates,
            // descriptor/mutual matches, then geometry-valid matches.
            mFrontendDiagnostics.b2Candidates += person.candidateCount;
            mFrontendDiagnostics.b2Matched += person.matchedCount;
            mFrontendDiagnostics.b2Valid += person.validGeometryCount;
            for (const B2CandidateFeature& feature : person.features)
            {
                const auto match = std::find_if(person.matches.begin(), person.matches.end(),
                    [&](const B2CandidateMatch& candidate) {
                        return candidate.currentCandidateId == feature.candidateId;
                    });
                B2ViewerPoint point;
                point.pixel = feature.keypoint.pt;
                if (feature.nearMaskBoundary || feature.nearDepthEdge)
                    point.pointClass = B2_VIEW_BOUNDARY;
                else if (match == person.matches.end() || !match->descriptorMatched)
                    point.pointClass = B2_VIEW_UNMATCHED;
                else
                {
                    const bool valid = match->geometryStatus == GEOMETRY_VALID &&
                        match->normalizedR3D >= 0.0f;
                    if (!valid)
                        point.pointClass = B2_VIEW_INVALID;
                    else
                    {
                        b2RawResiduals.push_back(match->rawR3D);
                        b2NormalizedResiduals.push_back(match->normalizedR3D);
                        point.pointClass = match->normalizedR3D >= mB2Config.highResidualNormThreshold ?
                            B2_VIEW_HIGH_RESIDUAL : B2_VIEW_LOW_RESIDUAL;
                    }
                }
                if (mFrontendDiagnostics.b2OverlayEnabled)
                    mFrontendDiagnostics.b2Points.push_back(point);
            }
        }
        const auto median = [](std::vector<float> values) {
            if (values.empty()) return -1.0f;
            const size_t middle = values.size() / 2;
            std::nth_element(values.begin(), values.begin() + middle, values.end());
            const float upper = values[middle];
            if (values.size() % 2) return upper;
            std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
            return 0.5f * (upper + values[middle - 1]);
        };
        mFrontendDiagnostics.b2MedianR3D = median(b2RawResiduals);
        mFrontendDiagnostics.b2MedianNormalizedR3D = median(b2NormalizedResiduals);
        if (result.persons.empty())
            mFrontendDiagnostics.b2EvidenceSummary = "B2: no person instance";
        else
        {
            std::ostringstream all;
            all << "B2 instances=" << result.persons.size();
            mFrontendDiagnostics.b2EvidenceSummary = all.str();
            for (const B2PersonInstance& person : result.persons)
            {
                const char* decision = "NOT_EVALUATED";
                switch (person.geometryDecision) {
                case GEOM_UNRELIABLE:
                    decision = "UNRELIABLE";
                    break;
                case GEOM_STATIC_CANDIDATE: decision = "STATIC_CANDIDATE"; break;
                case GEOM_MOVING_CANDIDATE: decision = "MOVING_CANDIDATE"; break;
                case GEOM_UNCERTAIN: decision = "UNCERTAIN"; break;
                default: break;
                }
                std::ostringstream summary;
                summary << "B2 ID=" << person.instanceId << " track=" << person.trackId
                        << ' ' << decision
                        << " c/m/v=" << person.candidateCount << '/' << person.matchedCount
                        << '/' << person.validGeometryCount
                        << " E=" << person.normalizedR3DMedian;
                mFrontendDiagnostics.b2InstanceEvidenceSummaries.push_back(summary.str());
            }
        }
    }
    {
        DiagnosticTimer timer(mbCollectStageDiagnostics && mpB3Fusion ? &mDiagnosticStages.b3Ms : nullptr);
        RunB3SemanticGeometryFusion(hasB2Result ? &b2Result : nullptr, timestamp);
    }
    if (frontend.useCoarsePoseForB2 && !mCoarsePoseQuality.reliable)
        for (const auto& result : mCurrentB3Results)
            if (result.fusion_reliable)
                throw std::runtime_error("Fallback evidence must not be reliable or advance B4");
    {
        DiagnosticTimer timer(mbCollectStageDiagnostics && mpB4StateMachine ? &mDiagnosticStages.b4Ms : nullptr);
        RunB4TemporalStateMachine(timestamp);
    }
    if (verifyEvidence && (mCurrentFrame.mvpMapPoints != mapPointsBeforeEvidence ||
        mCurrentFrame.mvbOutlier != outliersBeforeEvidence ||
        mCurrentFrame.N != featureCountBeforeEvidence ||
        mCurrentFrame.mDescriptors.size() != descriptorsBeforeEvidence.size() ||
        (!descriptorsBeforeEvidence.empty() &&
         cv::norm(mCurrentFrame.mDescriptors,descriptorsBeforeEvidence,cv::NORM_INF) != 0) ||
        mCurrentFrame.HasPose() != hadPoseBeforeEvidence ||
        (hadPoseBeforeEvidence &&
         std::memcmp(mCurrentFrame.GetPose().data(),poseBeforeEvidence.data(),
                     sizeof(float)*Sophus::SE3f::num_parameters) != 0)))
        throw std::runtime_error("EvidenceOnly changed Frame associations, descriptors, outliers or pose");
    if (mpB2Processor)
        cout << "[B2 READONLY] frame=" << b2Frame.frameId
             << " map_points_unchanged=1 frame_pose_unchanged=1 descriptors_unchanged=1"
             << " applied_action=NONE_EVIDENCE_ONLY" << endl;
    if (mCurrentFrame.mbSemanticAnnotated)
    {
        std::unordered_map<int, TemporalState> currentStates;
        for (const auto& result : mCurrentB4Results)
            if (!result.isLost) currentStates[result.trackId] = temporalState(result.state);
        UpdateFeatureTemporalStates(mCurrentFrame, currentStates);
        if (frontend.featurePolicy.enable)
        {
            const auto statistics = SummarizeFeaturePolicies(mCurrentFrame,coarsePolicySnapshot,
                frontend.coarseFilterEnable);
            if (mFeaturePolicySummaryCsv.is_open())
            {
                WriteFeaturePolicyCsv(mFeaturePolicySummaryCsv,mCurrentFrame,statistics,frontend.coarseFilterEnable);
                mFeaturePolicySummaryCsv.flush();
                if (!mFeaturePolicySummaryCsv)
                    throw std::runtime_error("Cannot flush FeaturePolicy summary CSV");
            }
            if (frontend.featurePolicy.printFrameSummary)
                cout << "[FEATURE POLICY] frame_id=" << mCurrentFrame.mnId
                     << " coarse_allowed=" << statistics.coarseAllowed
                     << " fine_allowed=" << statistics.fineAllowed
                     << " map_allowed=" << statistics.mapAllowed
                     << " applied=" << int(std::count(mCurrentFrame.mvDynamicExcluded.begin(),mCurrentFrame.mvDynamicExcluded.end(),true)>0)
                     << " actual_excluded="
                     << std::count(mCurrentFrame.mvDynamicExcluded.begin(),mCurrentFrame.mvDynamicExcluded.end(),true) << endl;
        }
        AssertRGBDFeatureAlignment(mCurrentFrame);
        int counts[4] = {0,0,0,0};
        for (int i = 0; i < mCurrentFrame.N; ++i)
        {
            ++counts[static_cast<int>(mCurrentFrame.mvSemanticRegion[i])];
            if (!frontend.coarseFilterEnable && (mCurrentFrame.mvFeaturePolicy[i] != FeatureUsePolicy::USE_NORMAL ||
                mCurrentFrame.mvDynamicExcluded[i]))
                throw std::runtime_error("Annotation-only stage must not apply feature exclusion");
        }
        cout << "[FEATURE SEMANTICS] frame_id=" << mCurrentFrame.mnId << " N=" << mCurrentFrame.N
             << " background=" << counts[0] << " person_interior=" << counts[1]
             << " person_boundary=" << counts[2] << " unknown=" << counts[3]
             << " normal=" << mCurrentFrame.N-std::count(mCurrentFrame.mvDynamicExcluded.begin(),mCurrentFrame.mvDynamicExcluded.end(),true)
             << " excluded=" << std::count(mCurrentFrame.mvDynamicExcluded.begin(),mCurrentFrame.mvDynamicExcluded.end(),true)
             << " aligned=1" << endl;
    }
    if (frontend.useCoarsePoseForB2)
        cout << "[TWO STAGE] frame=" << mPendingRGBDB2Frame.frameId
             << " event=evidence_complete pose_source=" << poseSource
             << " pose_reliable=" << int(poseReliable && mCurrentFrame.HasPose())
             << " b4_updates=1 applied=0" << endl;
}

void Tracking::RunB3SemanticGeometryFusion(const B2Result* b2Result, double timestamp)
{
    // A missed YOLO/B2 result must not leave B3/B4 consuming the prior frame.
    mCurrentB3Results.clear();
    if (!mpB3Fusion)
        return;
    const std::vector<YOLOSegInstance>& semanticInstances =
        mpDynamicDetector->LastRGBDInstances();
    // instance_id is the same-frame semantic/B2 join key.  track_id is the
    // persistent B2 association key and is indexed as well for the next B3
    // temporal stage, where a semantic tracker can supply a track ID first.
    std::unordered_map<int, const B2PersonInstance*> geometryByInstance;
    std::unordered_map<int, const B2PersonInstance*> geometryByTrack;
    if (b2Result)
        for (const B2PersonInstance& person : b2Result->persons)
        {
            geometryByInstance[person.instanceId] = &person;
            if (person.trackId >= 0)
                geometryByTrack[person.trackId] = &person;
        }
    for (size_t instanceIndex = 0; instanceIndex < semanticInstances.size(); ++instanceIndex)
    {
        const YOLOSegInstance& source = semanticInstances[instanceIndex];
        SemanticInstance semantic;
        semantic.frameId = mbRGBDEvidenceReady ? mPendingRGBDB2Frame.frameId : mCurrentFrame.mnId;
        semantic.timestamp = timestamp;
        semantic.instanceId = static_cast<int>(instanceIndex);
        semantic.trackId = source.trackId;
        semantic.classId = source.classId;
        semantic.className = source.classId == 0 ? "person" :
            "class_" + std::to_string(source.classId);
        semantic.confidence = source.confidence;
        semantic.boundingBox = source.boundingBox;
        const B2PersonInstance* geometry = nullptr;
        if (semantic.trackId >= 0)
        {
            const auto trackIt = geometryByTrack.find(semantic.trackId);
            if (trackIt != geometryByTrack.end())
                geometry = trackIt->second;
        }
        if (!geometry)
        {
            const auto instanceIt = geometryByInstance.find(static_cast<int>(instanceIndex));
            if (instanceIt != geometryByInstance.end())
                geometry = instanceIt->second;
        }
        if (geometry && geometry->trackId != semantic.trackId)
            throw std::runtime_error("Shared instance track_id mismatch between YOLO and B2");
        mCurrentB3Results.push_back(mpB3Fusion->Fuse(semantic, geometry));
    }
}

void Tracking::RunB4TemporalStateMachine(double timestamp)
{
    mCurrentB4Results.clear();
    if (!mpB4StateMachine)
        return;
    const float imageDiagonal = std::sqrt(float(mImGray.cols * mImGray.cols +
                                                mImGray.rows * mImGray.rows));
    mCurrentB4Results = mpB4StateMachine->Process(
        mbRGBDEvidenceReady ? mPendingRGBDB2Frame.frameId : mCurrentFrame.mnId, timestamp,
                                                   mCurrentB3Results, imageDiagonal);
}

void Tracking::UpdateDynamicLogs()
{
    if (!mpB3Fusion)
        return;
    // b3Instances is a read-only visual container shared by B3 and B4.
    mFrontendDiagnostics.b3OverlayEnabled = mB3Config.showOverlay ||
        (mB4Config.showOverlay && mpB4StateMachine);
    mFrontendDiagnostics.b3InstanceEvidenceSummaries.clear();
    mFrontendDiagnostics.b3Instances.clear();
    mFrontendDiagnostics.b4OverlayEnabled = mB4Config.showOverlay && mpB4StateMachine;
    mFrontendDiagnostics.b4EvidenceSummary = "B4 tracks=" +
        std::to_string(mCurrentB4Results.size());
    mFrontendDiagnostics.b3EvidenceSummary = "B3 instances=" +
        std::to_string(mCurrentB3Results.size());
    for (const B3FusionResult& result : mCurrentB3Results)
    {
        std::ostringstream summary;
        summary << "B3 ID=" << result.instance_id << " track=" << result.track_id
                << " fusion=" << B3FusionCategoryName(result.fusion_category)
                << " reason=" << result.fusion_reason
                << " action=" << result.applied_action;
        mFrontendDiagnostics.b3InstanceEvidenceSummaries.push_back(summary.str());
        B3ViewerInstance viewerInstance;
        viewerInstance.boundingBox = result.bounding_box;
        if (!result.fusion_reliable)
        {
            viewerInstance.color = cv::Scalar(128, 128, 128); // gray
            viewerInstance.primaryLabel = "ID" + std::to_string(result.instance_id) + " UNRELIABLE";
            viewerInstance.secondaryLabel = "NO DECISION";
        }
        else if (result.fusion_category == B3_SEM_DYNAMIC_GEOM_MOVING)
        {
            viewerInstance.color = cv::Scalar(0, 0, 255); // red, BGR
            viewerInstance.primaryLabel = "ID" + std::to_string(result.instance_id) + " PERSON+MOVING";
            viewerInstance.secondaryLabel = "B3 DYNAMIC CANDIDATE";
        }
        else if (result.fusion_category == B3_SEM_DYNAMIC_GEOM_STATIC)
        {
            viewerInstance.color = cv::Scalar(0, 255, 255); // yellow, BGR
            viewerInstance.primaryLabel = "ID" + std::to_string(result.instance_id) + " PERSON+STATIC";
            viewerInstance.secondaryLabel = "B3 DEFER";
        }
        else if (result.fusion_category == B3_SEM_STATIC_GEOM_MOVING)
        {
            viewerInstance.color = cv::Scalar(255, 0, 255); // purple, BGR
            viewerInstance.primaryLabel = "ID" + std::to_string(result.instance_id) + " NONPERSON+MOVING";
            viewerInstance.secondaryLabel = "B3 RECORD ONLY";
        }
        else
        {
            viewerInstance.color = cv::Scalar(0, 255, 0); // green, BGR
            viewerInstance.primaryLabel = "ID" + std::to_string(result.instance_id) + " NONPERSON+STATIC";
            viewerInstance.secondaryLabel = "B3 USE NORMALLY";
        }
        viewerInstance.tertiaryLabel = "fusion=" +
            std::string(B3FusionCategoryName(result.fusion_category)) +
            " reason=" + result.fusion_reason;
        const auto b4 = std::find_if(mCurrentB4Results.begin(), mCurrentB4Results.end(),
            [&](const B4StateResult& item) { return item.instanceId == result.instance_id; });
        if (b4 != mCurrentB4Results.end() && !b4->isLost && mB4Config.showOverlay)
        {
            // B4 is the authoritative viewer state once enabled. This is a
            // copy for drawing only; it does not feed any tracking decision.
            if (b4->state == B4MotionState::STATIC)
                viewerInstance.color = cv::Scalar(0, 255, 0);       // green
            else if (b4->state == B4MotionState::PENDING)
                viewerInstance.color = cv::Scalar(0, 255, 255);     // yellow
            else
                viewerInstance.color = cv::Scalar(0, 0, 255);       // red
            viewerInstance.primaryLabel = "ID" + std::to_string(b4->trackId) +
                " " + B4MotionStateName(b4->state);
            viewerInstance.secondaryLabel = "Ns/Nd/Nr=" + std::to_string(b4->nStatic) + "/" +
                std::to_string(b4->nDynamic) + "/" + std::to_string(b4->nRecovery);
            viewerInstance.tertiaryLabel = "reason=" + b4->reasonCode;
            viewerInstance.quaternaryLabel = "B4 " + std::string(B4MotionStateName(b4->previousState)) +
                "->" + B4MotionStateName(b4->state) +
                " evidence=" + B4EvidenceTypeName(b4->evidenceType);
        }
        mFrontendDiagnostics.b3Instances.push_back(std::move(viewerInstance));
        if (mB3Config.printFrameSummary)
            std::cout << "[B3] frame=" << result.frame_id
                      << " track=" << result.track_id
                      << " inst=" << result.instance_id
                      << " sem=" << result.semantic_class_name
                      << " sem_dynamic=" << int(result.semantic_dynamic_prior)
                      << " geom=" << GeometryStateName(result.geometry_state)
                      << " reliable=" << int(result.geometry_reliable)
                      << " raw=" << std::fixed << std::setprecision(4) << result.raw_r3d_median
                      << " norm=" << result.norm_r3d_median
                      << " high=" << result.high_ratio
                      << " fusion=" << B3FusionCategoryName(result.fusion_category)
                      << " recommend=" << result.recommended_action
                      << " applied=" << result.applied_action
                      << " reason=" << result.fusion_reason << std::endl;
        if (mB3SummaryCsv)
            mB3SummaryCsv << result.frame_id << ',' << std::setprecision(17) << result.timestamp << ','
                          << result.instance_id << ',' << result.track_id << ','
                          << result.semantic_class_id << ',' << result.semantic_class_name << ','
                          << result.semantic_confidence << ',' << int(result.semantic_dynamic_prior) << ','
                          << int(result.geometry_reliable) << ',' << int(result.geometry_state) << ','
                          << result.geometry_reason << ',' << result.candidate_count << ','
                          << result.matched_count << ',' << result.valid_count << ','
                          << result.match_ratio << ',' << result.valid_match_ratio << ','
                          << result.raw_r3d_median << ',' << result.norm_r3d_median << ','
                          << result.high_ratio << ',' << result.bounding_box.x << ','
                          << result.bounding_box.y << ',' << result.bounding_box.width << ','
                          << result.bounding_box.height << ','
                          << B3FusionCategoryName(result.fusion_category) << ','
                          << int(result.fusion_reliable) << ',' << result.fusion_reason << ','
                          << result.recommended_action << ',' << result.applied_action << '\n';
    }
    // Tracks that YOLO did not detect in this frame are still visible through
    // their TTL. They are drawn gray from the B4-owned last bbox snapshot.
    if (mB4Config.showOverlay && mpB4StateMachine)
        for (const B4StateResult& b4 : mCurrentB4Results)
        {
            if (!b4.isLost || b4.missedFrames > mB4Config.trackTTL)
                continue;
            B3ViewerInstance lost;
            lost.boundingBox = b4.boundingBox;
            lost.color = cv::Scalar(128, 128, 128); // gray
            lost.primaryLabel = "ID" + std::to_string(b4.trackId) + " LOST";
            lost.secondaryLabel = "state=" + std::string(B4MotionStateName(b4.state)) +
                " missed=" + std::to_string(b4.missedFrames) + "/" +
                std::to_string(mB4Config.trackTTL);
            lost.tertiaryLabel = "Ns/Nd/Nr=" + std::to_string(b4.nStatic) + "/" +
                std::to_string(b4.nDynamic) + "/" + std::to_string(b4.nRecovery);
            lost.quaternaryLabel = "reason=" + b4.reasonCode;
            mFrontendDiagnostics.b3Instances.push_back(std::move(lost));
        }
    if (mB3SummaryCsv)
        mB3SummaryCsv.flush();
    for (const B4StateResult& result : mCurrentB4Results)
    {
        if (mB4Config.printFrameSummary)
            std::cout << "[B4] frame=" << result.frameId
                      << " track=" << result.trackId
                      << " prev=" << B4MotionStateName(result.previousState)
                      << " curr=" << B4MotionStateName(result.state)
                      << " evidence=" << (result.evidenceType == B4EvidenceType::RELIABLE_STATIC ? "STATIC" :
                                             result.evidenceType == B4EvidenceType::RELIABLE_MOVING ? "MOVING" : "UNRELIABLE")
                      << " Ns=" << result.nStatic << " Nd=" << result.nDynamic
                      << " Nr=" << result.nRecovery
                      << " evidence_age_seconds=" << (std::isfinite(result.evidenceAgeSeconds) ?
                          std::to_string(result.evidenceAgeSeconds) : "NA")
                      << " counter_expired=" << result.counterExpired
                      << " counters_reset=" << result.countersReset
                      << " static_state_expired=" << result.staticStateExpired
                      << " expiry_reason=" << result.expiryReason
                      << " expiry_prev=" << B4MotionStateName(result.previousState)
                      << " expiry_curr=" << B4MotionStateName(result.stateAfterExpiry)
                      << " reason=" << result.reasonCode
                      << " recommend=" << result.recommendedPolicy
                      << " applied=" << result.appliedPolicy << std::endl;
        if (mB4SummaryCsv)
            mB4SummaryCsv << std::setprecision(17) << result.timestamp << ',' << result.frameId << ','
                          << result.instanceId << ',' << result.trackId << ','
                          << B3FusionCategoryName(result.fusionCategory) << ',' << int(result.b3Reliable) << ','
                          << result.b3Reason << ',' << B4MotionStateName(result.previousState) << ','
                          << B4MotionStateName(result.state) << ',' << result.nStatic << ','
                          << result.nDynamic << ',' << result.nRecovery << ',' << result.nUnreliable << ','
                          << result.missedFrames << ',' << result.ageFrames << ',' << result.reasonCode << ','
                          << result.recommendedPolicy << ',' << result.appliedPolicy << ','
                          << int(result.evidenceOnly) << ',' << int(result.hasReliableHistory) << ',';
        if (mB4SummaryCsv) {
            if (std::isfinite(result.evidenceAgeSeconds)) mB4SummaryCsv << result.evidenceAgeSeconds;
            mB4SummaryCsv << ',' << int(result.counterExpired) << ',' << int(result.countersReset) << ','
                          << int(result.staticStateExpired) << ',' << result.expiryReason << ','
                          << B4MotionStateName(result.previousState) << ','
                          << B4MotionStateName(result.stateAfterExpiry) << ',' << result.reasonCode << '\n';
        }
    }
    if (mB4SummaryCsv)
        mB4SummaryCsv.flush();
}

void Tracking::EmitFrontendDiagnostics()
{
    FrontendDiagnostics &d=mFrontendDiagnostics;
    d.frameId=mCurrentFrame.mnId;
    d.timestamp=mCurrentFrame.mTimeStamp;
    d.yoloInferenceMs=mpDynamicDetector ? mpDynamicDetector->LastInferenceMs() : 0.0;
    d.rawFastCandidates=mCurrentFrame.mnRawFastCandidatesLeft+mCurrentFrame.mnRawFastCandidatesRight;
    d.maskedRejected=mCurrentFrame.mnMaskedRejectedLeft+mCurrentFrame.mnMaskedRejectedRight;
    d.finalOrbFeatures=mCurrentFrame.N;
    d.poseInliers=mnMatchesInliers;
    d.trackingState=int(mState);
    int maskPixels=0, imagePixels=0;
    for(const cv::Mat* mask : {&mCurrentFrame.mDynamicMaskLeft,&mCurrentFrame.mDynamicMaskRight}) {
        if(!mask->empty()) {
            maskPixels+=cv::countNonZero(*mask);
            imagePixels+=mask->rows*mask->cols;
        }
    }
    d.personMaskRatio=imagePixels ? double(maskPixels)/imagePixels : 0.0;
    d.matches=0;
    for(int i=0;i<mCurrentFrame.N;i++)
        if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) ++d.matches;

    // Refresh after possible keyframe insertion so GUI and terminal agree.
    CollectDynamicDiagnostics();
    mpFrameDrawer->Update(this);
    if(mSensor==System::RGBD && mpDynamicDetector->Frontend().diagnosticSaveOverlay && d.frameId%100==0) {
        const std::string folder=mB2Config.csvOutputParent+"/dynamic_frontend_overlays";
        // Directory creation is handled in the read-only snapshot collector.
        const std::string path=folder+"/frame_"+std::to_string(d.frameId)+".png";
        cv::Mat rendered;
        if(d.stateActionOverlayEnabled) rendered=mpFrameDrawer->DrawFrame();
        else {
            // Save-only must not toggle shared Viewer flags, even briefly.
            // Render the SAME pure overlay helper on a private image instead.
            cv::cvtColor(mImGray,rendered,cv::COLOR_GRAY2BGR);
            cv::copyMakeBorder(rendered,rendered,0,70,0,0,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
            FrameDrawer::DrawStateActionOverlay(rendered,mCurrentFrame.mvKeys,
                mCurrentFrame.mvSemanticRegion,d);
        }
        if(std::ifstream(path).good() || !cv::imwrite(path,rendered))
            throw std::runtime_error("Cannot save new diagnostic overlay: "+path);
    }
    cout << "[FRONTEND] frame_id=" << d.frameId
         << " timestamp=" << fixed << setprecision(6) << d.timestamp
         << " yolo_ms=" << setprecision(3) << d.yoloInferenceMs
         << " person_mask_ratio=" << setprecision(4) << d.personMaskRatio
         << " raw_fast=" << d.rawFastCandidates
         << " mask_rejected=" << d.maskedRejected
         << " final_orb=" << d.finalOrbFeatures
         << " matches=" << d.matches
         << " pose_inliers=" << d.poseInliers
         << " state=" << d.trackingState
         << " keyframe=" << (d.becameKeyFrame ? 1 : 0) << endl;
}

Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
{
    mImGray = im;
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if (mSensor == System::MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    else if(mSensor == System::IMU_MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }

    if (mState==NO_IMAGES_YET)
        t0=timestamp;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    lastID = mCurrentFrame.mnId;
    Track();

    return mCurrentFrame.GetPose();
}


void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

void Tracking::PreintegrateIMU()
{

    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    if(mlQueueImuData.size() == 0)
    {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    while(true)
    {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if(!mlQueueImuData.empty())
            {
                IMU::Point* m = &mlQueueImuData.front();
                cout.precision(17);
                if(m->t<mCurrentFrame.mpPrevFrame->mTimeStamp-mImuPer)
                {
                    mlQueueImuData.pop_front();
                }
                else if(m->t<mCurrentFrame.mTimeStamp-mImuPer)
                {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                }
                else
                {
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            }
            else
            {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500);
    }

    const int n = mvImuFromLastFrame.size()-1;
    if(n==0){
        cout << "Empty IMU measurements vector!!!\n";
        return;
    }

    IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib);

    for(int i=0; i<n; i++)
    {
        float tstep;
        Eigen::Vector3f acc, angVel;
        if((i==0) && (i<(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tini = mvImuFromLastFrame[i].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tini/tab))*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        else if(i<(n-1))
        {
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
        }
        else if((i>0) && (i==(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
            tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t;
        }
        else if((i==0) && (i==(n-1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc,angVel,tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();

    //Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
}


bool Tracking::PredictStateIMU()
{
    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if(mbMapUpdated && mpLastKeyFrame)
    {
        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else if(!mbMapUpdated)
    {
        const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

        mCurrentFrame.mImuBias = mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}

void Tracking::ResetFrameIMU()
{
    // TODO To implement...
}


void Tracking::Track()
{

    mFrontendDiagnostics = FrontendDiagnostics();
    // A YOLO miss or an absent B2 result must never leave prior-frame B3
    // instances visible to the next frame.
    mCurrentB3Results.clear();
    mnMatchesInliers = 0;

    if (bStepByStep)
    {
        std::cout << "Tracking: Waiting to the next step" << std::endl;
        while(!mbStep && bStepByStep)
            usleep(500);
        mbStep = false;
    }

    if(mpLocalMapper->mbBadImu)
    {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }

    Map* pCurrentMap = mpAtlas->GetCurrentMap();
    if(!pCurrentMap)
    {
        cout << "ERROR: There is not an active map in the atlas" << endl;
    }

    if(mState!=NO_IMAGES_YET)
    {
        if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
        {
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        }
        else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
        {
            // cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp << endl;
            // cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
            if(mpAtlas->isInertial())
            {

                if(mpAtlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    if(!pCurrentMap->GetIniertialBA2())
                    {
                        mpSystem->ResetActiveMap();
                    }
                    else
                    {
                        CreateMapInAtlas();
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }
                return;
            }

        }
    }


    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mbCreatedMap)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPreIMU = std::chrono::steady_clock::now();
#endif
        PreintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPreIMU = std::chrono::steady_clock::now();

        double timePreImu = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreIMU - time_StartPreIMU).count();
        vdIMUInteg_ms.push_back(timePreImu);
#endif

    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }


    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD)
        {
            StereoInitialization();
        }
        else
        {
            MonocularInitialization();
        }

        //mpFrameDrawer->Update(this);

        if(mState!=OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if(mpAtlas->GetAllMaps().size() == 1)
        {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;
        DiagnosticTimer coarseTimer(mbRGBDEvidenceReady && mbCollectStageDiagnostics ? &mDiagnosticStages.coarseMs : nullptr);

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPosePred = std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if(mState==OK)
            {

                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if((!mbVelocity && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }


                if (!bOK)
                {
                    if ( mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) &&
                         (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
                    {
                        mState = LOST;
                    }
                    else if(pCurrentMap->KeyFramesInMap()>10)
                    {
                        // cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                    }
                    else
                    {
                        mState = LOST;
                    }
                }
            }
            else
            {

                if (mState == RECENTLY_LOST)
                {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))
                    {
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        if (mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                    else
                    {
                        // Relocalization
                        bOK = Relocalization();
                        //std::cout << "mCurrentFrame.mTimeStamp:" << to_string(mCurrentFrame.mTimeStamp) << std::endl;
                        //std::cout << "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
                        if(mCurrentFrame.mTimeStamp-mTimeStampLost>3.0f && !bOK)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                }
                else if (mState == LOST)
                {

                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->KeyFramesInMap()<10)
                    {
                        mpSystem->ResetActiveMap();
                        Verbose::PrintMess("Reseting current map...", Verbose::VERBOSITY_NORMAL);
                    }else
                        CreateMapInAtlas();

                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }

        }
        else
        {
            // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            if(mState==LOST)
            {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    if(mbVelocity)
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    Sophus::SE3f TcwMM;
                    CoarsePoseQuality coarseQualityMM;
                    if(mbVelocity)
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.GetPose();
                        coarseQualityMM = mCoarsePoseQuality;
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCoarsePoseQuality = coarseQualityMM;
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPosePred = std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPosePred - time_StartPosePred).count();
        vdPosePred_ms.push_back(timePosePred);
#endif


#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartLMTrack = std::chrono::steady_clock::now();
#endif
        coarseTimer.Stop();
        // All original initial-tracking branches converge here. Evidence reads
        // a coarse-pose/inlier snapshot and never changes a SLAM decision.
        const bool coarseEvidence = mbRGBDEvidenceReady &&
            mpDynamicDetector->Frontend().useCoarsePoseForB2;
        if (coarseEvidence)
        {
            const bool poseFinite = mCurrentFrame.HasPose() &&
                mCurrentFrame.GetPose().matrix().allFinite();
            FinalizeCoarsePoseQuality(bOK);
            const bool coarseReliable = mCoarsePoseQuality.reliable;
            cout << "[TWO STAGE] frame=" << mCurrentFrame.mnId
                 << " event=coarse_boundary initial_success=" << int(bOK)
                 << " pose_finite=" << int(poseFinite)
                 << " pose_reliable=" << int(coarseReliable) << endl;
            RunRGBDGeometryEvidence(coarseReliable, "coarse");
        }
        // First pass: CURRENT B4 states clear stale coarse-stage P/D matches.
        // The second pass after SearchLocalPoints prevents their reintroduction.
        if(coarseEvidence && mpDynamicDetector->Frontend().finePolicyEnable && bOK &&
            (!mbOnlyTracking || !mbVO))
            ApplyFineFeaturePolicy("pre_local_map");
#ifdef REGISTER_TIMES
        // Keep the existing local-map timing metric about local-map tracking,
        // not the newly inserted B2/B3/B4 diagnostic work.
        time_StartLMTrack = std::chrono::steady_clock::now();
#endif
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
                bOK = TrackLocalMap();
                mbTwoStageFineCompleted = true;
            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
            {
                bOK = TrackLocalMap();
                mbTwoStageFineCompleted = true;
            }
        }
        if(KeyFrameStageGuardEnabled()) RecordKeyFrameStage("final_snapshot");
        if (coarseEvidence)
            cout << "[TWO STAGE] frame=" << mCurrentFrame.mnId
                 << " event=fine_complete attempted=" << int(mbTwoStageFineCompleted)
                 << " success=" << int(bOK) << endl;

        if(bOK)
            mState = OK;
        else if (mState == OK)
        {
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                {
                    cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }

                mState=RECENTLY_LOST;
            }
            else
                mState=RECENTLY_LOST; // visual to lost

            /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
                mTimeStampLost = mCurrentFrame.mTimeStamp;
            //}
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        if((mCurrentFrame.mnId<(mnLastRelocFrameId+mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) &&
           (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }

        if(pCurrentMap->isImuInitialized())
        {
            if(bOK)
            {
                if(mCurrentFrame.mnId==(mnLastRelocFrameId+mnFramesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    ResetFrameIMU();
                }
                else if(mCurrentFrame.mnId>(mnLastRelocFrameId+30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndLMTrack = std::chrono::steady_clock::now();

        double timeLMTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLMTrack - time_StartLMTrack).count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

        // Update drawer
        mpFrameDrawer->Update(this);
        if(mCurrentFrame.isSet())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        if(bOK || mState==RECENTLY_LOST)
        {
            // Update motion model
            if(mLastFrame.isSet() && mCurrentFrame.isSet())
            {
                Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
                mVelocity = mCurrentFrame.GetPose() * LastTwc;
                mbVelocity = true;
            }
            else {
                mbVelocity = false;
            }

            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();
#endif
            bool bNeedKF = NeedNewKeyFrame();
            if (coarseEvidence)
                cout << "[TWO STAGE] frame=" << mCurrentFrame.mnId
                     << " event=keyframe_decision fine_completed=" << int(mbTwoStageFineCompleted)
                     << " needed=" << int(bNeedKF) << endl;

            // Check if we need to insert a new keyframe
            // if(bNeedKF && bOK)
            if(bNeedKF && (bOK || (mInsertKFsLost && mState==RECENTLY_LOST &&
                                   (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))))
            {
                mFrontendDiagnostics.becameKeyFrame = CreateNewKeyFrame();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();

            double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndNewKF - time_StartNewKF).count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(pCurrentMap->KeyFramesInMap()<=10)
            {
                mpSystem->ResetActiveMap();
                return;
            }
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                if (!pCurrentMap->isImuInitialized())
                {
                    Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }

            CreateMapInAtlas();

            return;
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }




    if(mState==OK || mState==RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if(mCurrentFrame.isSet())
        {
            Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }

    }

#ifdef REGISTER_LOOP
    if (Stop()) {

        // Safe area to stop
        while(isStopped())
        {
            usleep(3000);
        }
    }
#endif
}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
            {
                cout << "not IMU meas" << endl;
                return;
            }

            if (!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA).norm()<0.5)
            {
                cout << "not enough acceleration" << endl;
                return;
            }

            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
        }
        else
            mCurrentFrame.SetPose(Sophus::SE3f());

        // Bootstrap is also a persistent-map creation path. New persons remain P.
        if(mbRGBDEvidenceReady && mpDynamicDetector->Frontend().mapAdmissionEnable)
            PrepareFrameMapAdmission(mCurrentFrame,mMapAdmissionLogPath);
        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
        MapAdmissionBatch admission(pKFini,"rgbd_bootstrap");

        // Insert KeyFrame in the map
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if(!mpCamera2){
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    if(!admission.Check(i)) continue;
                    Eigen::Vector3f x3D;
                    mCurrentFrame.UnprojectStereo(i, x3D);
                    MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKFini,i);
                    pKFini->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    admission.Created(i);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }
        } else{
            for(int i = 0; i < mCurrentFrame.Nleft; i++){
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1){
                    Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i);
                    pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP,i);
                    pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
                }
            }
        }

        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

        //cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

        if (mbRGBDEvidenceReady && mpDynamicDetector->Frontend().useCoarsePoseForB2)
            cout << "[TWO STAGE] frame=" << mCurrentFrame.mnId
                 << " event=bootstrap_keyframe reason=INITIAL_MAP_REQUIRED" << endl;
        mpLocalMapper->InsertKeyFrame(pKFini);
        // The initialization frame is also a real KeyFrame, even though it
        // does not pass through NeedNewKeyFrame()/CreateNewKeyFrame().
        mFrontendDiagnostics.becameKeyFrame = true;

        if(KeyFrameStageGuardEnabled()) {
            mKeyFrameDecision.bootstrap=true;
            mKeyFrameDecision.created=true;
            RecordKeyFrameStage("bootstrap");
        }

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        //mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        mState=OK;
    }
}


void Tracking::MonocularInitialization()
{

    if(!mbReadyToInitializate)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            if (mSensor == System::IMU_MONOCULAR)
            {
                if(mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            }

            mbReadyToInitializate = true;

            return;
        }
    }
    else
    {
        if (((int)mCurrentFrame.mvKeys.size()<=100)||((mSensor == System::IMU_MONOCULAR)&&(mLastFrame.mTimeStamp-mInitialFrame.mTimeStamp>1.0)))
        {
            mbReadyToInitializate = false;

            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            mbReadyToInitializate = false;
            return;
        }

        Sophus::SE3f Tcw;
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Tcw,mvIniP3D,vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(Sophus::SE3f());
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}



void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpAtlas->AddMapPoint(pMP);
    }


    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f/medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
    }


    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    //mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

    Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
    mbVelocity = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;

    initID = pKFcur->mnId;
}


void Tracking::CreateMapInAtlas()
{
    if (mpB4StateMachine && mB4Config.evidenceExpiryEnable) {
        mpB4StateMachine->Reset();
        mCurrentB4Results.clear();
    }
    if (mbRGBDEvidenceReady && mpDynamicDetector->Frontend().useCoarsePoseForB2)
        mCoarsePoseContext.new_map = true;
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mbSetInit=false;

    mnInitialFrameId = mCurrentFrame.mnId+1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mbVelocity = false;
    //mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
    Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId+1), Verbose::VERBOSITY_NORMAL);
    mbVO = false; // Init value for know if there are enough MapPoints in the last KF
    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        mbReadyToInitializate = false;
    }

    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpImuPreintegratedFromLastKF)
    {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
    }

    if(mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

    if(mpReferenceKF)
        mpReferenceKF = static_cast<KeyFrame*>(NULL);

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
    {
        cout << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.GetPose());

    //mCurrentFrame.PrintPointDistribution();


    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
    OptimizeNormalInitialPose(nmatches,"reference_keyframe");

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        //if(i >= mCurrentFrame.Nleft) break;
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1? mLastFrame.N : mLastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for(int i=0; i<Nfeat;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
            bCreateNew = true;

        if(bCreateNew)
        {
            Eigen::Vector3f x3D;

            if(mLastFrame.Nleft == -1){
                mLastFrame.UnprojectStereo(i, x3D);
            }
            else{
                x3D = mLastFrame.UnprojectStereoFishEye(i);
            }

            MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);
            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;

    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    {
        // Predict state with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();
        return true;
    }
    else
    {
        mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
    }




    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;

    if(mSensor==System::STEREO)
        th=7;
    else
        th=15;

    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);

    }

    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            return true;
        else
            return false;
    }

    // Apply previous-state policy only here, after the original matching search.
    OptimizeNormalInitialPose(nmatches,"motion_model");

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    DiagnosticTimer fineTimer(mbRGBDEvidenceReady && mbCollectStageDiagnostics ? &mDiagnosticStages.fineMs : nullptr);

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;

    UpdateLocalMap();
    SearchLocalPoints();

    const bool finePolicy=mbRGBDEvidenceReady && mpDynamicDetector->Frontend().finePolicyEnable;
    const int requiredSupport=finePolicy ? FineStaticSupportRequired(mCurrentFrame.mnId,
        mnLastRelocFrameId,mMaxFrames,mState==RECENTLY_LOST) : 0;
    if(finePolicy)
    {
        ApplyFineFeaturePolicy("pre_pose_optimization");
        int staticSupport=0;
        for(int i=0;i<mCurrentFrame.N;++i)
            if(mCurrentFrame.mvpMapPoints[i] &&
                (mbOnlyTracking || mCurrentFrame.mvpMapPoints[i]->Observations()>0)) ++staticSupport;
        if(!FinePoseSupported(staticSupport,requiredSupport,true))
        {
            mnMatchesInliers=0;
            mpLocalMapper->mnMatchesInliers=0;
            ReportFineFeaturePolicy("result",0,true,requiredSupport);
            return false; // No P/D restoration. Original Track loss/relocalization path follows.
        }
        mbFineOptimizationExecuted=true;
    }

    // TOO check outliers before PO
    int aux1 = 0, aux2=0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    if(finePolicy) VerifyFineOptimizerInputs(mCurrentFrame);
    int inliers;
    if (!mpAtlas->isImuInitialized())
        Optimizer::PoseOptimization(&mCurrentFrame);
    else
    {
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        }
        else
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if(!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
            else
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
    if(finePolicy)
    {
        const bool success=FinePoseSupported(mnMatchesInliers,requiredSupport,
            mCurrentFrame.HasPose() && mCurrentFrame.GetPose().matrix().allFinite());
        ReportFineFeaturePolicy("result",mnMatchesInliers,!success,requiredSupport);
        return success;
    }
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
        return true;


    if (mSensor == System::IMU_MONOCULAR)
    {
        if((mnMatchesInliers<15 && mpAtlas->isImuInitialized())||(mnMatchesInliers<50 && !mpAtlas->isImuInitialized()))
        {
            return false;
        }
        else
            return true;
    }
    else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else
    {
        if(mnMatchesInliers<30)
            return false;
        else
            return true;
    }
}

bool Tracking::KeyFrameStageGuardEnabled() const
{
    return mbRGBDEvidenceReady && mpDynamicDetector && mpDynamicDetector->Frontend().finePolicyEnable;
}

KeyFrameGateResult Tracking::CurrentKeyFrameGate(bool requireNeedDecision)
{
    auto& c=mKeyFrameDecision;
    c.active=KeyFrameStageGuardEnabled();
    if(!c.active) return EvaluateKeyFrameDecision(c,requireNeedDecision);
    c.frameId=static_cast<long>(mCurrentFrame.mnId);
    c.coarseFallback=!mCoarseFilterFallbackReason.empty();
    c.coarseReliable=mCoarsePoseQuality.reliable;
    c.coarseMatches=mCoarsePoseQuality.match_count;
    c.coarseInliers=mCoarsePoseQuality.inlier_count;
    c.evidenceComplete=mbRGBDEvidenceProcessed;
    c.fineCompleted=mbTwoStageFineCompleted;
    c.optimizerExecuted=mbFineOptimizationExecuted;
    c.fineFailed=mbFineTrackingFailed;
    c.currentInliers=mnMatchesInliers;
    c.finitePose=mCurrentFrame.HasPose() && mCurrentFrame.GetPose().matrix().allFinite();
    c.finalPoseUnchanged=c.finalFrameId==c.frameId && c.finitePose &&
        mCurrentFrame.GetPose().matrix().isApprox(mFinalKeyFramePose.matrix(),1e-6f);
    c.semanticPolicyAvailable=false;
    if(c.semanticFrameId==c.frameId && mCurrentFrame.mbSemanticAnnotated &&
        mCurrentFrame.SemanticMetadataAligned()) {
        try {
            const auto policies=ComputeFeaturePolicies(mCurrentFrame,FeaturePolicyPhase::FINE,true);
            for(int i=0;i<mCurrentFrame.N;++i) {
                const bool reject=!FeaturePolicyAllows(policies[i],FeaturePolicyPhase::FINE);
                if(mCurrentFrame.mvFeaturePolicy[i]!=policies[i] || mCurrentFrame.mvDynamicExcluded[i]!=reject)
                    throw std::runtime_error("Keyframe semantic policy metadata inconsistent");
            }
            // Fine-only ablation intentionally retains its old unfiltered map
            // creation. After a successful creation those NEW associations are
            // not optimizer inputs. Never confuse them with semantic failure.
            // Need/Create approval always verifies the pre-creation inputs;
            // map-admission ACTION additionally verifies post-creation inputs.
            if(!c.created || mpDynamicDetector->Frontend().mapAdmissionEnable)
                VerifyFineOptimizerInputs(mCurrentFrame);
            c.semanticPolicyAvailable=true;
        } catch(const std::runtime_error&) {
            // No silent semantic fallback and no insertion with stale policy.
            // Existing inference/decoder errors remain fatal at their source.
        }
    }
    return EvaluateKeyFrameDecision(c,requireNeedDecision);
}

void Tracking::RecordKeyFrameStage(const char* phase,bool requireNeedDecision)
{
    if(!KeyFrameStageGuardEnabled()) return;
    auto result=CurrentKeyFrameGate(requireNeedDecision);
    const auto& c=mKeyFrameDecision;
    // Initialization constructs the FIRST map, not a regular tracking keyframe.
    // This log exception cannot bypass NeedNewKeyFrame/CreateNewKeyFrame guards.
    if(c.bootstrap && (std::strcmp(phase,"bootstrap")==0 || std::strcmp(phase,"frame_summary")==0))
        result={true,"BOOTSTRAP_INITIAL_MAP_REQUIRED"};
    std::ostringstream message;
    message << "[KEYFRAME STAGE] frame=" << c.frameId << " phase=" << phase
        << " coarse_matches=" << c.coarseMatches << " coarse_inliers=" << c.coarseInliers
        << " final_inliers=" << c.finalInliers << " decision_inliers=" << c.decisionInliers
        << " required_static_support=" << c.requiredStaticSupport
        << " allowed=" << int(result.allowed) << " reason=" << result.reason
        << " needed=" << int(c.needed) << " created=" << int(c.created)
        << " bootstrap=" << int(c.bootstrap);
    cout << message.str() << endl;
    mKeyFrameStageCsv << std::setprecision(17) << c.frameId << ',' << mCurrentFrame.mTimeStamp << ',' << phase
        << ',' << c.semanticFrameId << ',' << c.finalFrameId << ',' << int(c.coarseFallback)
        << ',' << int(c.coarseReliable) << ',' << int(c.evidenceComplete) << ',' << int(c.fineCompleted)
        << ',' << int(c.optimizerExecuted) << ',' << int(c.fineSucceeded) << ',' << int(c.fineFailed)
        << ',' << int(c.semanticPolicyAvailable) << ',' << c.coarseMatches << ',' << c.coarseInliers
        << ',' << c.finalInliers << ',' << c.currentInliers << ',' << c.decisionInliers
        << ',' << c.requiredStaticSupport << ',' << int(c.finitePose) << ',' << int(c.finalPoseUnchanged)
        << ',' << int(result.allowed) << ',' << result.reason << ',' << mCoarsePoseQuality.reason
        << ',' << int(c.needEvaluated) << ',' << int(c.needed) << ',' << int(c.createAttempted)
        << ',' << int(c.created) << ',' << int(c.bootstrap) << '\n';
    mKeyFrameStageCsv.flush();
    if(!mKeyFrameStageCsv) throw std::runtime_error("Cannot write keyframe stage CSV");
}

bool Tracking::NeedNewKeyFrame()
{
    if(!KeyFrameStageGuardEnabled()) return EvaluateOriginalKeyFrameNeed(mnMatchesInliers);
    if(!CurrentKeyFrameGate().allowed) {
        RecordKeyFrameStage("need_guard");
        return false;
    }
    // Never substitute the initial/coarse matches for the local-map inliers.
    mKeyFrameDecision.needEvaluated=true;
    mKeyFrameDecision.decisionInliers=mKeyFrameDecision.finalInliers;
    mKeyFrameDecision.needed=EvaluateOriginalKeyFrameNeed(mKeyFrameDecision.finalInliers);
    RecordKeyFrameStage("need_decision");
    return mKeyFrameDecision.needed;
}

bool Tracking::EvaluateOriginalKeyFrameNeed(int decisionInliers)
{
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        /*if(mSensor == System::MONOCULAR)
        {
            std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        return false;
    }

    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
        //Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
    const int thStereoClosedPoints = 15;
    if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
    {
        //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
        thRefRatio = 0.9f;
    }*/

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(decisionInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle); //mpLocalMapper->KeyframesInQueue() < 2);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && mSensor!=System::IMU_RGBD && (decisionInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (((decisionInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && decisionInliers>15);

    //std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;
    // Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((decisionInliers<75) && (decisionInliers>15)) || mState==RECENTLY_LOST) && (mSensor == System::IMU_MONOCULAR)) // Original keyframe heuristic, unchanged thresholds.
        c4=true;
    else
        c4=false;

    if(((c1a||c1b||c1c) && c2)||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle || mpLocalMapper->IsInitializing())
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
            {
                //std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
                return false;
            }
        }
    }
    else
        return false;
}

bool Tracking::CreateNewKeyFrame()
{
    if(KeyFrameStageGuardEnabled()) {
        mKeyFrameDecision.createAttempted=true;
        RecordKeyFrameStage("create_guard",true);
        if(!CurrentKeyFrameGate(true).allowed) return false;
    }
    // Guard BEFORE constructing KeyFrame or creating any of its depth MapPoints.
    if(mbRGBDEvidenceReady && mpDynamicDetector->Frontend().finePolicyEnable &&
        !FinePolicyAllowsKeyFrame(mbFineOptimizationExecuted,mbFineTrackingFailed)) return false;
    if (mbRGBDEvidenceReady && mpDynamicDetector->Frontend().useCoarsePoseForB2 &&
        (!mbRGBDEvidenceProcessed || !mbTwoStageFineCompleted))
        throw std::runtime_error("Regular KeyFrame insertion requires evidence and TrackLocalMap completion");
    if(mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
        return false;

    if(!mpLocalMapper->SetNotStop(true))
        return false;

    if(mbRGBDEvidenceReady && mpDynamicDetector->Frontend().mapAdmissionEnable)
        PrepareFrameMapAdmission(mCurrentFrame,mMapAdmissionLogPath);
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    MapAdmissionBatch admission(pKF,"rgbd_depth_creation");

    if(mpAtlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }
    else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
    }

    if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            maxPoint = 100;

        vector<pair<float,int> > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                if(!admission.Check(i)) continue;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    Eigen::Vector3f x3D;

                    if(mCurrentFrame.Nleft == -1){
                        mCurrentFrame.UnprojectStereo(i, x3D);
                    }
                    else{
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF,i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
                        pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    admission.Created(i);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
                {
                    break;
                }
            }
            //Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
        }
    }


    if (mbRGBDEvidenceReady && mpDynamicDetector->Frontend().useCoarsePoseForB2)
        cout << "[TWO STAGE] frame=" << mCurrentFrame.mnId
             << " event=keyframe_enqueue fine_completed=" << int(mbTwoStageFineCompleted) << endl;
    mpLocalMapper->InsertKeyFrame(pKF);

    if(KeyFrameStageGuardEnabled()) {
        mKeyFrameDecision.created=true;
        RecordKeyFrameStage("created",true);
    }

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
    return true;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=6;
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15

        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    int count_pts = 0;

    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {

            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }
    else
    {
        for(int i=0; i<mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }


    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80) // 80
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);


        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&mvpLocalKeyFrames.size()<80)
    {
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                tempKeyFrame=tempKeyFrame->mPrevKF;
            }
        }
    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    if (mbRGBDEvidenceReady && mpDynamicDetector->Frontend().useCoarsePoseForB2)
        mCoarsePoseContext.relocalizing = true;
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    if(vpCandidateKFs.empty()) {
        Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,6,0.5,5.991);  //This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            bool bTcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers, eigTcw);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(bTcw)
            {
                Sophus::SE3f Tcw(eigTcw);
                mCurrentFrame.SetPose(Tcw);
                // Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = OptimizeInitialPoseWithDiagnostics();

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = OptimizeInitialPoseWithDiagnostics();

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = OptimizeInitialPoseWithDiagnostics();

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }

}

void Tracking::Reset(bool bLocMap)
{
    if (mpB4StateMachine && mB4Config.evidenceExpiryEnable) {
        mpB4StateMachine->Reset();
        mCurrentB4Results.clear();
    }
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }


    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit=false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap)
{
    if (mpB4StateMachine && mB4Config.evidenceExpiryEnable) {
        mpB4StateMachine->Reset();
        mCurrentB4Results.clear();
    }
    Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();


    //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    //Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    //mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET; //NOT_INITIALIZED;

    mbReadyToInitializate = false;

    list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for(Map* pMap : mpAtlas->GetAllMaps())
    {
        if(pMap->GetAllKeyFrames().size() > 0)
        {
            if(index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    //cout << "First Frame id: " << index << endl;
    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
    {
        if(index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    mbVelocity = false;

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPoint*> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    mK_.setIdentity();
    mK_(0,0) = fx;
    mK_(1,1) = fy;
    mK_(0,2) = cx;
    mK_(1,2) = cy;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame)
{
    Map * pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();
    for(auto lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        while(pKF->isBad())
        {
            pKF = pKF->GetParent();
        }

        if(pKF->GetMap() == pMap)
        {
            (*lit).translation() *= s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    while(!mCurrentFrame.imuIsPreintegrated())
    {
        usleep(500);
    }


    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated)
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}

void Tracking::NewDataset()
{
    mnNumDataset++;
}

int Tracking::GetNumberDataset()
{
    return mnNumDataset;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder)
{
    mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
    //mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap)
{
    mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
    if(!strNameFile_kf.empty())
        mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracking::GetImageScale()
{
    return mImageScale;
}

#ifdef REGISTER_LOOP
void Tracking::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
}

bool Tracking::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Tracking STOP" << endl;
        return true;
    }

    return false;
}

bool Tracking::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool Tracking::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

void Tracking::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
}
#endif

} //namespace ORB_SLAM
