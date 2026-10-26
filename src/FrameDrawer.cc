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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>
#include <cmath>

namespace ORB_SLAM3
{

FrameDrawer::FrameDrawer(Atlas* pAtlas):both(false),mpAtlas(pAtlas)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mImRight = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Scalar FrameDrawer::TemporalFeatureColor(SemanticRegion region,TemporalState state)
{
    if(region==SemanticRegion::BACKGROUND) return cv::Scalar(0,255,0);
    if(region!=SemanticRegion::PERSON_INTERIOR) return cv::Scalar(128,128,128);
    switch(state) {
    case TemporalState::STATIC:return cv::Scalar(255,255,0); // cyan
    case TemporalState::PENDING:return cv::Scalar(0,255,255); // yellow
    case TemporalState::DYNAMIC:return cv::Scalar(0,0,255); // red
    default:return cv::Scalar(128,128,128);
    }
}

void FrameDrawer::DrawStateActionOverlay(cv::Mat& image,const std::vector<cv::KeyPoint>& keys,
    const std::vector<SemanticRegion>& regions,const FrontendDiagnostics& diagnostics,float imageScale)
{
    CV_Assert(imageScale>0 && keys.size()==regions.size() && keys.size()==diagnostics.currentFeatureStates.size());
    for(size_t i=0;i<keys.size();++i)
        cv::circle(image,keys[i].pt/imageScale,2,TemporalFeatureColor(regions[i],diagnostics.currentFeatureStates[i]),-1,cv::LINE_AA);
    for(const auto& instance:diagnostics.stateActionInstances) {
        cv::Rect box(cvRound(instance.boundingBox.x/imageScale),cvRound(instance.boundingBox.y/imageScale),
            cvRound(instance.boundingBox.width/imageScale),cvRound(instance.boundingBox.height/imageScale));
        box &= cv::Rect(0,0,image.cols,image.rows);
        if(box.empty()) continue;
        cv::rectangle(image,box,instance.color,2);
        int y=std::max(16,std::min(box.y+16,image.rows-70));
        for(const auto& text:{instance.primaryLabel,instance.secondaryLabel,instance.tertiaryLabel,instance.quaternaryLabel}) {
            const int width=cv::getTextSize(text,cv::FONT_HERSHEY_SIMPLEX,.36,1,nullptr).width;
            const int x=std::max(0,std::min(box.x,image.cols-width-2));
            cv::putText(image,text,cv::Point(x,y),cv::FONT_HERSHEY_SIMPLEX,.36,cv::Scalar(0,0,0),3,cv::LINE_AA);
            cv::putText(image,text,cv::Point(x,y),cv::FONT_HERSHEY_SIMPLEX,.36,instance.color,1,cv::LINE_AA);
            y+=16;
        }
    }
    cv::putText(image,"BG green | S cyan | P yellow | D red | boundary/unknown gray",cv::Point(4,image.rows-27),
        cv::FONT_HERSHEY_SIMPLEX,.38,cv::Scalar(255,255,255),1,cv::LINE_AA);
    cv::putText(image,diagnostics.stageTimingSummary,cv::Point(4,image.rows-8),
        cv::FONT_HERSHEY_SIMPLEX,.34,cv::Scalar(255,255,255),1,cv::LINE_AA);
}

void FrameDrawer::DrawSemanticFeatures(cv::Mat& image, const std::vector<cv::KeyPoint>& keys,
    const std::vector<SemanticRegion>& regions, float imageScale)
{
    CV_Assert(keys.size() == regions.size() && imageScale > 0.f);
    for (size_t i = 0; i < keys.size(); ++i) {
        cv::Scalar color(128,128,128);
        switch (regions[i]) {
        case SemanticRegion::BACKGROUND: color = cv::Scalar(255,255,0); break;
        case SemanticRegion::PERSON_INTERIOR: color = cv::Scalar(255,0,0); break;
        case SemanticRegion::PERSON_BOUNDARY: color = cv::Scalar(0,165,255); break;
        default: break;
        }
        if (std::isfinite(keys[i].pt.x) && std::isfinite(keys[i].pt.y) &&
            keys[i].pt.x >= 0.f && keys[i].pt.y >= 0.f &&
            keys[i].pt.x/imageScale < image.cols && keys[i].pt.y/imageScale < image.rows)
            cv::circle(image,keys[i].pt/imageScale,2,color,-1,cv::LINE_AA);
    }
    cv::putText(image,"MAIN ORB: BG=cyan PERSON=blue BOUNDARY=orange UNKNOWN=gray",
        cv::Point(4,image.rows-8),cv::FONT_HERSHEY_SIMPLEX,.38,cv::Scalar(255,255,255),1,cv::LINE_AA);
}

cv::Mat FrameDrawer::DrawFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<cv::KeyPoint> vMaskedRejected;
    std::vector<SemanticRegion> semanticRegions;
    bool showSemantics = false;
    cv::Mat dynamicMask;
    FrontendDiagnostics frontendDiagnostics;
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state
    vector<float> vCurrentDepth;
    float thDepth;

    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(0,255,0);
    cv::Scalar odometryColor(255,0,0);

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);
        dynamicMask=mDynamicMask.clone();
        vMaskedRejected=mvMaskedRejected;
        frontendDiagnostics=mFrontendDiagnostics;
        showSemantics = mbSemanticOverlayEnabled;
        if (showSemantics) {
            semanticRegions = mvSemanticRegion;
            vCurrentKeys = mvCurrentKeys;
        }

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

            vCurrentDepth = mvCurrentDepth;
            thDepth = mThDepth;

        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
        if(!dynamicMask.empty())
            cv::resize(dynamicMask,dynamicMask,im.size(),0,0,cv::INTER_NEAREST);
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    if(!dynamicMask.empty() && dynamicMask.size()==im.size()) {
        cv::Mat colored=im.clone();
        colored.setTo(cv::Scalar(0,0,255),dynamicMask);
        cv::addWeighted(im,0.68,colored,0.32,0,im);
    }

    //Draw
    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }
                cv::line(im,pt1,pt2,standardColor);
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
        {
            cv::Point2f pt1,pt2;
            if(imageScale != 1.f)
            {
                pt1 = (*it).first / imageScale;
                pt2 = (*it).second / imageScale;
            }
            else
            {
                pt1 = (*it).first;
                pt2 = (*it).second;
            }
            cv::line(im,pt1,pt2, standardColor,5);
        }

    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = vCurrentKeys[i].pt / imageScale;
                    float px = vCurrentKeys[i].pt.x / imageScale;
                    float py = vCurrentKeys[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = vCurrentKeys[i].pt;
                    pt1.x=vCurrentKeys[i].pt.x-r;
                    pt1.y=vCurrentKeys[i].pt.y-r;
                    pt2.x=vCurrentKeys[i].pt.x+r;
                    pt2.y=vCurrentKeys[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,standardColor);
                    cv::circle(im,point,2,standardColor,-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,odometryColor);
                    cv::circle(im,point,2,odometryColor,-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    // Green: final descriptor-bearing ORB features. Red X: candidates rejected
    // before octree/descriptor computation because they overlap a person mask.
    if (showSemantics && !frontendDiagnostics.stateActionOverlayEnabled)
        DrawSemanticFeatures(im,vCurrentKeys,semanticRegions,imageScale);
    else
        for(const cv::KeyPoint& kp : vCurrentKeys)
            cv::circle(im,kp.pt/imageScale,2,cv::Scalar(0,255,0),-1,cv::LINE_AA);
    for(const cv::KeyPoint& kp : vMaskedRejected) {
        const cv::Point2f p=kp.pt/imageScale;
        cv::line(im,p+cv::Point2f(-3,-3),p+cv::Point2f(3,3),cv::Scalar(0,0,255),1,cv::LINE_AA);
        cv::line(im,p+cv::Point2f(-3,3),p+cv::Point2f(3,-3),cv::Scalar(0,0,255),1,cv::LINE_AA);
    }

    // B2 is evidence-only. These colors describe measurement availability and
    // residual evidence, never a final dynamic/static decision.
    if (frontendDiagnostics.b2OverlayEnabled && !frontendDiagnostics.stateActionOverlayEnabled) {
        for (const B2ViewerPoint& point : frontendDiagnostics.b2Points) {
            cv::Scalar color(255,0,0); // blue: unmatched candidate
            switch (point.pointClass) {
            case B2_VIEW_INVALID: color = cv::Scalar(128,128,128); break; // gray
            case B2_VIEW_LOW_RESIDUAL: color = cv::Scalar(0,255,0); break; // green
            case B2_VIEW_HIGH_RESIDUAL: color = cv::Scalar(0,0,255); break; // red
            case B2_VIEW_BOUNDARY: color = cv::Scalar(0,255,255); break; // yellow
            default: break;
            }
            cv::circle(im,point.pixel/imageScale,4,color,1,cv::LINE_AA);
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);
    std::ostringstream diagnostic;
    diagnostic << "ORB " << frontendDiagnostics.finalOrbFeatures
               << " | reject " << frontendDiagnostics.maskedRejected
               << " | inliers " << frontendDiagnostics.poseInliers
               << " | matches " << frontendDiagnostics.matches
               << " | KF " << (frontendDiagnostics.becameKeyFrame ? "yes" : "no");
    if(frontendDiagnostics.stateActionOverlayEnabled) {
        cv::copyMakeBorder(imWithInfo,imWithInfo,0,70,0,0,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
        cv::putText(imWithInfo,diagnostic.str(),cv::Point(4,imWithInfo.rows-49),cv::FONT_HERSHEY_SIMPLEX,
            .44,cv::Scalar(255,255,255),1,cv::LINE_AA);
        DrawStateActionOverlay(imWithInfo,vCurrentKeys,semanticRegions,frontendDiagnostics,imageScale);
        return imWithInfo;
    }
    if(!frontendDiagnostics.b2EvidenceSummary.empty())
        diagnostic << " | " << frontendDiagnostics.b2EvidenceSummary;
    if(frontendDiagnostics.b3OverlayEnabled && !frontendDiagnostics.b3EvidenceSummary.empty())
        diagnostic << " | " << frontendDiagnostics.b3EvidenceSummary;
    cv::putText(imWithInfo,diagnostic.str(),cv::Point(5,22),cv::FONT_HERSHEY_SIMPLEX,
                0.52,cv::Scalar(0,255,255),1,cv::LINE_AA);
    if(frontendDiagnostics.b2OverlayEnabled || frontendDiagnostics.b3OverlayEnabled) {
        int y = frontendDiagnostics.b2OverlayEnabled ? 64 : 43;
        if (frontendDiagnostics.b2OverlayEnabled) {
        std::ostringstream b2Diagnostic;
        b2Diagnostic << "B2 candidates " << frontendDiagnostics.b2Candidates
                     << " | B2 matched " << frontendDiagnostics.b2Matched
                     << " | B2 valid " << frontendDiagnostics.b2Valid
                     << " | median r3D " << frontendDiagnostics.b2MedianR3D
                     << " | median normalized r3D " << frontendDiagnostics.b2MedianNormalizedR3D
                     << " | B2 time " << frontendDiagnostics.b2ProcessingMs << " ms"
                     << " | High residual evidence";
        cv::putText(imWithInfo,b2Diagnostic.str(),cv::Point(5,43),cv::FONT_HERSHEY_SIMPLEX,
                    0.43,cv::Scalar(0,255,255),1,cv::LINE_AA);
        for (const std::string& instance : frontendDiagnostics.b2InstanceEvidenceSummaries) {
            cv::putText(imWithInfo,instance,cv::Point(5,y),cv::FONT_HERSHEY_SIMPLEX,
                        0.43,cv::Scalar(0,255,255),1,cv::LINE_AA);
            y += 19;
        }
        } else {
            for (const std::string& instance : frontendDiagnostics.b3InstanceEvidenceSummaries) {
                cv::putText(imWithInfo,instance,cv::Point(5,y),cv::FONT_HERSHEY_SIMPLEX,
                            0.43,cv::Scalar(255,255,0),1,cv::LINE_AA);
                y += 19;
            }
        }
        if (frontendDiagnostics.b3OverlayEnabled && frontendDiagnostics.b2OverlayEnabled) for (const std::string& instance : frontendDiagnostics.b3InstanceEvidenceSummaries) {
            cv::putText(imWithInfo,instance,cv::Point(5,y),cv::FONT_HERSHEY_SIMPLEX,
                        0.43,cv::Scalar(255,255,0),1,cv::LINE_AA);
            y += 19;
        }
    }

    if (frontendDiagnostics.b3OverlayEnabled && !frontendDiagnostics.stateActionOverlayEnabled) {
        for (const B3ViewerInstance& instance : frontendDiagnostics.b3Instances) {
            const cv::Rect box(cvRound(instance.boundingBox.x / imageScale),
                               cvRound(instance.boundingBox.y / imageScale),
                               cvRound(instance.boundingBox.width / imageScale),
                               cvRound(instance.boundingBox.height / imageScale));
            if (box.width <= 0 || box.height <= 0) continue;
            cv::rectangle(imWithInfo, box, instance.color, 2, cv::LINE_AA);
            const int labelY = std::max(15, box.y - 7);
            cv::putText(imWithInfo, instance.primaryLabel, cv::Point(box.x, labelY),
                        cv::FONT_HERSHEY_SIMPLEX, 0.48, instance.color, 2, cv::LINE_AA);
            cv::putText(imWithInfo, instance.secondaryLabel, cv::Point(box.x, labelY + 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.43, instance.color, 1, cv::LINE_AA);
            cv::putText(imWithInfo, instance.tertiaryLabel, cv::Point(box.x, labelY + 36),
                        cv::FONT_HERSHEY_SIMPLEX, 0.38, instance.color, 1, cv::LINE_AA);
            if (frontendDiagnostics.b4OverlayEnabled)
                cv::putText(imWithInfo, instance.quaternaryLabel, cv::Point(box.x, labelY + 54),
                            cv::FONT_HERSHEY_SIMPLEX, 0.38, instance.color, 1, cv::LINE_AA);
        }
    }

    if(frontendDiagnostics.stateActionOverlayEnabled)
        DrawStateActionOverlay(imWithInfo,vCurrentKeys,semanticRegions,frontendDiagnostics,imageScale);
    return imWithInfo;
}

cv::Mat FrameDrawer::DrawRightFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<cv::KeyPoint> vMaskedRejected;
    cv::Mat dynamicMask;
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mImRight.copyTo(im);
        dynamicMask=mDynamicMaskRight.clone();
        vMaskedRejected=mvMaskedRejectedRight;

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeysRight;
        }
    } // destroy scoped mutex -> release mutex

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
        if(!dynamicMask.empty())
            cv::resize(dynamicMask,dynamicMask,im.size(),0,0,cv::INTER_NEAREST);
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    if(!dynamicMask.empty() && dynamicMask.size()==im.size()) {
        cv::Mat colored=im.clone();
        colored.setTo(cv::Scalar(0,0,255),dynamicMask);
        cv::addWeighted(im,0.68,colored,0.32,0,im);
    }

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }

                cv::line(im,pt1,pt2,cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = mvCurrentKeysRight.size();
        const int Nleft = mvCurrentKeys.size();

        for(int i=0;i<n;i++)
        {
            if(vbVO[i + Nleft] || vbMap[i + Nleft])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = mvCurrentKeysRight[i].pt / imageScale;
                    float px = mvCurrentKeysRight[i].pt.x / imageScale;
                    float py = mvCurrentKeysRight[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = mvCurrentKeysRight[i].pt;
                    pt1.x=mvCurrentKeysRight[i].pt.x-r;
                    pt1.y=mvCurrentKeysRight[i].pt.y-r;
                    pt2.x=mvCurrentKeysRight[i].pt.x+r;
                    pt2.y=mvCurrentKeysRight[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i + Nleft])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,point,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,point,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    for(const cv::KeyPoint& kp : vCurrentKeys)
        cv::circle(im,kp.pt/imageScale,2,cv::Scalar(0,255,0),-1,cv::LINE_AA);
    for(const cv::KeyPoint& kp : vMaskedRejected) {
        const cv::Point2f p=kp.pt/imageScale;
        cv::line(im,p+cv::Point2f(-3,-3),p+cv::Point2f(3,3),cv::Scalar(0,0,255),1,cv::LINE_AA);
        cv::line(im,p+cv::Point2f(-3,3),p+cv::Point2f(3,-3),cv::Scalar(0,0,255),1,cv::LINE_AA);
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}



void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mDynamicMask=pTracker->mCurrentFrame.mDynamicMaskLeft.clone();
    mvMaskedRejected=pTracker->mCurrentFrame.mvMaskedRejectedLeft;
    mDynamicMaskRight=pTracker->mCurrentFrame.mDynamicMaskRight.clone();
    mvMaskedRejectedRight=pTracker->mCurrentFrame.mvMaskedRejectedRight;
    mFrontendDiagnostics=pTracker->mFrontendDiagnostics;
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mvSemanticRegion=pTracker->mCurrentFrame.mvSemanticRegion;
    mbSemanticOverlayEnabled=pTracker->mCurrentFrame.mbSemanticOverlayEnabled;
    mThDepth = pTracker->mCurrentFrame.mThDepth;
    mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

    if(both){
        mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
        pTracker->mImRight.copyTo(mImRight);
        N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
    }
    else{
        N = mvCurrentKeys.size();
    }

    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    //Variables for the new visualization
    mCurrentFrame = pTracker->mCurrentFrame;
    mmProjectPoints = mCurrentFrame.mmProjectPoints;
    mmMatchedInImage.clear();

    mvpLocalMap = pTracker->GetLocalMapMPS();
    mvMatchedKeys.clear();
    mvMatchedKeys.reserve(N);
    mvpMatchedMPs.clear();
    mvpMatchedMPs.reserve(N);
    mvOutlierKeys.clear();
    mvOutlierKeys.reserve(N);
    mvpOutlierMPs.clear();
    mvpOutlierMPs.reserve(N);

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;

                    mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;
                }
                else
                {
                    mvpOutlierMPs.push_back(pMP);
                    mvOutlierKeys.push_back(mvCurrentKeys[i]);
                }
            }
        }

    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
