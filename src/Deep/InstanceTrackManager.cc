#include "Deep/InstanceTrackManager.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <chrono>
#include <limits>

namespace ORB_SLAM3 {
float InstanceTrackManager::MaskIoU(const cv::Mat& a, const cv::Mat& b) {
    if (a.empty() || b.empty() || a.type()!=CV_8UC1 || b.type()!=CV_8UC1 || a.size()!=b.size()) return 0.f;
    cv::Mat inter, uni; cv::bitwise_and(a,b,inter); cv::bitwise_or(a,b,uni);
    const int n=cv::countNonZero(uni); return n ? float(cv::countNonZero(inter))/n : 0.f;
}
float InstanceTrackManager::BoxIoU(const cv::Rect& a, const cv::Rect& b) {
    const int n=a.area()+b.area()-(a&b).area(); return n ? float((a&b).area())/n : 0.f;
}
float InstanceTrackManager::CenterDistance(const cv::Point2f& a, const cv::Point2f& b,
                                           const cv::Size& s) {
    const float dx=a.x-b.x, dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy)/std::max(1.f, std::sqrt(float(s.width*s.width+s.height*s.height)));
}
void InstanceTrackManager::AssociateLegacy(std::vector<YOLOSegInstance>& instances, const cv::Size& imageSize,
                                          long frameId,double timestamp) {
    for (auto& entry : mTracks) ++entry.second.missedFrames;
    struct Pair { int track; size_t instance; float score; };
    std::vector<Pair> candidates;
    for (const auto& entry : mTracks) for (size_t i=0; i<instances.size(); ++i) {
        const Track& t=entry.second; const YOLOSegInstance& in=instances[i];
        if (t.classId != in.classId) continue;
        const float mask=MaskIoU(t.mask,in.mask), box=BoxIoU(t.boundingBox,in.boundingBox);
        const cv::Point2f center(in.boundingBox.x+.5f*in.boundingBox.width,in.boundingBox.y+.5f*in.boundingBox.height);
        const float distance=CenterDistance(t.center,center,imageSize);
        if (!(mask>=mConfig.minMaskIoU || box>=mConfig.minBoxIoU || distance<=mConfig.maxCenterDistance)) continue;
        candidates.push_back({entry.first,i,.50f*mask+.25f*box+.25f*(1.f-std::min(1.f,distance))});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Pair& a,const Pair& b){return a.score>b.score;});
    std::vector<int> assignment(instances.size(),-1); std::unordered_set<int> used;
    for (const Pair& p:candidates) if (!used.count(p.track) && assignment[p.instance]<0) { assignment[p.instance]=p.track; used.insert(p.track); }
    for (size_t i=0;i<instances.size();++i) {
        int id=assignment[i]; if(id<0) { id=mNextTrackId++; mTracks[id].id=id; }
        YOLOSegInstance& in=instances[i]; Track& t=mTracks[id];
        auto& a=in.association; a=InstanceAssociationResult(); a.evaluated=true;
        a.instanceId=i; a.trackId=id; a.allowStateInheritance=t.lastMatchedFrame>=0;
        a.previousMatchedFrame=t.lastMatchedFrame; a.previousMatchedTimestamp=t.lastMatchedTimestamp;
        a.allowGeometryPair=t.lastMatchedFrame>=0 && t.lastMatchedFrame==frameId-1;
        a.associationStatus=t.lastMatchedFrame<0 ? AssociationStatus::NEW_TRACK :
            a.allowGeometryPair ? AssociationStatus::CONTINUOUS_CONFIRMED : AssociationStatus::REACQUIRED;
        a.reason="A0_LEGACY_GREEDY";
        in.trackId=id; t.classId=in.classId; t.mask=in.mask.clone(); t.boundingBox=in.boundingBox;
        t.center=cv::Point2f(in.boundingBox.x+.5f*in.boundingBox.width,in.boundingBox.y+.5f*in.boundingBox.height); t.missedFrames=0;
        t.lastMatchedFrame=frameId; t.lastMatchedTimestamp=timestamp;
    }
    for(auto it=mTracks.begin();it!=mTracks.end();) {
        if(it->second.missedFrames>mConfig.trackTTL) {
            mEvents.push_back({it->first,"TERMINATED","TRACK_TTL_EXPIRED"});it=mTracks.erase(it);
        } else ++it;
    }
}

namespace {
double Median(std::vector<float> values) {
    if(values.empty()) return -1;
    auto middle=values.begin()+values.size()/2;std::nth_element(values.begin(),middle,values.end());
    return *middle;
}
cv::Point2f Center(const cv::Rect& b) {return cv::Point2f(b.x+b.width*.5f,b.y+b.height*.5f);}
float Depth(const cv::Mat& depth,const cv::Mat& mask,const InstanceAssociationConfig& cfg) {
    if(depth.empty()) return -1;
    std::vector<float> samples;
    for(int y=0;y<depth.rows;y+=4) for(int x=0;x<depth.cols;x+=4)
        if(mask.at<uchar>(y,x)) {
            float z=depth.at<float>(y,x);
            if(std::isfinite(z)&&z>=.3f&&z<=5.f) samples.push_back(z);
        }
    if(samples.size()<size_t(cfg.minDepthSamples)) return -1;
    const float middle=Median(samples);
    for(float& z:samples) z=std::abs(z-middle);
    return Median(samples)<=cfg.maxDepthMAD*middle ? middle : -1;
}
}

void InstanceTrackManager::ValidateInputTime(long frameId,double timestamp) const {
    if(frameId<0 || !std::isfinite(timestamp) ||
        (mHasInput && (frameId<=mLastFrame || timestamp<=mLastTimestamp)))
        throw std::runtime_error("InstanceAssociation requires strictly advancing finite acquisition times/frame IDs; reset before sequence switch");
}
void InstanceTrackManager::Reset() {
    mEvents.clear();
    for(const auto& t:mTracks) mEvents.push_back({t.first,"TERMINATED","SEQUENCE_RESET"});
    mTracks.clear();mHasInput=false;mLastFrame=-1;mLastTimestamp=0;mLegacyFrame=0;++mSequenceId;
}
void InstanceTrackManager::Associate(std::vector<YOLOSegInstance>& instances,const cv::Size& size) {
    if(mAssociation.mode!=0) throw std::runtime_error("Enhanced association requires acquisition timestamp/frame input");
    Associate(instances,size,mLegacyFrame,mLegacyFrame/30.0);
    ++mLegacyFrame;
}
std::vector<int> InstanceTrackManager::SolveAssignment(const std::vector<std::vector<double>>& a) {
    const int n=a.size();if(!n) return {};
    for(const auto& row:a) {
        if(row.size()!=size_t(n)) throw std::runtime_error("Assignment matrix must be square");
        for(double cost:row) if(!std::isfinite(cost)) throw std::runtime_error("Assignment costs must be finite");
    }
    std::vector<double> u(n+1),v(n+1);
    std::vector<int> p(n+1),way(n+1);
    for(int i=1;i<=n;++i) {
        p[0]=i;int j0=0;std::vector<double> minv(n+1,1e12);std::vector<bool> used(n+1,false);
        do {
            used[j0]=true;int i0=p[j0],j1=0;double delta=1e12;
            for(int j=1;j<=n;++j) if(!used[j]) {
                double cur=a[i0-1][j-1]-u[i0]-v[j];
                if(cur<minv[j]) {minv[j]=cur;way[j]=j0;}
                if(minv[j]<delta) {delta=minv[j];j1=j;}
            }
            for(int j=0;j<=n;++j) {if(used[j]) {u[p[j]]+=delta;v[j]-=delta;} else minv[j]-=delta;}
            j0=j1;
        } while(p[j0]!=0);
        do {int j1=way[j0];p[j0]=p[j1];j0=j1;} while(j0);
    }
    std::vector<int> out(n,-1);for(int j=1;j<=n;++j) out[p[j]-1]=j-1;return out;
}
void InstanceTrackManager::Associate(std::vector<YOLOSegInstance>& instances,const cv::Size& size,
    long frameId,double timestamp,const cv::Mat& gray,const cv::Mat& depth) {
    const auto started=std::chrono::steady_clock::now();
    ValidateInputTime(frameId,timestamp);
    if(size.width<=0||size.height<=0 || (!gray.empty()&&(gray.type()!=CV_8UC1||gray.size()!=size)) ||
       (!depth.empty()&&(depth.type()!=CV_32FC1||depth.size()!=size)))
        throw std::runtime_error("InstanceAssociation image/depth size/type mismatch");
    const cv::Rect bounds(0,0,size.width,size.height);
    for(const auto& in:instances) {
        if(in.classId<0 || !std::isfinite(in.confidence) || in.confidence<0 || in.confidence>1 ||
           in.boundingBox.empty() || (in.boundingBox&bounds)!=in.boundingBox ||
           in.mask.type()!=CV_8UC1||in.mask.size()!=size||!cv::countNonZero(in.mask))
            throw std::runtime_error("Invalid InstanceAssociation instance/mask/bbox");
        if(cv::countNonZero((in.mask!=0)&(in.mask!=255)))
            throw std::runtime_error("InstanceAssociation mask must be binary 0/255");
    }
    mEvents.clear();
    if(mAssociation.mode==0) AssociateLegacy(instances,size,frameId,timestamp);
    else {
        // A returning detection after exactly TTL missed images may match;
        // TTL+1 missed images must be removed BEFORE assignment can reset age.
        for(auto it=mTracks.begin();it!=mTracks.end();) {
            if(frameId-it->second.lastMatchedFrame-1>mConfig.trackTTL) {
                mEvents.push_back({it->first,"TERMINATED","PRE_ASSIGNMENT_TTL_EXPIRED"});it=mTracks.erase(it);
            } else ++it;
        }
        std::vector<int> ids;for(const auto& entry:mTracks) ids.push_back(entry.first);
        std::sort(ids.begin(),ids.end());
        struct Prediction {cv::Mat mask;cv::Rect box;int flow=0;std::string source="NONE";bool usable=true;};
        std::vector<Prediction> predictions;
        const auto& c=mAssociation;
        for(int id:ids) {
            const Track& track=mTracks.at(id);Prediction pred;pred.mask=track.mask;pred.box=track.boundingBox;
            const double dt=timestamp-track.lastMatchedTimestamp;
            pred.usable=!track.ambiguousTemplate && (c.mode<2 || dt<=c.maxPredictionSeconds);
            if(c.mode>=2 && pred.usable) {
                cv::Point2f displacement=track.velocity*float(dt);
                pred.source="VELOCITY_LOW_CONFIDENCE";
                if(!gray.empty()&&!track.gray.empty()&&track.gray.size()==gray.size()) {
                    cv::Mat safe;
                    cv::erode(track.mask,safe,cv::getStructuringElement(cv::MORPH_ELLIPSE,
                        cv::Size(2*c.safeErodeRadius+1,2*c.safeErodeRadius+1)));
                    std::vector<cv::Point2f> points,next,back;
                    cv::goodFeaturesToTrack(track.gray,points,c.flowMaxCorners,c.flowQuality,5,safe);
                    if(!points.empty()) {
                        std::vector<uchar> forwardStatus,backStatus;std::vector<float> error;
                        try {
                            cv::calcOpticalFlowPyrLK(track.gray,gray,points,next,forwardStatus,error);
                            cv::calcOpticalFlowPyrLK(gray,track.gray,next,back,backStatus,error);
                            std::vector<float> dx,dy;
                            for(size_t k=0;k<points.size();++k)
                                if(forwardStatus[k]&&backStatus[k]&&cv::norm(points[k]-back[k])<=c.flowFBError &&
                                   next[k].x>=0&&next[k].y>=0&&next[k].x<size.width&&next[k].y<size.height) {
                                    dx.push_back(next[k].x-points[k].x);dy.push_back(next[k].y-points[k].y);
                                }
                            pred.flow=dx.size();
                            if(pred.flow>=c.flowMinSupport) {displacement=cv::Point2f(Median(dx),Median(dy));pred.source="LK_FB_MEDIAN";}
                        } catch(const cv::Exception&) {pred.flow=0;pred.source="LK_FAILED_VELOCITY";}
                    }
                }
                // Bound both extrapolation time and its spatial reach.
                const double maximum=c.maxCenterDistance*std::hypot(track.boundingBox.width,track.boundingBox.height);
                const double distance=cv::norm(displacement);
                if(distance>maximum) displacement*=float(maximum/distance);
                cv::Mat affine=(cv::Mat_<double>(2,3)<<1,0,displacement.x,0,1,displacement.y);
                cv::warpAffine(track.mask,pred.mask,affine,size,cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(0));
                pred.box=cv::Rect(track.boundingBox.x+cvRound(displacement.x),track.boundingBox.y+cvRound(displacement.y),
                                  track.boundingBox.width,track.boundingBox.height)&bounds;
                pred.usable=pred.usable&&!pred.box.empty();
            }
            predictions.push_back(pred);
        }
        const int t=ids.size(),d=instances.size(),n=t+d;const double invalid=1e6;
        std::vector<float> depths;for(const auto& in:instances) depths.push_back(Depth(depth,in.mask,c));
        std::vector<bool> merged(d,false);
        if(c.mode>=2) for(int j=0;j<d;++j) {
            int covered=0;
            for(int i=0;i<t;++i) if(predictions[i].usable && mTracks.at(ids[i]).classId==instances[j].classId) {
                cv::Mat inter;cv::bitwise_and(predictions[i].mask,instances[j].mask,inter);
                if(double(cv::countNonZero(inter))/std::max(1,cv::countNonZero(predictions[i].mask))>=c.mergeCoverage) ++covered;
            }
            merged[j]=covered>=2;
        }
        std::vector<std::vector<double>> cost(t,std::vector<double>(d,invalid));
        for(int i=0;i<t;++i) for(int j=0;j<d;++j) {
            const Track& track=mTracks.at(ids[i]);const auto& pred=predictions[i];const auto& in=instances[j];
            if(!pred.usable||merged[j]||track.classId!=in.classId) continue;
            double scale=double(in.boundingBox.area())/std::max(1,pred.box.area());
            double distance=cv::norm(Center(pred.box)-Center(in.boundingBox))/
                std::max(1.,std::hypot(pred.box.width,pred.box.height));
            const double gate=c.maxCenterDistance*(c.mode>=2&&pred.flow<c.flowMinSupport?.75:1.);
            if(scale>c.maxScaleRatio || scale<1/c.maxScaleRatio || distance>gate) continue;
            const double mask=MaskIoU(pred.mask,in.mask),box=BoxIoU(pred.box,in.boundingBox);
            if(mask<c.minMaskIoU && box<c.minBoxIoU) continue;
            double weighted=0,weights=0;int terms=0;
            auto add=[&](double weight,double value,bool evidence=false){if(weight>0){weighted+=weight*value;weights+=weight;if(evidence)++terms;}};
            add(c.maskWeight,1-mask,true);add(c.boxWeight,1-box,true);add(c.centerWeight,std::min(1.,distance/c.maxCenterDistance));
            if(c.mode>=2 && pred.flow>=c.flowMinSupport) add(c.flowWeight,1-std::min(1.,double(pred.flow)/c.flowMaxCorners));
            if(track.depth>0 && depths[j]>0) {
                double limit=std::max(c.maxDepthDifference,c.maxDepthRelative*track.depth);
                double diff=std::abs(track.depth-depths[j]);if(diff>limit) continue;
                add(c.depthWeight,diff/limit,true);
            }
            if(weights>0&&terms>=c.minEvidenceTerms&&weighted/weights<=c.maxCost) cost[i][j]=weighted/weights;
        }
        std::vector<std::vector<double>> matrix(n,std::vector<double>(n,invalid));
        for(int i=0;i<t;++i) {for(int j=0;j<d;++j) matrix[i][j]=cost[i][j];matrix[i][d+i]=c.unmatchedTrackCost;}
        for(int j=0;j<d;++j) {matrix[t+j][j]=c.unmatchedInstanceCost;for(int i=0;i<t;++i)matrix[t+j][d+i]=0;}
        const auto assignment=SolveAssignment(matrix);
        std::vector<int> matched(d,-1);for(int i=0;i<t;++i) if(assignment[i]<d&&cost[i][assignment[i]]<invalid) matched[assignment[i]]=i;
        for(int j=0;j<d;++j) {
            auto& in=instances[j];InstanceAssociationResult a;a.evaluated=true;a.instanceId=j;
            int i=matched[j];bool ambiguous=merged[j];
            if(i<0 && c.mode>=2)
                for(int k=0;k<t;++k) if(cost[k][j]<invalid) ambiguous=true;
            if(i>=0) {
                a.associationCost=cost[i][j];double rowSecond=invalid,columnSecond=invalid;
                for(int k=0;k<d;++k) if(k!=j) rowSecond=std::min(rowSecond,cost[i][k]);
                for(int k=0;k<t;++k) if(k!=i) columnSecond=std::min(columnSecond,cost[k][j]);
                if(rowSecond<invalid) a.rowMargin=rowSecond-cost[i][j];
                if(columnSecond<invalid) a.columnMargin=columnSecond-cost[i][j];
                if(c.mode>=2) ambiguous=ambiguous ||
                    (std::isfinite(a.rowMargin)&&a.rowMargin<c.minMargin) ||
                    (std::isfinite(a.columnMargin)&&a.columnMargin<c.minMargin) ||
                    c.unmatchedTrackCost+c.unmatchedInstanceCost-cost[i][j]<c.minMargin;
                a.prediction=predictions[i].source;a.flowSupport=predictions[i].flow;
            }
            if(i<0||ambiguous) {
                const int id=mNextTrackId++;Track fresh;fresh.id=id;fresh.ambiguousTemplate=ambiguous;mTracks.emplace(id,fresh);
                a.trackId=id;a.associationStatus=ambiguous?AssociationStatus::AMBIGUOUS:AssociationStatus::NEW_TRACK;
                a.reason=merged[j]?"MERGED_INSTANCES":ambiguous?"ASSIGNMENT_AMBIGUOUS":"NO_ELIGIBLE_MATCH";
                mEvents.push_back({id,AssociationStatusName(a.associationStatus),a.reason});
            } else {
                const Track& old=mTracks.at(ids[i]);a.trackId=ids[i];
                a.previousMatchedFrame=old.lastMatchedFrame;a.previousMatchedTimestamp=old.lastMatchedTimestamp;
                const bool adjacent=frameId==old.lastMatchedFrame+1;
                a.associationStatus=adjacent?AssociationStatus::CONTINUOUS_CONFIRMED:AssociationStatus::REACQUIRED;
                a.allowStateInheritance=adjacent||c.mode<3;
                a.allowGeometryPair=adjacent;
                a.resetEvidence=!adjacent&&c.mode>=3;
                a.stateResetReason=a.resetEvidence?"REACQUIRED_RESET_TO_P":"NONE";
                a.reason=adjacent?"CONFIRMED_ADJACENT":"REACQUIRED_MEASUREMENT_BASELINE";
                if(!adjacent) mEvents.push_back({a.trackId,"REACQUIRED",a.stateResetReason});
            }
            Track& track=mTracks.at(a.trackId);
            a.depthAvailable=i>=0&&!ambiguous&&track.depth>0&&depths[j]>0;
            if(track.lastMatchedFrame>=0) track.velocity=(Center(in.boundingBox)-track.center)/float(timestamp-track.lastMatchedTimestamp);
            track.classId=in.classId;track.mask=in.mask.clone();track.boundingBox=in.boundingBox;track.center=Center(in.boundingBox);
            track.gray=gray.clone();track.depth=depths[j];track.lastMatchedFrame=frameId;track.lastMatchedTimestamp=timestamp;track.missedFrames=0;
            in.trackId=a.trackId;in.association=a;
        }
        for(auto it=mTracks.begin();it!=mTracks.end();) {
            it->second.missedFrames=frameId-it->second.lastMatchedFrame;
            if(it->second.missedFrames>mConfig.trackTTL) {
                mEvents.push_back({it->first,"TERMINATED","TRACK_TTL_EXPIRED"});it=mTracks.erase(it);
            } else ++it;
        }
    }
    mHasInput=true;mLastFrame=frameId;mLastTimestamp=timestamp;
    mProcessingMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
}
}
