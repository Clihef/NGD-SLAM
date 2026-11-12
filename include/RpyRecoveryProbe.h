#ifndef NGD_RPY_RECOVERY_PROBE_H
#define NGD_RPY_RECOVERY_PROBE_H
#include "Frame.h"
#include "MapPoint.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include <fstream>
#include <iomanip>
#include <memory>
#include <unordered_map>
namespace ORB_SLAM3 { namespace RecoveryProbe {
inline bool Enabled(long frame) {return std::getenv("NGD_RECOVERY_PROBE") && frame>=550 && frame<=575;}
inline void Pose(long frame,double timestamp,const char* stage,const Sophus::SE3f& Tcw,int matches,int inliers,double lastOrb,double lastLK) {
 if(!Enabled(frame))return;
 static std::ofstream f("recovery_stage_poses.csv");static bool header=false;
 if(!header){f<<"frame,timestamp,stage,matches,inliers,last_orb_timestamp,last_lk_timestamp,tx,ty,tz,qx,qy,qz,qw\n";header=true;}
 const auto T=Tcw.inverse();const auto q=T.unit_quaternion();const auto t=T.translation();
 f<<std::setprecision(17)<<frame<<','<<timestamp<<','<<stage<<','<<matches<<','<<inliers<<','<<lastOrb<<','<<lastLK<<','<<t.x()<<','<<t.y()<<','<<t.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n';
}
class FrozenPoint : public MapPoint {
public:
 explicit FrozenPoint(MapPoint* p) {mnId=p->mnId;mWorldPos=p->GetWorldPos();mDescriptor=p->GetDescriptor().clone();nObs=p->Observations();mbBad=p->isBad();mpMap=nullptr;mpRefKF=nullptr;mNormalVector.setZero();mfMinDistance=0;mfMaxDistance=1000;}
};
struct Candidate {Sophus::SE3f pose;int matches=0,inliers=0;std::vector<unsigned long> assignments;};
inline Candidate Project(Frame current,const Frame& last,const Sophus::SE3f& seed,long frame,const char* kind,bool dump) {
 current.SetPose(seed);std::fill(current.mvpMapPoints.begin(),current.mvpMapPoints.end(),nullptr);ORBmatcher matcher(.9,true);Candidate result;
 result.matches=matcher.SearchByProjection(current,last,15,false);
 if(result.matches<20){std::fill(current.mvpMapPoints.begin(),current.mvpMapPoints.end(),nullptr);result.matches=matcher.SearchByProjection(current,last,30,false);}
 if(result.matches>=20)Optimizer::PoseOptimization(&current);
 result.pose=current.GetPose();
 static std::ofstream correspondences("recovery_correspondences.csv");static bool header=false;
 if(dump&&!header){correspondences<<"frame,kind,index,point_id,u,v,octave,right_u,inv_sigma2,x,y,z,outlier\n";header=true;}
 for(int i=0;i<current.N;++i) if(auto* p=current.mvpMapPoints[i]) {
  result.assignments.push_back(p->mnId);
  if(!current.mvbOutlier[i]&&p->Observations()>0)++result.inliers;
  if(dump){const auto xyz=p->GetWorldPos();const auto& key=current.mvKeysUn[i];correspondences<<std::setprecision(17)<<frame<<','<<kind<<','<<i<<','<<p->mnId<<','<<key.pt.x<<','<<key.pt.y<<','<<key.octave<<','<<current.mvuRight[i]<<','<<current.mvInvLevelSigma2[key.octave]<<','<<xyz.x()<<','<<xyz.y()<<','<<xyz.z()<<','<<current.mvbOutlier[i]<<'\n';}
 }
 return result;
}
// Freeze geometry/descriptors/counts so the two predictions see exactly the same map and image features.
inline void Compare(long frame,const Frame& original,const Frame& previous,const Frame& lastLK,
                    const Sophus::SE3f& legacySeed,const Sophus::SE3f& scaledSeed,const Sophus::SE3f& lkSeed) {
 if(!Enabled(frame)||original.Nleft!=-1)return;
 Frame frozenLast(previous),frozenCurrent(original);std::unordered_map<MapPoint*,std::unique_ptr<FrozenPoint>> points;
 for(auto*& p:frozenLast.mvpMapPoints) if(p){auto it=points.find(p);if(it==points.end())it=points.emplace(p,std::unique_ptr<FrozenPoint>(new FrozenPoint(p))).first;p=it->second.get();}
 std::fill(frozenCurrent.mvpMapPoints.begin(),frozenCurrent.mvpMapPoints.end(),nullptr);
 const auto a=Project(frozenCurrent,frozenLast,legacySeed,frame,"legacy",true);
 const auto b=Project(frozenCurrent,frozenLast,lkSeed,frame,"lk",true);
 const auto scaled=Project(frozenCurrent,frozenLast,scaledSeed,frame,"orb_scaled",true);
 const auto repeatA=Project(frozenCurrent,frozenLast,legacySeed,frame,"legacy_repeat",false);
 const auto repeatB=Project(frozenCurrent,frozenLast,lkSeed,frame,"lk_repeat",false);
 const bool deterministic=a.assignments==repeatA.assignments&&b.assignments==repeatB.assignments&&
 (a.pose.matrix()-repeatA.pose.matrix()).norm()<1e-6&&(b.pose.matrix()-repeatB.pose.matrix()).norm()<1e-6;
 Pose(frame,original.mTimeStamp,"legacy_seed",legacySeed,0,0,previous.mTimeStamp,lastLK.mTimeStamp);
 Pose(frame,original.mTimeStamp,"lk_seed",lkSeed,0,0,previous.mTimeStamp,lastLK.mTimeStamp);
 Pose(frame,original.mTimeStamp,"orb_scaled_seed",scaledSeed,0,0,previous.mTimeStamp,lastLK.mTimeStamp);
 Pose(frame,original.mTimeStamp,"orb_scaled_frozen_optimized",scaled.pose,scaled.matches,scaled.inliers,previous.mTimeStamp,lastLK.mTimeStamp);
 Pose(frame,original.mTimeStamp,"legacy_frozen_optimized",a.pose,a.matches,a.inliers,previous.mTimeStamp,lastLK.mTimeStamp);
 Pose(frame,original.mTimeStamp,"lk_frozen_optimized",b.pose,b.matches,b.inliers,previous.mTimeStamp,lastLK.mTimeStamp);
 static std::ofstream f("recovery_frozen_repeats.csv");static bool header=false;
 if(!header){f<<"frame,point_count,legacy_matches,legacy_inliers,lk_matches,lk_inliers,deterministic\n";header=true;}
 f<<frame<<','<<points.size()<<','<<a.matches<<','<<a.inliers<<','<<b.matches<<','<<b.inliers<<','<<deterministic<<'\n';
}
}}
#endif
