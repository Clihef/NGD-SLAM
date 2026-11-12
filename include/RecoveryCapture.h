#ifndef NGD_RECOVERY_CAPTURE_H
#define NGD_RECOVERY_CAPTURE_H
#include "Frame.h"
#include "MapPoint.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <chrono>
#include <fstream>
#include <map>
#include <limits>
#include <stdexcept>
namespace ORB_SLAM3 { namespace RecoveryCapture {
inline bool Enabled(long frame) {return std::getenv("NGD_RECOVERY_CAPTURE") && frame>=550 && frame<=580;}
inline std::map<std::string,std::string>& Text() {static std::map<std::string,std::string> v;return v;}
inline std::map<std::string,cv::Mat>& Images() {static std::map<std::string,cv::Mat> v;return v;}
inline std::map<long,double>& Millis() {static std::map<long,double> v;return v;}
using Clock=std::chrono::steady_clock;
inline void Finish(long frame,Clock::time_point start) {Millis()[frame]+=std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
inline cv::Mat Pose(const Sophus::SE3f& p) {cv::Mat m(4,4,CV_32F);for(int i=0;i<4;++i)for(int j=0;j<4;++j)m.at<float>(i,j)=p.matrix()(i,j);return m;}
inline void PnP(long frame,double timestamp,const std::vector<cv::Point3f>& xyz,const std::vector<cv::Point2f>& uv,
 const cv::Mat& K,const cv::Mat& dist,const cv::Mat& r,const cv::Mat& t,const std::vector<int>& inliers,
 bool ok,uint64_t before,uint64_t after) {
 if(!Enabled(frame))return;const auto start=Clock::now();
 cv::FileStorage f("capture.yml",cv::FileStorage::WRITE|cv::FileStorage::MEMORY);
 f<<"frame"<<int(frame)<<"timestamp"<<timestamp<<"xyz"<<xyz<<"uv"<<uv<<"K"<<K<<"dist"<<dist
  <<"rvec"<<r<<"tvec"<<t<<"inliers"<<inliers<<"ok"<<int(ok)<<"rng_before"<<std::to_string(before)<<"rng_after"<<std::to_string(after)
  <<"iterations"<<100<<"reprojection_error"<<5.0<<"confidence"<<.99<<"method"<<int(cv::SOLVEPNP_ITERATIVE);
 Text()["capture_pnp_"+std::to_string(frame)+".yml"]=f.releaseAndGetString();Finish(frame,start);
}
inline void Scene(long frame,const Frame& current,const Frame& last,const Sophus::SE3f& velocity,double interval) {
 if(!Enabled(frame))return;const auto start=Clock::now();
 cv::FileStorage f("capture.yml",cv::FileStorage::WRITE|cv::FileStorage::MEMORY);
 f<<"frame"<<int(frame)<<"timestamp"<<current.mTimeStamp<<"last_timestamp"<<last.mTimeStamp
  <<"seed_Tcw"<<Pose(current.GetPose())<<"last_Tcw"<<Pose(last.GetPose())<<"delta_Tcw"<<Pose(velocity)<<"delta_interval"<<interval
  <<"K"<<current.mK<<"dist"<<current.mDistCoef<<"mb"<<current.mb<<"mbf"<<current.mbf
  <<"min_x"<<current.mnMinX<<"max_x"<<current.mnMaxX<<"min_y"<<current.mnMinY<<"max_y"<<current.mnMaxY
  <<"keys"<<current.mvKeys<<"keys_un"<<current.mvKeysUn<<"descriptors"<<current.mDescriptors
  <<"right_u"<<current.mvuRight<<"inv_sigma2"<<current.mvInvLevelSigma2<<"scale_factors"<<current.mvScaleFactors
  <<"last_keys"<<last.mvKeys<<"last_keys_un"<<last.mvKeysUn;
 cv::Mat xyz(last.N,3,CV_32F,cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
 cv::Mat descriptors=cv::Mat::zeros(last.N,32,CV_8U);std::vector<int> valid(last.N),observations(last.N),outliers(last.N);std::vector<std::string> ids(last.N,"none");
 for(int i=0;i<last.N;++i) {
  outliers[i]=last.mvbOutlier[i];if(auto* p=last.mvpMapPoints[i]) {
   const auto v=p->GetWorldPos();for(int j=0;j<3;++j)xyz.at<float>(i,j)=v[j];
   p->GetDescriptor().copyTo(descriptors.row(i));valid[i]=!p->isBad();observations[i]=p->Observations();ids[i]=std::to_string(p->mnId);
  }
 }
 f<<"last_xyz"<<xyz<<"last_point_descriptors"<<descriptors<<"last_valid"<<valid<<"last_observations"<<observations<<"last_outliers"<<outliers<<"last_point_ids"<<ids;
 Text()["capture_orb_"+std::to_string(frame)+".yml"]=f.releaseAndGetString();Finish(frame,start);
}
inline void Image(long frame,const cv::Mat& rgb,const cv::Mat& mask) {
 if(!Enabled(frame))return;const auto start=Clock::now();
 Images()["capture_rgb_"+std::to_string(frame)+".png"]=rgb.clone();Images()["capture_mask_"+std::to_string(frame)+".png"]=mask*255;Finish(frame,start);
}
// Flush after shutdown so disk writes and PNG encoding do not stall tracking.
inline void Flush() {
 if(!std::getenv("NGD_RECOVERY_CAPTURE"))return;
 for(const auto& v:Text()){std::ofstream f(v.first);f<<v.second;if(!f)throw std::runtime_error("Cannot write recovery capture");}
 for(const auto& v:Images())if(!cv::imwrite(v.first,v.second))throw std::runtime_error("Cannot write recovery image");
 std::ofstream f("capture_timing.csv");f<<"frame,capture_ms\n";for(const auto& v:Millis())f<<v.first<<','<<v.second<<'\n';
}
}}
#endif
