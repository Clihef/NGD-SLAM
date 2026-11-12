#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "LocalMapping.h"
#include "YOLO.h"
#include "Diagnostics.h"
#include "KeyFrame.h"

using namespace ORB_SLAM3;
class TrackingProbe : public Tracking {
public:
    using Tracking::Tracking;
    using Tracking::ComputePixelPotential;
    using Tracking::ExtractDynaPoints;
    using Tracking::StoreCurrentFramePose;
    void SetReference(KeyFrame* reference) { mpReferenceKF=reference; }
    void SetCooldown() { mnLastRelocFrameId=Frame::nNextId; mbVelocity=false; }
};

void CheckPixels()
{
    cv::Mat image=cv::Mat::ones(16,16,CV_8U)*100;
    int score;
    for(int y=0;y<16;++y) for(int x=0;x<16;++x) {
        TrackingProbe::ComputePixelPotential(score,cv::Point2f(x,y),image);
        if(x<3 || y<3 || x>=13 || y>=13) assert(score==-1);
    }
    TrackingProbe::ComputePixelPotential(score,cv::Point2f(-1,8),image); assert(score==-1);
    TrackingProbe::ComputePixelPotential(score,cv::Point2f(std::nanf(""),8),image); assert(score==-1);
    image.at<uchar>(8,5)=image.at<uchar>(8,11)=image.at<uchar>(5,8)=150;
    for(int i=0;i<1000;++i) { TrackingProbe::ComputePixelPotential(score,cv::Point2f(8,8),image); assert(score==250); }
    cv::Mat mask=cv::Mat::ones(image.size(),CV_8U);
    std::vector<cv::Point2f> points;
    TrackingProbe::ExtractDynaPoints(points,image,mask,4);
    for(auto point:points) assert(point.x>=3 && point.y>=3 && point.x<13 && point.y<13);
    TrackingProbe::ExtractDynaPoints(points,image,mask,0); assert(points.empty());
    std::cout << "PASS: border pixels, NaN coordinates, deterministic dynamic scores and invalid grid sizes\n";
}

int main(int argc,char** argv)
{
    assert(argc==2);
    setenv("NGD_DIAGNOSTICS","1",1);
    CheckPixels();
    Frame unset(true);
    assert(!unset.HasValidPose() && unset.N==0 && unset.Nleft==-1 && unset.mpCamera==nullptr);
    unset.SetPose(Sophus::SE3f()); assert(unset.HasValidPose());
    Sophus::SE3f nonfinite; nonfinite.translation().x()=std::numeric_limits<float>::quiet_NaN();
    unset.SetPose(nonfinite); assert(!unset.HasPose() && !unset.HasValidPose() && !unset.isSet());
    Frame copied(unset); assert(!copied.HasValidPose());
    std::cout << "PASS: unset/invalidated/NaN poses are rejected; lightweight frame copies have initialized metadata\n";

    const std::string root=argv[1], config=root+"/Examples/RGB-D/TUM3.yaml";
    Settings settings(config,System::RGBD);
    Atlas atlas(0);
    FrameDrawer drawer(&atlas);
    MapDrawer mapDrawer(&atlas,config,&settings);
    LocalMapping mapper(nullptr,&atlas,0,false);
    ORBVocabulary vocabulary;
    TrackingProbe tracker(nullptr,&vocabulary,&drawer,&mapDrawer,&atlas,nullptr,config,System::RGBD,&settings);
    tracker.SetLocalMapper(&mapper);
    YOLO semantic(.5,.4,320,320,root+"/Thirdparty/YOLO/coco.names",root+"/Thirdparty/YOLO/yolo-fastest-xl.cfg",root+"/Thirdparty/YOLO/yolo-fastest-xl.weights","test");
    assert(!semantic.CheckFinish()); tracker.SetYOLO(&semantic);

    Frame seed(true); seed.mpORBvocabulary=&vocabulary; seed.SetPose(Sophus::SE3f()); seed.mTimeStamp=1;
    KeyFrame* reference=nullptr;
    for(int i=0;i<31;++i) { reference=new KeyFrame(seed,atlas.GetCurrentMap(),nullptr); atlas.AddKeyFrame(reference); }
    assert(atlas.KeyFramesInMap()==31);
    tracker.SetReference(reference); tracker.SetCooldown();
    tracker.mState=Tracking::OK; tracker.mFrameNum=4;
    tracker.mLastFrame=Frame(seed); tracker.mLastFrameLK=Frame(seed);
    tracker.mImGrayLastKey=tracker.mImGrayLastLK=cv::Mat::zeros(480,640,CV_8U);
    tracker.mImMaskLastKey=cv::Mat::zeros(480,640,CV_8U);
    const cv::Mat rgb=cv::Mat::zeros(480,640,CV_8UC3), depth=cv::Mat::ones(480,640,CV_16U)*5000;
    const auto nextID=Frame::nNextId;
    tracker.GrabImageRGBD(rgb,depth,2,"");
    assert(Diagnostic().flow_ok==0 && Diagnostic().orb_track==1 && Diagnostic().fallback_forced==1);
    assert(!tracker.mbNeedKF); // Cooldown still prevents KF insertion, not tracking recovery.
    assert(tracker.mCurrentFrame.mnId==nextID && Frame::nNextId==nextID+1);
    assert(tracker.mState!=Tracking::OK && !tracker.mCurrentFrame.HasValidPose());
    assert(tracker.mlRelativeFramePoses.empty() && tracker.mlFrameTimes.empty());
    std::cout << "PASS: real RGB-D path executes ORB on empty optical flow during a 30-frame KF cooldown; invalid frame is not recorded\n";

    for(float angle : {-20.0f,20.0f,-5.0f}) {
        tracker.mState=Tracking::OK;
        tracker.mLastFrameLK=Frame(seed);
        tracker.mLastFrameLK.SetPose(Sophus::SE3f(Eigen::Quaternionf(Eigen::AngleAxisf(angle*float(CV_PI)/180.0f,Eigen::Vector3f::UnitZ())),Eigen::Vector3f::Zero()));
        tracker.mImMaskLastKey=cv::Mat::zeros(480,640,CV_8U);
        tracker.mImMaskLastKey(cv::Rect(20,20,10,10)).setTo(1);
        semantic.mOutputPair={cv::Mat::zeros(480,640,CV_8U),cv::Mat::zeros(480,640,CV_8U)};
        tracker.GrabImageRGBD(rgb,depth,tracker.mFrameNum,"");
        assert(Diagnostic().mask_rejected==(std::abs(angle)>10));
    }
    std::cout << "PASS: actual semantic-mask gate handles positive/negative large rotation and small rotation\n";

    tracker.mCurrentFrame=Frame(seed); tracker.mCurrentFrame.mpReferenceKF=reference;
    tracker.mState=Tracking::OK; assert(tracker.StoreCurrentFramePose());
    tracker.mState=Tracking::RECENTLY_LOST; assert(!tracker.StoreCurrentFramePose());
    tracker.mState=Tracking::OK; tracker.mCurrentFrame.mpReferenceKF=nullptr; assert(!tracker.StoreCurrentFramePose());
    tracker.mCurrentFrame.mpReferenceKF=reference; tracker.mCurrentFrame.mTimeStamp=std::numeric_limits<double>::infinity(); assert(!tracker.StoreCurrentFramePose());
    tracker.mCurrentFrame.mTimeStamp=3; tracker.mCurrentFrame.SetPose(nonfinite); assert(!tracker.StoreCurrentFramePose());
    assert(tracker.mlRelativeFramePoses.size()==1 && tracker.mlpReferences.size()==1 && tracker.mlFrameTimes.size()==1 && tracker.mlbLost.size()==1);
    std::cout << "PASS: recording rejects downgraded state, null reference, nonfinite timestamp and invalid pose without fabricated entries\n";
}
