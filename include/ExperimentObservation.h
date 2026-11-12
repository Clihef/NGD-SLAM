#ifndef NGD_EXPERIMENT_OBSERVATION_H
#define NGD_EXPERIMENT_OBSERVATION_H
#include "Diagnostics.h"
#include <opencv2/core.hpp>
#include "sophus/se3.hpp"
#include <cmath>
#include <limits>
#include <string>
namespace ORB_SLAM3 {
// Read-only experiment instrumentation: never feeds an acceptance decision back to tracking.
inline void InitializeExperimentSeed() {
    if(const char* value=std::getenv("NGD_RNG_SEED")) {
        const unsigned seed=static_cast<unsigned>(std::stoul(value));
        std::srand(seed); cv::setRNGSeed(static_cast<int>(seed));
    }
}
inline void ObservePose(long frame,double timestamp,int state,bool hasPose,const Sophus::SE3f& Tcw,
                        unsigned long mapId,bool nativeRecorded) {
    if(!DiagnosticEnabled()) return;
    auto& d=Diagnostic();
    const bool finite=Tcw.matrix().allFinite() && std::abs(Tcw.unit_quaternion().squaredNorm()-1.0f)<1e-3f;
    const bool solverOK=d.orb_track ? d.orb_ok==1 : d.flow_ok==1;
    const bool accepted=state==2 && hasPose && finite && solverOK;
    d.pose_valid=accepted; d.pose_saved=nativeRecorded;
    static std::ofstream out("pose_observations.csv");
    static bool header=false;
    if(!header) {out << "frame,timestamp,state,pose_set,pose_finite,flow_ok,orb_track,orb_ok,accepted,native_recorded,pose_source,map_id,tx,ty,tz,qx,qy,qz,qw\n";header=true;}
    Eigen::Vector3f t=Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
    Eigen::Vector4f q=Eigen::Vector4f::Constant(std::numeric_limits<float>::quiet_NaN());
    if(finite) {const auto Twc=Tcw.inverse();t=Twc.translation();q=Twc.unit_quaternion().coeffs();}
    out << std::setprecision(17) << frame << ',' << timestamp << ',' << state << ',' << hasPose << ',' << finite << ','
        << d.flow_ok << ',' << d.orb_track << ',' << d.orb_ok << ',' << accepted << ',' << nativeRecorded << ','
        << (d.orb_track ? "orb" : (d.flow_ok>=0 ? "flow" : "none")) << ',' << mapId << ','
        << t.x() << ',' << t.y() << ',' << t.z() << ',' << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << '\n';
}
}
#endif
