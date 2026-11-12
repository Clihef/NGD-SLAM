#ifndef NGD_DIAGNOSTICS_H
#define NGD_DIAGNOSTICS_H
#include <cstdlib>
#include <fstream>
#include <iomanip>
namespace ORB_SLAM3 {
struct FrameDiagnostic {
    int pose_valid=0, pose_saved=0, fallback_forced=0;
    int state_before=-1, flow_input=-1, flow_status=-1, flow_matches=-1, pnp_ok=-1, pnp_inliers=-1, velocity_reject=-1, flow_ok=-1;
    int orb_track=0, ref_matches=-1, motion_matches=-1, kf_created=0, reloc_called=0, reloc_ok=-1;
    long semantic_frame=-1, mask_source=-1;
    double semantic_timestamp=-1, mask_timestamp=-1, rotation_gate_deg=0;
    int mask_rejected=0, dynamic_input=-1, dynamic_tracked=-1, mask_pixels=0;
};
inline bool DiagnosticEnabled() { static bool enabled=std::getenv("NGD_DIAGNOSTICS")!=nullptr; return enabled; }
inline FrameDiagnostic& Diagnostic() { static thread_local FrameDiagnostic value; return value; }
inline void WriteDiagnostic(long frame, long internal_frame, double timestamp, int state, int matches, bool need_kf) {
    if(!DiagnosticEnabled()) return;
    static std::ofstream stream("tracking_diagnostics.csv");
    static bool header=false;
    if(!header) { stream << "frame,internal_frame,timestamp,state_before,state_after,flow_input,flow_status,flow_matches,pnp_ok,pnp_inliers,velocity_reject,flow_ok,need_kf,orb_track,ref_matches,motion_matches,matches_final,kf_created,reloc_called,reloc_ok,semantic_frame,semantic_timestamp,mask_source,mask_timestamp,rotation_gate_deg,mask_rejected,dynamic_input,dynamic_tracked,mask_pixels,pose_valid,pose_saved,fallback_forced\n"; header=true; }
    const auto& d=Diagnostic();
    stream << std::setprecision(17) << frame << ',' << internal_frame << ',' << timestamp << ',' << d.state_before << ',' << state << ',' << d.flow_input << ',' << d.flow_status << ',' << d.flow_matches << ',' << d.pnp_ok << ',' << d.pnp_inliers << ',' << d.velocity_reject << ',' << d.flow_ok << ',' << need_kf << ',' << d.orb_track << ',' << d.ref_matches << ',' << d.motion_matches << ',' << matches << ',' << d.kf_created << ',' << d.reloc_called << ',' << d.reloc_ok << ',' << d.semantic_frame << ',' << d.semantic_timestamp << ',' << d.mask_source << ',' << d.mask_timestamp << ',' << d.rotation_gate_deg << ',' << d.mask_rejected << ',' << d.dynamic_input << ',' << d.dynamic_tracked << ',' << d.mask_pixels << ',' << d.pose_valid << ',' << d.pose_saved << ',' << d.fallback_forced << '\n';
}
}
#endif
