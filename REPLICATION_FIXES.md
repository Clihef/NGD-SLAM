# Replication tracking safety fixes

Baseline: `7f22b750177f711f6d76b1e30a44e458948d27c6` on `replication`.

## Behavior

- RGB-D optical-flow failure forces ORB tracking even if keyframe insertion is blocked by relocalization cooldown, mapper state or queue limits. Keyframe insertion still uses `NeedNewKeyFrame`; recovery and insertion are separate decisions. The first three initialization frames retain their original ORB/KF bootstrap behavior. One input image keeps one frame ID when switching from optical flow to ORB.
- Flow is used only with an initialized map, `OK` state and a valid previous frame. A failed/rejected PnP result is never installed as a pose; ORB reinitializes its motion-model estimate when no valid flow pose exists. PnP settings and matching/velocity thresholds are unchanged.
- `Frame::HasValidPose()` checks that a pose was actually set, all pose values are finite and its quaternion is normalized (numerical tolerance `1e-3`, not an accuracy threshold). `InvalidatePose()` clears the pose flags; `SetPose()` rejects nonfinite/invalid transforms. Lightweight frame metadata is initialized before it can be copied.
- Trajectory recording requires `OK`, a valid current pose, a reference keyframe and finite timestamp/relative transform. Failed frames remain in per-input CSV logs; their poses are omitted from trajectory files. No old pose or old timestamp is duplicated to fill a gap. TUM frame/keyframe export also checks final optimized transforms. Empty-map frame export produces an empty file instead of indexing a nonexistent first keyframe.
- `System::HasValidPose()` reports validity alongside the existing `TrackRGBD`/tracking-state API. A returned SE3 object alone is not a validity indicator. Invalid frames do not update the LK velocity or the displayed current camera pose; downgraded frames are invalidated before copying into tracking history.
- Dynamic point scoring initializes both intensity counters and rejects the complete radius-three border before any pixel access. Empty/mismatched images and nonpositive dynamic-point grid sizes are guarded.
- The existing rotation gate now evaluates `std::abs(rollDeg) > 10`, including negative angles. The original pose-axis formula and 10-degree threshold are retained; this change does not redefine the gate as inter-frame SO(3) rotation.
- Semantic-thread finish state and initial tracking inlier/reference fields are initialized. Detection postprocessing skips empty clipped ROIs, empty depth samples and absent foreground components.

## Logs and evaluation

`NGD_RECORD_EXPERIMENT=1` adds `pose_valid` to `frame_metrics.csv`. With `NGD_DIAGNOSTICS=1`, diagnostic rows also include `pose_valid`, `pose_saved` and `fallback_forced`. The existing `orb_track` now indicates actual ORB execution independently of `need_kf`.

All input frames are still processed and logged. Trajectory coverage can be less than 100% if tracking fails; report coverage and state counts together with ATE/RPE. These are explicitly rejected estimates, not removed input frames. Previously frozen baseline results and diagnostic archives are unchanged.

## Targeted checks

Build against the configured dependency prefix:

```bash
cmake --build build --target rgbd_tum -j 6
./tests/run_tracking_safety_test.sh
./tests/run_mask_sanitizers.sh
```

The test scripts default to `../dependencies/install`; set `NGD_DEPENDENCY_PREFIX` when using another installation. No display server, full TUM replay, accuracy comparison, network or running semantic inference thread is required.

The native regression exercises the actual RGB-D recovery path with empty flow and 31 synthetic keyframes inside a relocalization cooldown; it checks ORB execution, unchanged input frame numbering, retained insertion cooldown and absence of fabricated trajectory rows. It also checks positive/negative rotation gating, invalid-pose recording, frame metadata and dynamic-point selection. ASan/UBSan instruments the exact production pixel-scoring function and sweeps all pixels of a 256x256 image.

Full TUM robustness/accuracy/timing experiments and IMU/stereo/monocular runtime validation remain deferred. These correctness fixes are not evidence of an ATE improvement.
