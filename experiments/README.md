# B3: B0/B3 controlled comparison

This branch identifies the complete tracking safety/recovery fixes plus the common read-only observer. B1/B2 ablations are not included.

`pose_observations.csv` records each input's online camera-to-world pose, actual final solver success, mathematical validity, tracking state, native recording decision and common acceptance. The observer never changes the frame pose, tracking state, mask, velocity, keyframe decision or recovery policy. `accepted` is an offline evaluation label, independent of whether the native trajectory exporter saved a pose. This avoids counting B0's stale-OK/default-pose outputs as valid coverage.

`NGD_DIAGNOSTICS=1` enables exactly the same observation header and CSV schema in both branches. `NGD_RNG_SEED` sets the C and OpenCV global RNGs after System initialization in both versions; unset preserves existing behavior. Thread scheduling remains asynchronous, so a common seed is not a guarantee of identical solver inputs or trajectories. Logging and randomized repeat blocks control that remaining variation; no run is selected or silently retried.

The prespecified evaluation is in `protocol_b0_b3.json`: RPY 10 runs per version; xyz/halfsphere/static 5 per version; one fixed GT association and quality denominator; one initial two-second online calibration; qualification at 5/10/20 cm and 5/10 degrees; recovery sustained for five frames, with unresolved events reported. Final-trajectory ATE/RPE are reported separately from online full-sequence quality. Common-support accuracy remains a conditional metric.

Version B0 intentionally retains known pre-fix behavior and defects for comparison; it is not the deployment recommendation. This comparison cannot attribute an improvement to one individual fix without the deferred ablations.
