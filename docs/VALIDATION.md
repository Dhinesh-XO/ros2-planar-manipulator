# Validation approach and evidence

## Why three forms of evidence

A moving picture can conceal invalid geometry, hard-coded animation or missing
ROS interfaces. Unit tests alone can share assumptions with production code.
We use independent geometry checks, a separate-process ROS witness and an actual
GUI recording. The witness does not import the planner, controller or kinematics.

## Reproduce

Run from the repository root, with no other project controller on domain 67:

```bash
source scripts/env.sh
python -m pytest src/planar_arm_control/test -q
python scripts/validate_ros.py
python /usr/bin/colcon test --packages-select planar_arm_control
python /usr/bin/colcon test-result --verbose
QT_QPA_PLATFORM=offscreen bash scripts/demo.sh artifacts/new-demo.mp4
```

Remove the offscreen variable to show the demo window on your desktop. The
recorder captures this application's window only, not other applications. It
refuses to overwrite an existing video. Rendering and ROS processing continue
while an encoder worker writes frames.

## Recorded development checks (25 September 2026)

- Pure planning/backend tests: 14 passed, including randomized independently
  checked geometry, invalid IK, valid endpoints with unsafe interpolation,
  trajectory derivative limits and distinct position/velocity behavior.
- Separate-process ROS acceptance: both modes passed. Position observed 524
  samples at 49.997 Hz; velocity/PID observed 586 at 49.998 Hz. Maximum observed
  publication intervals were 37.34 ms and 40.00 ms in that run. This is measured
  behavior on this machine, not a deterministic 20 ms deadline guarantee.
- The witness verified the full sequence order, both Cartesian endpoint outcomes,
  projection of (7,3), rejection of invalid/unsafe inputs, rejection while busy,
  rejection of mode changes during execution, and cancellation holding the pose.
- GUI demo passed all five stages. The 1280x900, 10 fps recording is 75.1 seconds
  long. During that run Qt refreshed 1,504 times; the maximum observed refresh gap
  was 109 ms, including startup/recording overhead.

Raw outputs are `artifacts/validation.json`, `telemetry_position.csv`,
`telemetry_velocity_pid.csv`, controller logs, `demo.mp4`, and stage screenshots.
Generated artifacts are intentionally excluded from Git; include the video
separately when submitting a repository link.

## Supplied-file integrity and limitations

SHA-256 of the original and current `planar_arm.py`:

```
1e1c5f4bb5718839da924d192458fb2a679cc25cbded89cccf04381ad16f3dde
```

The witness checks this hash. The adapter does not fix the supplied IK; it rejects
invalid outputs and reports failures. The tests cover this simulation, not hardware
safety, network loss under load, every workspace target, or all possible IK
branches. Continuous path certification is conservative and may reject a feasible
route that would require another joint-space path or intermediate waypoint.
