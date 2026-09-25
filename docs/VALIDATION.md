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
bash scripts/demo.sh artifacts/new-demo.mp4
```

The enhanced OpenGL view requires a real or OpenGL-capable virtual display.
Do not use Qt's offscreen platform for its 3D recording. The
recorder captures this application's window only, not other applications. It
refuses to overwrite an existing video. Rendering and ROS processing continue
while an encoder worker writes frames.

## Baseline checks (25 September 2026, tag baseline-v1)

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

## Enhanced workcell and selective reuse checks

- `pytest` and `colcon test`: **19 tests passed**. Added full-route preflight,
  all-three-joint motion, and three tests of the unchanged vendored logging helper.
- Enhanced independent ROS witness: both modes passed. Latest recorded run:
  position 892 samples at 49.832 Hz (maximum interval 103.03 ms), velocity/PID
  1,001 samples at 50.000 Hz (maximum 39.64 ms). This run overlapped physical
  backend testing; no hard-real-time deadline claim is made.
- Sequence checks now distinguish object placement (-3,3) from final tool
  retreat (-3,3.7), require all nine phases, and verify all three joints move.
- Enhanced software GUI recording: `artifacts/enhanced-release-demo.mp4`,
  75.2 seconds, 1280x900 at 10 fps. All five stages passed; 1,487 GUI ticks,
  maximum refresh gap 119 ms, maximum window capture 52 ms and zero encoder
  queue drops in that final run.
- Physical witness: `artifacts/gazebo_validation.json` reports success,
  observing 2,620 raw physical joint messages and 1,310 object poses during the
  sequence. Attachment then detachment were acknowledged. The released cube
  settled near (-2.99613,2.95000), consistent with the documented 5 mm fixture
  release gap. Projection and mid-command simulation-clock-stall rejection passed.
- Gazebo GUI recording: `artifacts/gazebo-final-demo.mp4`, 70.8 seconds. The
  sequence, projection, below-ground rejection and cancellation passed. With
  profiling enabled, 1,094 GUI refreshes were observed; maximum gap 177 ms.
- The reused recorder's latest physical-test session matched all 4,983 actual/reference
  CSV rows by exact source stamp and joint name, with no unmatched actual rows
  in that run. This is evidence for that recording, not a lossless logging claim.

Development failures were useful and are not counted as passes: a link initially
swept into the cube; high wrist gains oscillated near extension; the first
physical recording suffered GUI stalls while raw joint/clock callbacks ran at
1 kHz. The corrected fixed lateral bracket, lower servo gains, proper successful
hold behavior, release clearance and 100 Hz joint publisher were retested.
The physical backend now samples simulation time from the joint header instead
of a separate 1 kHz Python clock subscription. Earlier failed/short recordings
remain development artifacts, not submission evidence.

Recording now preserves wall time across missed captures by holding the previous
frame, instead of silently speeding up a slow capture run. Such held frames do
not conceal the reported GUI timing metrics. The original validated videos above
were recorded without significant capture gaps after the rate correction.

Reproduce the physical test on a separate domain:

```bash
source scripts/env.sh
KINESHIA_ROS_DOMAIN_ID=71 source scripts/env.sh
python scripts/validate_gazebo.py
```

See [Gazebo limitations](GAZEBO.md), [reuse review](REPOSITORY_REVIEW.md), and
[learning notes for the enhancement](ENHANCEMENT_NOTES.md).

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
