# Repository review: reuse without changing the assignment

Reviewed against `Instructions.docx` and the supplied `Code.zip` on 25 September
2026. These are source-level findings at the commits below, not claims that all
four upstream applications have been installed and run successfully.

## Non-negotiable contract

Three **planar** revolute joints; link lengths 3, 2, 1.5; the stated joint limits
and y >= 0 constraint; unchanged `planar_arm.py`; ROS 2 Humble/Jazzy; PyQt5 /
PyQtGraph; smooth timed trajectories; feedback-driven move/pick/move/place;
the exact (4,2), (-3,3) and projected (7,3) demonstrations. A solid 3D rendering
does not add axes or change this planar robot. Gazebo is a replaceable execution
backend, not another planner.

| Repository | Fit and evidence | Decision |
|---|---|---|
| [attu0/kineshia](https://github.com/attu0/kineshia/tree/27529bc97b0daa4ac6db8f0144c22c0d4a0de755) | Exact starter lineage and arm definition. Supplied kinematics hash matches. Its current controller/GUI are simple examples, not our full constraint-checked architecture. | Keep the already supplied library, not replace our controller. |
| [ros2-robotics-telemetry-suite](https://github.com/Nitin-Chaudhary-081/ros2-robotics-telemetry-suite/tree/7a8186419aa8eaebcecbaa20b4818ecfddb84dd1) | Logging is separable from robot geometry. Arm uses a spatial yaw/pitch/pitch model, different dimensions and topics. | Reuse only the ROS-independent CSV/JSONL helper; adapt our own observer to our typed messages. |
| [doosan-robot-ros2-gui](https://github.com/kimsm0405/doosan-robot-ros2-gui/tree/e9b79cca2d8468bff1ac62ad97213ad3468dae21) | Six-joint Doosan target and a JointTrajectory interface. Inputs are treated as joint positions, not solved as our Cartesian targets. | Do not import robot/control code. Our existing queued-signal GUI boundary is retained. |
| [Bifrost](https://github.com/otherworld-dev/Bifrost/tree/f14bed78138f3c5ad88deb0e2bdd9d0791b210e5) | Useful PyQt5/PyQtGraph/OpenGL presentation reference, but built for a six-axis ThorRR and serial firmware. | Reference only. Original primitive meshes for our three-link geometry; no Thor meshes, firmware or IK copied. |

## Specific findings worth explaining

### Kineshia: starter compatibility is not implementation completeness

SHA-256 of the supplied and upstream `planar_arm.py`:
`1e1c5f4bb5718839da924d192458fb2a679cc25cbded89cccf04381ad16f3dde`.
The upstream GUI sequences operations with delayed Qt timers; an elapsed delay
does not prove the robot reached its target. We retain controller-owned phases
that advance only after measured position/velocity settle. Its package declares
proprietary evaluation-use terms; we do not imply it is generally open-licensed.

### Telemetry: consume the smallest useful, testable component

Copied unchanged:
`robot_telemetry/robot_telemetry/telemetry_csv.py` at the pinned commit.
Its package declares Apache-2.0. License text and source attribution are included
under `src/planar_arm_control/third_party/` and installed with the package.

Our new **separate, read-only recorder node** records `/joint_states`,
`/joint_commands` and phase transitions. It has no command publisher. It writes
source timestamps as well as receive timestamps, logs absent effort as blank
(not an invented current measurement), flushes periodically, and creates a new
session directory every run. This last detail matters: upstream CsvLog opens
with `w`, despite its "append-only" description, so reopening a filename
would truncate it. Tests explicitly cover that behavior.

The recorder improves auditability, not control. Disk I/O is outside the control
process and GUI thread. It is not a hard-real-time or lossless recorder, and no
MCAP, rosbag or mobile-robot analytics support is claimed just from copying it.

### Doosan: a QThread object does not make every method asynchronous

In `src/my_ros2_assignment/my_ros2_assignment/my_node.py`, the button handler
directly calls `worker.execute_move(...)`. That method contains `time.sleep`.
A normal direct Python call still runs in the caller's GUI thread, even though
the object inherits QThread. Also, values are sent as the first three entries
of a six-joint target, not passed to Cartesian IK. Neither pattern should be
copied into this assignment. The custom package declares Apache-2.0; bundled
robot packages have their own terms, which we do not need because none is reused.

### Bifrost: presentation reference, not our robot model

Its README describes a six-axis ThorRR controller, PyQt5 and OpenGL STL rendering.
It declares CC BY-SA 4.0 for repository files. We do not copy those files or
meshes into the evaluation project. A similar *capability*—an orbitable view,
solid links, visible gripper and toolpath—can be implemented directly with
PyQtGraph primitives driven by our supplied FK and real ROS feedback.

## What to say in a review

“I checked geometry, interfaces, concurrency, licensing and failure behavior
before reuse. I reused a small logging component with attribution, not another
robot's controller. The planner and sequence remain independent of rendering
and backend. A realistic-looking model must still display measured state.”

## Running the reused component

```bash
source scripts/env.sh
ros2 launch planar_arm_control bringup.launch.py record_telemetry:=true
```

Or, beside an already running controller:

```bash
ros2 run planar_arm_control telemetry_recorder --ros-args \
  -p output_directory:=artifacts/telemetry
```

Each session contains `joint_states.csv`, `joint_commands.csv`, `events.jsonl`.
Use source timestamps to match measured/reference samples; do not compare
arbitrary callback-arrival neighbors. See `test/test_telemetry.py` for the
vendored-helper tests. End-to-end evidence is recorded separately from this
source-level review.

Offline inspection (no ROS required):

```bash
python scripts/analyze_logs.py artifacts/telemetry/<session-directory>
```

This adapter's analyzer checks timestamp-matched command/feedback rows, tracking
error, publication gaps and recorded phases. It is our own small analyzer, not
the upstream suite's mobile-robot analytics pipeline.
