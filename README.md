# Kineshia Robotics — ROS 2 planar arm

A simulation-only 3-DoF arm with validated quintic motion, a completion-driven
pick/place sequence and a live PyQt5/PyQtGraph operator GUI. Supports ideal
position playback and an illustrative velocity/PID tracking mode. The enhanced
version adds a solid-link 3D workcell, a Gazebo Harmonic backend, approach/lift/
retreat phases, and a separate read-only telemetry recorder.

The supplied `planar_arm.py` is preserved byte-for-byte. Read
[known limitations and validation](docs/VALIDATION.md) before assuming its IK
outputs are safe to execute.

## Setup

Tested on Ubuntu 22.04, ROS 2 Humble and system Python 3.10. Do not use Conda
Python 3.13 with the Humble binary installation.

```bash
# ROS 2 Humble must already be installed and sourced.
# Install missing OS dependencies if needed:
sudo apt install python3-venv python3-pyqt5 python3-numpy python3-pytest \
  python3-colcon-common-extensions ffmpeg

# From this repository's root:
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --no-deps pyqtgraph==0.13.7 PyOpenGL==3.1.7
bash scripts/build.sh
source scripts/env.sh
ros2 launch planar_arm_control bringup.launch.py
```

The helper selects the virtual environment backed by system Python, sources ROS
and the workspace overlay, and defaults to localhost-only ROS domain 67. Source
it in every project terminal. For two-machine operation explicitly set
`KINESHIA_LOCALHOST_ONLY=0` on both machines and use the same
`KINESHIA_ROS_DOMAIN_ID`; network/DDS discovery must also be configured for your
network. This development run validated local communication only.

## Operate

The GUI defaults to pick **(4,2)** and place **(-3,3)**. Click **Run pick and
place**. The controller advances only after measured motion completion and
grasp/release confirmation. The richer sequence is approach → pick → lift →
transfer → place → retreat. In the software backend it ends with the **object
at (-3,3)** and the **tool at (-3,3.7)**. Gazebo commands the same release target;
its object settles about 5 mm lower onto the support (see the documented physical
clearance below). All intermediate targets are checked before admission.
Solid meshes and the orbitable camera are a 3D presentation of the same planar
kinematics, not a new spatial arm. The second view retains the 2D ground plot.

The single-target controls default to **(7,3)**. Click **Move to target** to see
projection to approximately **(5.974443,2.560475)**. The GUI shows both markers,
the resolved position and projection status. Projected targets are rejected for
object operations because they must not imply grasping an object elsewhere.

Try a **negative y** to observe rejection. The supplied IK can return invalid
angles; the wrapper rejects them. Feasibility can depend on the starting pose
and the solver branch; (0,1) is rejected from our initial pose, not necessarily
from every pose. Cancel stops and holds; reset explicitly
resets the simulation to its initial configuration and is permitted only when idle.
Switch `control_mode` while idle using the GUI or the ROS parameter service.

```bash
# Headless controller:
ros2 launch planar_arm_control bringup.launch.py gui:=false control_mode:=velocity_pid

# Typed command from a separate terminal:
ros2 service call /move_to_target planar_arm_interfaces/srv/MoveTo \
  "{target: {x: 4.0, y: 2.0, z: 0.0}, duration: 4.0}"
ros2 service call /pick_place planar_arm_interfaces/srv/PickPlace \
  "{pick: {x: 4.0, y: 2.0}, place: {x: -3.0, y: 3.0}, duration: 4.0}"
ros2 topic echo /controller_status
ros2 service call /cancel_motion std_srvs/srv/Trigger '{}'
ros2 param set /controller_node control_mode velocity_pid
```

An accepted response means the command was admitted, **not completed**. Observe
`controller_status` using the returned command ID. Busy requests are rejected.
Coordinates are in the base frame and in the same unspecified length unit as
the supplied link lengths; z must be zero. Internal joint values are radians.

## Interfaces

| Name | Type | Purpose |
|---|---|---|
| `/joint_states` | sensor_msgs/JointState, 50 Hz | Actual simulated joint feedback |
| `/joint_commands` | sensor_msgs/JointState, 50 Hz | Desired joint reference telemetry, not a command input |
| `/controller_status` | planar_arm_interfaces/ControllerStatus, 10 Hz | Command ID, state, targets, object, error and timer jitter |
| `/planned_path` | nav_msgs/Path, transient-local | Planned tool path for visualization, not a control input |
| `/move_to_target` | planar_arm_interfaces/MoveTo service | Admit a Cartesian move; return projection information |
| `/pick_place` | planar_arm_interfaces/PickPlace service | Preflight and admit a complete sequence |
| `/cancel_motion` | std_srvs/Trigger service | Stop and hold current pose |
| `/reset_simulation` | std_srvs/Trigger service | Reset idle simulation; not a hardware homing command |

Status uses reliable transient-local QoS so a late GUI receives the current
status. State/reference topics use reliable delivery with bounded queues. The
GUI pairs state/reference by timestamp, keeps bounded plot history, and marks
telemetry stale after one second. Qt stays on its main thread; ROS runs in a
worker and uses queued signals.

Launch arguments: `gui`, `control_mode`, `publish_rate_hz`,
`trajectory_duration`, `record_telemetry`, `telemetry_directory`. Controller parameters additionally include
`max_velocity=1.5` rad/s, `max_acceleration=3.0` rad/s² and
`grasp_duration=0.7` s. Configuration other than mode is read-only after startup.
These motion limits are simulation choices, not manufacturer specifications.

## Architecture

- `planning.py`: checked IK, direct joint path certification and quintic timing.
- `backends.py`: replaceable execution boundary; ideal position and velocity/PID simulation.
- `controller_node.py`: ROS interfaces, timers, state machine and command admission.
- `gui_node.py`: display, asynchronous requests and Qt/ROS thread boundary.
- `workcell_view.py`: original primitive meshes driven by measured joint FK.
- `gazebo_backend.py`: bridged physical joint/object feedback and position commands.
- `telemetry_recorder.py`: independent CSV/JSONL observer, no command authority.
- `planar_arm_interfaces`: generated typed messages/services; application remains Python.

The software backend uses monotonic time; Gazebo trajectories use bridged
simulation time while watchdogs use wall time. ROS wall timestamps label paired
state/reference telemetry. Gazebo includes rigid-body dynamics and a detachable
grasp constraint; it does **not** validate frictional grasping. There is no URDF,
current-control mode, complete self-collision model or ROS action.
The direct planner may reject a target requiring a different path. A future
hardware backend must supply measured state and device-local stop/watchdog behavior.

## Optional Gazebo workcell and logging

```bash
source scripts/env.sh
# Requires Gazebo Harmonic and a compatible ros_gz_bridge installation:
ros2 launch planar_arm_control gazebo.launch.py
# Add gazebo_gui:=true for the simulator's own separate view.

# Or use the ordinary software simulation with the reused logging helper:
ros2 launch planar_arm_control bringup.launch.py record_telemetry:=true
```

Do not run two controllers in the same ROS domain. Gazebo has fixed fixtures at
the required pick/place coordinates. Restart that launch to restore the object
for another cycle; the GUI disables the software-only reset and mode switch.
The Gazebo backend runs position servos; the software backend demonstrates the
additional velocity/PID mode. See [Gazebo setup and limitations](docs/GAZEBO.md).

The recorder writes a fresh timestamped folder under `artifacts/telemetry` (or
`artifacts/gazebo_telemetry`). Only a small attributed CSV/JSONL helper was reused
from the repositories you supplied. See [repository review](docs/REPOSITORY_REVIEW.md)
and [third-party notice](src/planar_arm_control/third_party/NOTICE.md).

## Validate and record

Use an unused domain, leaving an operator's live controller alone:

```bash
KINESHIA_ROS_DOMAIN_ID=72 source scripts/env.sh
python -m pytest src/planar_arm_control/test -q
python scripts/validate_ros.py
python scripts/validate_reliability.py --gui
# Physical-backend check on a separate, unused domain:
KINESHIA_ROS_DOMAIN_ID=71 source scripts/env.sh
python scripts/validate_gazebo.py
python /usr/bin/colcon test --packages-select planar_arm_control
python /usr/bin/colcon test-result --verbose
bash scripts/demo.sh artifacts/my-demo.mp4
```

The enhanced 3D view requires a working OpenGL display. Do not use Qt's
`offscreen` platform for the 3D demo; use a real display or an OpenGL-capable
virtual display. The ordinary headless controller and tests need no display.
The 75-second demo runs position pick/place, boundary projection, below-ground
rejection, velocity/PID pick/place and cancellation. Existing videos are not
overwritten. Generated CSVs, reports, screenshots and video live in `artifacts/`.

The independent ROS witness computes geometry itself from received joint values;
it does not import the controller, planner or supplied kinematics. See
[validation evidence](docs/VALIDATION.md) for measured results and limitations.

## Learning and submission

Start with [learning notes and rehearsal questions](docs/LEARNING_NOTES.md).
Use the [submission checklist and two-minute explanation](docs/SUBMISSION_CHECKLIST.md)
for the final review, and [final audit](docs/SUBMISSION_AUDIT.md) for its evidence.
Review the [one-page design-note draft](docs/DESIGN_NOTE.md) and
[hardware-transition answer draft](docs/HARDWARE_TRANSITION.md) in your own words.
The assignment permits AI assistance and requires you to explain the work.

To regenerate printable review drafts after editing (optional documentation tool):

```bash
# Requires the OS package python3-reportlab; not a controller dependency.
/usr/bin/python3 scripts/render_submission_notes.py
```

The PDFs go to `artifacts/submission-notes/`. Review drafts are not a substitute
for your own explanation; this step does not send a submission.

Submit a Git repository link, or a ZIP **including .git history**, these written
deliverables, and a 1–3 minute recording. Generated artifacts are Git-ignored;
attach `artifacts/demo.mp4` separately or include it in a submission archive.
For the enhanced version use `artifacts/enhanced-release-demo.mp4`; the additional
physical-backend recording is `artifacts/gazebo-final-demo.mp4`.
Nothing has been emailed or published by the project scripts.
