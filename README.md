# Kineshia Robotics — ROS 2 planar arm

A simulation-only 3-DoF arm with validated quintic motion, a completion-driven
pick/place sequence and a live PyQt5/PyQtGraph operator GUI. Supports ideal
position playback and an illustrative velocity/PID tracking mode.

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
.venv/bin/python -m pip install --no-deps pyqtgraph==0.13.7
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
simulated grasp/release dwell. The object marker follows the tip while held.

The single-target controls default to **(7,3)**. Click **Move to target** to see
projection to approximately **(5.974443,2.560475)**. The GUI shows both markers,
the resolved position and projection status. Projected targets are rejected for
object operations because they must not imply grasping an object elsewhere.

Try **(0,1)** or a negative y to observe rejection. The supplied IK can return
invalid angles; the wrapper rejects them. Cancel stops and holds; reset explicitly
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
`trajectory_duration`. Controller parameters additionally include
`max_velocity=1.5` rad/s, `max_acceleration=3.0` rad/s² and
`grasp_duration=0.7` s. Configuration other than mode is read-only after startup.
These motion limits are simulation choices, not manufacturer specifications.

## Architecture

- `planning.py`: checked IK, direct joint path certification and quintic timing.
- `backends.py`: replaceable execution boundary; ideal position and velocity/PID simulation.
- `controller_node.py`: ROS interfaces, timers, state machine and command admission.
- `gui_node.py`: display, asynchronous requests and Qt/ROS thread boundary.
- `planar_arm_interfaces`: generated typed messages/services; application remains Python.

The controller uses elapsed monotonic time for execution; ROS timestamps label
telemetry. Gazebo `/clock` playback is not supported. There is no Gazebo, URDF,
physical gripper, current/torque simulation, self-collision model or ROS action.
The direct planner may reject a target requiring a different path. A future
hardware backend must supply measured state and device-local stop/watchdog behavior.

## Validate and record

Run with no other controller on the project domain:

```bash
source scripts/env.sh
python -m pytest src/planar_arm_control/test -q
python scripts/validate_ros.py
python /usr/bin/colcon test --packages-select planar_arm_control
python /usr/bin/colcon test-result --verbose
bash scripts/demo.sh artifacts/my-demo.mp4
```

For a machine without a display, prefix the last command with
`QT_QPA_PLATFORM=offscreen`. This still runs and records the actual Qt window.
The 75-second demo runs position pick/place, boundary projection, invalid IK
rejection, velocity/PID pick/place and cancellation. Existing videos are not
overwritten. Generated CSVs, reports, screenshots and video live in `artifacts/`.

The independent ROS witness computes geometry itself from received joint values;
it does not import the controller, planner or supplied kinematics. See
[validation evidence](docs/VALIDATION.md) for measured results and limitations.

## Learning and submission

Start with [learning notes and rehearsal questions](docs/LEARNING_NOTES.md).
Review the [one-page design-note draft](docs/DESIGN_NOTE.md) and
[hardware-transition answer draft](docs/HARDWARE_TRANSITION.md) in your own words.
The assignment permits AI assistance and requires you to explain the work.

Submit a Git repository link, or a ZIP **including .git history**, these written
deliverables, and a 1–3 minute recording. Generated artifacts are Git-ignored;
attach `artifacts/demo.mp4` separately or include it in a submission archive.
Nothing has been emailed or published by the project scripts.
