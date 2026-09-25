# Physical backend: setup, contract and limits

This optional backend preserves the supplied three-joint planar problem.
`planning.py`, the typed services and the PyQt operator GUI are the same for the
software and Gazebo backends. It does not import any other robot's kinematics.

## Environment

Tested with Ubuntu 22.04, ROS 2 Humble and Gazebo Harmonic 8.14.0. Harmonic is
**not Humble's default Gazebo pairing**. This machine already had Harmonic;
the compatible `ros-humble-ros-gzharmonic-bridge` and interfaces packages were
extracted into `.deps/gz_harmonic/opt/ros/humble` without changing system packages.
`scripts/env.sh` exposes that prefix when it exists.

On a new Humble machine, use the official Gazebo package repository and its
Humble/Harmonic installation instructions before installing the bridge:

- [Gazebo ROS installation and pairing](https://gazebosim.org/docs/harmonic/ros_installation/)
- [ROS/Gazebo bridge](https://github.com/gazebosim/ros_gz)

Do not mix a Fortress-linked bridge with Harmonic shared libraries. The optional
dependency is deliberately not forced on the ordinary software-only package.
The current workspace-local bridge uses package versions `0.244.12-3jammy`;
its runtime dependencies also include `actuator_msgs` and `vision_msgs`. The
downloaded packages are development dependencies, not copied into the submission
source archive. A fresh machine needs a compatible bridge installed separately.

```bash
bash scripts/build.sh
source scripts/env.sh
ros2 launch planar_arm_control gazebo.launch.py
# Optional simulator-native GUI:
ros2 launch planar_arm_control gazebo.launch.py gazebo_gui:=true
```

Run only **one** of these at a time in a ROS domain. The launch automatically
uses a matching `GZ_PARTITION=kineshia_<ROS_DOMAIN_ID>` to isolate transport.
The PyQt view requires OpenGL; the Gazebo server itself runs headlessly by default.

## Coordinate contract and model

| Assignment | Gazebo |
|---|---|
| One unspecified length unit | 0.1 m |
| Target (x,y) | World (0.1x, 0, 0.08 + 0.1y) at the tool |
| Link lengths (3,2,1.5) | (0.3,0.2,0.15) m |
| Three positive planar rotations | Three parallel axes along world -Y |
| Joint limits | Identical radians in the physical revolute joints |

A fixed lateral bracket places the tool in the payload plane, 6 cm in front
of the links. This adds **no degree of freedom** and changes neither planar
FK nor target positions. It prevents a link sweeping through the object on the
first approach. The base-height offset lets the rendered finite-radius base
sit on the floor; the planner still enforces the stricter assignment y >= 0
constraint on link segments.

The SDF contains a fixed base, inertial links, three effort-driven position PID
servos, two decorative actuated gripper fingers, gravity, a dynamic payload and
colliding support rails. Masses, gains and scales are illustrative simulation
choices, **not identified hardware parameters**. Arm self-collision is disabled;
not every visual fixture has collision geometry. No general obstacle planner
or complete collision-free workcell certification is claimed.

The place rails sit 5 mm below the nominal cube-bottom release height to avoid
pressing a tilted, attached cube into the support. After release, the cube drops
under gravity and settles near assignment y=2.95, while the commanded tool
release target remains exactly (-3,3). This deliberate clearance and the
physical placement tolerance are reported explicitly; the ideal software
backend places the object exactly at (-3,3).

## Command and feedback flow

```
PyQt request → controller/planner → GazeboBackend → ros_gz_bridge → joint servos
                                   ↑                               ↓
                              physical joints / object pose / attachment state
                                   ↓
                         public telemetry → PyQt + CSV/JSONL observer
```

The bridge maps three Float64 position targets, gripper opening and attach/detach
requests into Gazebo. Measured JointState, physical object pose, attachment
acknowledgement return over ROS. The measured joint header carries simulation
time at 100 Hz, avoiding a separate 1 kHz clock callback. `/joint_commands` remains
reference telemetry; the GUI never publishes low-level actuator commands.

Gazebo's detachable-joint system attaches its child at startup. Admission waits
until the backend has detached it, restored the cube to the pick fixture, and
received fresh physical feedback. Restoring the cube is a one-time nonblocking
native Gazebo `set_pose` request: the installed Humble/Harmonic bridge binary
supports fewer services than newer upstream source. Normal motion, gripper
commands and feedback all use `ros_gz_bridge`.

The trajectory uses simulation time. Wall-time freshness checks detect a
paused/stalled clock or missing feedback; an active command then fails and
requests a hold. Completion requires measured position and velocity within
tolerance for at least 150 ms, and a released object near the place fixture.
Successful completion retains the target; cancellation holds measured position.
These are software safeguards, not a certified physical emergency stop.

## Grasp and workflow limitations

The visible fingers animate opening/closing, but **grasp uses an acknowledged
fixed attachment constraint**, not frictional finger contact. Attachment is
requested only when tool and cube are near the pick point. Detachment releases
the dynamic cube onto the supports under gravity. Object telemetry comes from
Gazebo, never from drawing it at a desired target.

Fixtures are fixed at pick (4,2) and place (-3,3). Other pick/place pairs are
rejected in this backend; arbitrary valid single moves remain available.
Restart the launch to restore the object for another cycle. Software reset and
velocity/PID switching are disabled here: the physical backend has its own
position PID servos, while the original software backend demonstrates the
separate velocity/PID mode. Gripper opening in status is a requested normalized
opening, not a measurement of finger contact force.

## Independent test and recording

```bash
source scripts/env.sh
KINESHIA_ROS_DOMAIN_ID=71 source scripts/env.sh
python scripts/validate_gazebo.py
```

This launches and owns a fresh headless workcell, observes raw physical joint
and object topics, checks both required target cases, then pauses the simulator
mid-command to check stale-clock handling. It stops only its own launched
processes. Results: `artifacts/gazebo_validation.json` and logs.

Beside a **fresh, idle** Gazebo launch on the same domain, record the actual Qt
window for about 70 seconds:

```bash
python -m planar_arm_control.gui_node --gazebo-demo --record artifacts/gazebo-demo.mp4
```

Prefer `gui:=false` on that launch to avoid a duplicate operator window.
