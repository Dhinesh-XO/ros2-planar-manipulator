# Working notes: understanding and explaining this project

Use these notes alongside the code and your own observations. Before submission,
rewrite the design/hardware notes in your own words and rehearse the demo. AI use
is permitted by the brief; understanding and ownership are still your responsibility.

## 1. The system in one minute

The operator sends a Cartesian target to a ROS service. The controller validates
it, calls the supplied inverse kinematics, validates the returned joint angles,
and constructs a timed joint trajectory. A timer samples that trajectory and
passes references to the simulation backend. Actual and desired joint states are
published separately. The GUI subscribes and renders them. A controller-owned
state machine coordinates move, grasp, move and release.

The GUI is a client. The controller works with the GUI absent. The black-box ROS
validation script proves this by commanding and observing a separate controller
process without importing the planner or opening a window.

## 2. ROS concepts, connected to actual files

| Concept | Meaning here | Where to look |
|---|---|---|
| Workspace | Directory built by colcon, containing packages | Repository root |
| Package | Unit of code/dependencies/install metadata | `src/planar_arm_control` |
| Node | Running participant with a responsibility | Controller and GUI |
| Topic | Stream of observations, with independent subscribers | `/joint_states` |
| Service | Request/response, here admitting a command | `/move_to_target` |
| Message | Typed data contract | `ControllerStatus.msg` |
| Parameter | Configuration of a node | `control_mode`, `publish_rate_hz` |
| Timer | Schedule a callback periodically | 50 Hz execution, 10 Hz status |
| Executor | Runs ready ROS callbacks | Worker threads in controller/GUI |
| Launch file | Starts/configures related processes | `bringup.launch.py` |
| QoS | Delivery/history policy | Latest status retained for late subscribers |

One JointState contains all three joints. Match values by joint name, not by a
guessed order. Positions and velocities use radians and radians/second. GUI plots
convert to degrees. Link lengths and Cartesian coordinates use the brief's
unspecified length unit; do not describe these values as metres without evidence.

The interface package uses ament_cmake to generate ROS types. Application code
remains an ament_python package. No C++ controller was added.

Try in a second terminal, after `source scripts/env.sh`:

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /joint_states --once
ros2 interface show planar_arm_interfaces/srv/MoveTo
ros2 param get /controller_node control_mode
```

Question to answer: why does a successful move service response not mean the arm
has reached the target? Answer: it acknowledges acceptance; status with the same
command ID reports the later execution outcome.

## 3. Kinematics, planning and execution are different jobs

Forward kinematics turns joint angles into link endpoint positions. Inverse
kinematics proposes angles for a target. Neither determines how fast to move.

Our joint path is q(u) = q_start + s(u)(q_goal - q_start), with u=t/T clamped to
[0,1] and s(u)=10u^3-15u^4+6u^5. This quintic starts and ends with zero velocity
and acceleration. T is increased if necessary to satisfy configured reference
speed/acceleration limits. Smoothness alone does not imply safe geometry.

Joint-limit intervals are convex: monotone interpolation between valid joint
angles remains inside them. Ground clearance needs an extra argument. For each
interval along the path, we bound sine of cumulative link angles and therefore
every endpoint's vertical position. Conservative bounds are subdivided. If the
path cannot be certified, it is rejected. This is a direct-path planner, not a
complete obstacle-avoidance planner; rejection does not prove no path exists.

For zero-width straight links, checking both ends above the ground covers the
whole link. Finite link thickness, self-collision and arbitrary obstacles are not
modeled in this assignment implementation.

Read `planning.py`, then run `python -m pytest src/planar_arm_control/test -q`.
Look at the test with valid endpoints but a ground-crossing interpolation. Explain
why checking only the final IK result would miss that failure.

## 4. Two modes and honest feedback

Position mode is ideal kinematic playback. Its actual state equals its commanded
state; it is useful for validating the ROS/planning pipeline but says little about
physical tracking quality.

Velocity/PID mode commands velocity into an illustrative first-order actuator
with a 50 ms time constant. Velocity feedforward plus position PID corrects the
tracking error. Velocity is saturated and the integral is bounded. This is a
simple simulation, not a rigid-body dynamics model or a current/torque controller.

Completion uses measured position and speed tolerances, not only elapsed time.
Velocity mode gets up to three extra seconds to settle. The backend validates
each integrated step and stops on invalid motion. Configured acceleration limits
apply to the reference; this implementation does not claim a physical actuator
acceleration guarantee.

Switch modes only while idle. An actual hardware backend would return measured
encoder values, implement device-local stopping, and handle communication errors.
The planner should not need device register addresses or serial-port logic.

Try both modes and identify which trace is commanded and which is measured. Why
is the error nonzero in velocity mode? What would increasing proportional gain do,
and why would that require retesting stability and limits on physical hardware?

## 5. Concurrency and sequencing

Qt owns the main thread and updates widgets there. A background ROS executor
receives messages and emits queued Qt signals. Service calls are asynchronous;
there are no blocking service waits in GUI callbacks. Plot buffers are bounded.
Command/feedback pairs are matched by source timestamp before computing error.

The controller has two executor threads and separate callback groups for command
planning and timed execution. Shared state is guarded by an RLock. Initial IK
planning happens without holding that lock, so timer publications can continue.
There is a short measured-state replan at the pick-to-place transition. This is
a measured soft-real-time Python application, not a hard-real-time system.

Sequence states: PLANNING -> MOVING_TO_PICK -> PICKING -> MOVING_TO_PLACE ->
PLACING -> SUCCEEDED. Grasp/release are explicit timed simulation states. A move
failure cannot be reported as a completed pick/place. Both paths are preflighted
before starting, and the second segment is checked again from actual state.

Busy commands are rejected, not silently queued. Cancel holds the present pose;
if an object was held, it remains held. Reset is explicitly a simulation reset
and requires no active motion. Reset teleports to the initial pose; it is not a
hardware homing procedure.

## 6. Validation that challenges the implementation

1. Pure tests independently calculate geometry, check trajectory limits and
   endpoints, challenge bad inputs and library failures, and exercise both modes.
2. The ROS witness launches a separate controller, sends real service requests,
   checks published joint limits/ground clearance independently, verifies exact
   sequence order, and measures message timing. It also checks cancellation,
   busy rejection and projected-target reporting.
3. The GUI demonstration runs the actual controls through both modes, projection,
   unsafe-target rejection and cancellation, and records only its own window.

Do not describe passing these as proof of hardware safety. They validate this
simulation under the tested conditions. Read the raw CSV and report in artifacts.
For an experiment, stop the controller and observe the GUI mark telemetry stale.
Then close the GUI during a motion and inspect the controller from another ROS
terminal; control ownership is independent of the window.

## 7. Supplied-library findings to explain candidly

- The Jacobian fallback can return angles outside limits or below the ground.
- An origin target returns the previous configuration without proving reach.
- The analytical branch computes beta but does not apply it; observed IK can
  change a valid current pose unnecessarily and often returns q3=0.
- Projection only handles distance beyond the outer radius, not every constraint
  that makes a target unreachable.

We preserve the file byte-for-byte. The adapter rejects invalid results and checks
Cartesian residual. Already-reached targets retain their current configuration.
We do not claim the adapter repairs the IK algorithm or explores all solutions.
Joint 3 can remain flat in the prescribed demo because of the supplied solver.

## 8. Interview rehearsal

- Trace one request from GUI to service to trajectory to backend to telemetry.
- Explain acceptance versus completion and why command IDs matter.
- Derive why total maximum reach is 6.5 and why (7,3) projects to about (5.974,2.560).
- Explain why a projected target is permitted for a single move but rejected for
  object pick/place: reaching a different point must not imply grasping the object.
- Explain why an action would be a useful next extension: standard feedback,
  cancellation and result semantics replace our service-plus-status convention.
- Describe one measured limitation and one improvement you would prioritize.

References: [ROS interfaces tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html),
[Qt 5 thread ownership and queued connections](https://doc.qt.io/archives/qt-5.15/threads-qobject.html).
