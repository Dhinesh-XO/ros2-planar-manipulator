# Notes to understand and explain the enhancement

## 1. A 3D view is not a spatial robot

DOF counts independent joint coordinates, not the number of dimensions in a
picture. Our three revolute axes remain parallel. The supplied FK still returns
the same (x,y) points; the view maps them onto a vertical plane and adds thickness.
The camera can orbit that plane. No waist-yaw axis or different link lengths
were borrowed from the example repositories.

## 2. Planning, execution and observation are different responsibilities

The planner answers “what reference trajectory is admissible?” The backend
answers “what actually happened when that reference was executed?” The GUI and
logger are observers and request clients. Removing the GUI does not stop the
controller's sequence. Swapping backends does not replace the planner.

For a demonstration, compare `/joint_commands` and `/joint_states`. Ideal
position playback makes them almost identical by construction. A velocity
servo or a physical Gazebo plant has tracking error. A renderer following the
command alone would conceal that error, so our meshes use feedback.

## 3. Why there are more visible phases

The required move → pick → move → place remains intact. Approach, lift,
transfer and retreat make its motion explicit. All seven motion legs are
preflighted before acceptance; grasp and release are two additional phases.
Each leg is replanned from measured state, then timed with a quintic.

The object finishes at (-3,3); the arm finishes at (-3,3.7) because it retreats.
This distinction matters when writing a test: checking the final tool position
against the object location would incorrectly reject a correct retreat.

## 4. Acceptance, settling and physical confirmation

An accepted service response is not task completion. Follow the command ID and
state transitions. Motion completion checks position AND velocity, continuously
for 150 ms. A single position crossing can occur while the arm is still moving.
Gazebo grasp/release additionally waits for the attachment acknowledgement.
An object that drops away from the place fixture must not produce SUCCEEDED.

On success, retaining the target matters. Replacing it immediately with a noisy
measurement changes the servo's error and can cause a derivative kick. Cancel
is a different operation: stop progressing the path and hold measured position.
Physical braking/watchdogs still belong in a real device backend.

## 5. What repository reuse actually contributed

The attributed CSV/JSONL helper is small, ROS-independent and testable. The
separate recorder adapts it to our typed topics. It revealed a real servo
settling issue during development by exposing command/feedback differences.
Its file-opening behavior was checked rather than assumed from its docstring.

Bifrost informed the choice of a solid OpenGL workcell presentation. Its Thor
meshes, serial interface and six-axis kinematics were not imported. Doosan's
direct method calls demonstrate why merely subclassing QThread is not enough
to move work off the GUI thread. See REPOSITORY_REVIEW.md for pinned evidence.

## 6. What not to overclaim

The Gazebo cube has mass, gravity and support contact; its grasp is a fixed
constraint, not validated friction. Collision geometry is simplified. The
planner certifies the planar ground constraint, not arbitrary obstacles.
The two software control modes and Gazebo position servos are distinct paths;
we do not claim current control, an action server, hard-real-time behavior or
hardware readiness. Logs and recordings are evidence of tested scenarios,
not proofs of every possible target or failure.

Try explaining these aloud: “Why are commands and states separate?”, “Why does
3-DOF not imply a spatial arm?”, “What exactly did I reuse?”, “What happens if
Gazebo stops advancing time?”, and “What would a device-local watchdog add?”
