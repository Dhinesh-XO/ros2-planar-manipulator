# Progress and your learning plan

## Assignment requirements

| Requirement | Status / evidence |
|---|---|
| ROS 2 Humble/Jazzy simulation | Implemented and exercised on Humble |
| Target acceptance and unchanged kinematics | Typed service; source hash verified |
| Smooth timed trajectory | Quintic; speed/acceleration reference limits tested |
| Joint limits and ground constraint | IK validation, continuous path certificate, independent witness |
| Replaceable execution backend | Abstract backend and simulation implementation |
| Fixed-rate JointState telemetry | Both modes observed at about 50 Hz |
| PyQt5/PyQtGraph GUI | Arm, all joints, end effector, controls and status |
| Qt and ROS coexistence | Worker executor, queued signals; live demo passed |
| Move/pick/move/place | Completion-driven state machine; order observed over ROS |
| Required coordinates and projected edge case | Demonstrated and independently checked |
| Clean package and launch | Two packages build; normal launch starts both nodes |
| README and design note | Present; design note awaits your ownership review |
| Hardware-transition answer | Draft present; rewrite/review in your own words |
| Git history | Preserved starter followed by implementation milestones |
| 1–3 minute recording | Enhanced software demo 75.2 s; Gazebo demo 70.8 s in artifacts |
| Optional second mode/error plot | Velocity/PID simulation implemented and tested |
| Optional Gazebo | Implemented; physical feedback, object transfer and clock-stall test passed |
| Optional ROS action | Not implemented; typed services/status/cancel remain the interface |
| Solid-link 3D GUI | Implemented in PyQtGraph; unchanged planar geometry and 2D view retained |
| Repository reuse | Isolated telemetry helper integrated with attribution; see REPOSITORY_REVIEW.md |

## Time estimate and focused next steps

The initial 2–3 focused-day estimate includes implementation, learning, validation,
writing and rehearsal for someone refreshing ROS. Assisted implementation has now
produced a working baseline; that does not remove the time needed to understand
and own the submission. Reserve roughly another 6–10 focused hours for the steps
below, adjusting to your comfort level. These are estimates, not deadlines.

1. **ROS and request flow (1–2 hours):** run the GUI, inspect nodes/topics/services,
   send a move from the terminal, and explain acceptance versus completion.
2. **Planning and constraints (1–2 hours):** trace planning.py and run the tests;
   explain quintic timing, projection and why valid endpoints are insufficient.
3. **Execution and GUI (1–2 hours):** compare the two modes, inspect tracking error,
   cancel a motion, and explain thread ownership and timestamp matching.
4. **Submission ownership (2–3 hours):** review the independent CSV evidence,
   rewrite the short notes in your own words, rehearse questions and decide
   whether to record your own narrated demonstration.

The brief specifies Monday 28 September 2026 at 10:00 AM, but does not name the
timezone. It also says five days from receipt and allows extension requests for
employed candidates. Resolve the actual deadline from your correspondence.

No repository has been published and no submission email has been sent. The
remaining human step is understanding/review, not an unimplemented core feature.
