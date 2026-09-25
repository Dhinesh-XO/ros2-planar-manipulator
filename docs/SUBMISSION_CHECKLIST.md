# Submission readiness and explanation practice

Freeze major features. The most useful remaining work is understanding, an
honest short explanation, and checking the exact files you will send. These
checks improve the submission; they cannot predict the hiring decision.

## Technical handover

- Use the original planar geometry and unchanged supplied library. The solid
  workcell is a presentation/backend enhancement, not a different arm.
- Demonstrate the software backend first: it directly satisfies the assignment.
  Explain Gazebo separately as an optional physics experiment with its own setup.
- Use `enhanced-release-demo.mp4` as the primary 75-second demonstration.
  `gazebo-final-demo.mp4` is supplementary, not required to run the main package.
- Read `SUBMISSION_AUDIT.md` and the raw reports before quoting test results.
  Videos were recorded before the final fault-path hardening; they are not
  evidence that the new failure tests passed. The new reports provide that evidence.
- Include source, `.git` history, README, the short written answers, and video.
  Do not include a local venv, build/install trees, downloaded bridge binaries,
  interrupted logs or failed development recordings as successful evidence.
- A clean extraction on this Ubuntu/Humble machine passed. A different machine,
  Jazzy installation, remote DDS network and real hardware remain unverified.

## Your remaining decisions

- Rewrite/review the design note and hardware answer until every sentence is
  something you can defend. The generated one-page PDFs remain labelled drafts.
- Check the deadline and timezone in the actual assessment correspondence.
- Decide whether to add your own 1–3 minute narration. Do not claim that you
  authored the provided library or the attributed logging helper.
- Confirm the submission channel and recipient yourself. No email, repository
  publication or assessment submission is performed by these scripts.

## Two-minute explanation outline

0:00–0:20 — State the problem: a simulated 3R planar arm, link lengths 3/2/1.5,
joint and ground constraints, supplied kinematics unchanged. Identify which
backend is running.

0:20–0:45 — Trace one request: PyQt sends a typed service request; the controller
validates IK and the path, constructs a quintic reference and owns execution.
Acceptance is immediate admission, not successful completion. Status carries
the command ID and later result.

0:45–1:10 — Show pick/place: completion and grasp/release conditions advance the
phases. Object placement and final tool retreat are different positions. Point
to joint reference, feedback and tracking error; explain why ideal position
feedback overlaps its reference but velocity/PID can lag.

1:10–1:35 — Show projection of (7,3), a negative-y rejection, and cancellation.
Describe one fault test: closing the GUI does not stop the controller; losing
the controller disables GUI commands. A software cancel is not a hardware E-stop.

1:35–2:00 — Explain validation and one limitation: independent geometry witness,
repeat/recovery tests, and preserved-library hash. Gazebo has an acknowledged
attachment grasp, not frictional grasping. The direct planner can conservatively
reject a feasible target; a general obstacle planner was not part of this scope.

## Mock interview: answer before checking the code

1. What is the difference between `/joint_commands` and `/joint_states`? Is
   publishing to `/joint_commands` the way to request a move in this project?
2. Why does an accepted service response not mean the task succeeded?
3. Why are valid initial and final joint configurations insufficient to prove
   the whole motion stays above the ground?
4. Why use measured convergence plus a settling interval, not `sleep(4)`?
5. Which thread can change a Qt widget, and how does ROS deliver data to it?
6. What happens when you cancel while holding an object? What changes after
   restarting the software controller? Why must hardware behave differently?
7. What evidence supports your 50 Hz claim, and why is it not hard real-time?
8. Which exact code was reused, and why were the other robots' mappings rejected?

For each answer use three parts: the design choice, its reason, and one concrete
file/test/observation supporting it. If you cannot explain a sentence, revise the
sentence or study the relevant code; memorizing the outline is not the goal.
