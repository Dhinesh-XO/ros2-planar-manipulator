# Submission-readiness audit — 25 September 2026

This records the engineering audit at that point in the history. The later
documentation closeout rewrote the two answers in plain English and regenerated
their one-page PDFs. Earlier references below to review drafts describe the
audit-time versions. The runtime code and recorded test evidence are unchanged.

Scope: freeze the architecture and robot geometry; improve failure containment,
reproducibility and explanation. This is a simulation assessment, not hardware
safety certification or a prediction of the hiring outcome.

## Findings and fixes

1. A fault-injected failure during the first measured-state replan escaped the
   pick/place service callback. The controller now reports failed admission,
   clears the pending route, holds its last measured pose and accepts a later
   valid request. This was a reproduced exception path, not an observed physical
   collision or a claim that the nominal demonstration routinely failed.
2. On stale feedback the backend requested a hold, but reference telemetry could
   retain the previous moving reference. Stop/failure now updates the reference
   to the same last measured hold pose, with zero requested velocity. No fresh
   joint feedback is invented while Gazebo is stale.
3. Regression tests cover both faults and a cancellation/planning race. The
   race already passed: the command token prevents a late plan from restarting
   a canceled motion. The two new fault tests failed before the fix and passed
   after it. Total unit/regression tests: **22**, not 22 plus the earlier 19.
4. Learning notes now describe all seven motion legs and two grasp/release
   phases. README distinguishes ideal exact placement from Gazebo's 5 mm
   support clearance. Both written-answer PDFs were checked to be one A4 page
   and visually inspected; they remain candidate-review drafts.

Runtime hardening and the audit scripts are committed at `d42119d`. Later
documentation/packaging commits do not change that controller implementation.
The supplied `planar_arm.py` remains unchanged, SHA-256:

```
1e1c5f4bb5718839da924d192458fb2a679cc25cbded89cccf04381ad16f3dde
```

## Independent repeat and interruption checks

`scripts/validate_reliability.py` owns its test processes on an unused ROS domain.
It reuses the independent geometry witness, not the planner or kinematics.
The optional GUI probe opens the real PyQt/OpenGL widgets and only observes them.

- Three complete repeat cycles in each software mode, with explicit simulation
  resets between cycles. Reset during motion is rejected.
- Cancel at APPROACH_PICK, PICKING, LIFTING, TRANSFERRING, PLACING and RETREATING
  in both modes: 12 cases. The pose holds; attachment state is preserved correctly;
  a held object blocks a new sequence until reset. Each case then resets and
  completes a valid recovery move.
- Negative, NaN, infinite and excessive durations are rejected, followed by a
  successful sequence in each mode.
- Force-kill/restart the GUI; the controller completes its existing command and
  the new GUI receives current status. Force-kill/restart the software controller;
  the GUI disables commands when stale, then reconnects to an idle controller.
  Commands are not replayed. Software restart resets the ideal world, not hardware.
- Force-kill the independent recorder while a sequence runs; control remains
  independent. This intentionally interrupted logger is not presented as a
  cleanly closed or lossless recording.

The corrected full run passed **25 checks**. The complete final outcome and individual checks are in
`artifacts/submission-audit/reliability-final/report.json`.
The first audit attempt stopped at a harness error: it invoked the recorder as
a Python module instead of its console-entry-point function. The logger had
never started. That run is **not counted as a pass**; the corrected full run
is the evidence to use.

## Physical backend recheck

On isolated domain 74, the updated controller passed physical pick/place,
attachment then detachment, boundary projection, stale-clock failure and
clock-resume-without-command-replay. It observed 2,616 raw physical joint
messages and 1,309 object poses during the sequence. The released object settled
near (-2.99615, 2.95000), consistent with the documented fixture clearance.

The cleanly closed recorder matched all 5,013 state/reference rows in this run.
Its overall maximum publication gap was about 360 ms, including the deliberately
paused clock and suppressed stale feedback. That fault-injection run must not
be described as uninterrupted 50 Hz telemetry. No torque/effort measurements
were available; empty effort fields are intentional.

Evidence: `artifacts/submission-audit/physics/gazebo_validation.json`, launch log
and the timestamped `telemetry/` session alongside it. Gazebo/bridge were tested
on the existing machine; their dependencies are not bundled or silently installed.

## Clean extraction and packaging scope

The previous review ZIP was extracted into a new temporary workspace. In an
empty shell environment, with user-site Python packages disabled, a new venv
was created with system Python 3.10 and the documented GUI dependencies. Both
packages built, all its original 19 tests passed, and both ROS acceptance modes
passed. Imports resolved inside the extracted workspace, not the development tree.

After hardening, committed source `d42119d` was archived and extracted into a
second new workspace. The same clean-environment procedure built both packages
and passed **22 tests**, followed by passing ROS acceptance in both modes.
Build/import logs and the independently rerun ROS results
are retained under `artifacts/submission-audit/clean-package/`.

This is **clean-workspace validation on the same Ubuntu 22.04/Humble machine**.
The system ROS/Qt/NumPy installation is shared deliberately through
`--system-site-packages`. It is not a clean-OS, Jazzy, remote-network or hardware
validation. Gazebo Harmonic needs a compatible bridge installed separately.

The final ZIP contains committed source, `.git` history, selected successful
evidence, both short review-draft PDFs and the existing good recordings.
`BUNDLE_MANIFEST.json` records file hashes and source provenance. Failed/short
development recordings, local dependencies, venvs and build trees are excluded.
The recordings come from `b0bb113`, before this fault-path patch; they demonstrate
the unchanged normal workflow, while the new reports cover the hardening.

## Reproduce without disturbing an operator

Choose genuinely unused domains; do not run a second controller on a live domain.
The software audit takes several minutes and owns/terminates only its test children.

```bash
KINESHIA_ROS_DOMAIN_ID=72 source scripts/env.sh
python -m pytest src/planar_arm_control/test -q
python scripts/validate_ros.py --output artifacts/my-audit/acceptance
python scripts/validate_reliability.py --gui --output artifacts/my-audit/reliability
# --gui requires a real/OpenGL-capable display; omit for headless repeat tests.

KINESHIA_ROS_DOMAIN_ID=74 source scripts/env.sh
python scripts/validate_gazebo.py --output artifacts/my-audit/physics
/usr/bin/python3 scripts/render_submission_notes.py
# After reviewing and committing the worktree, choose a NEW archive filename:
/usr/bin/python3 scripts/package_submission.py /path/to/new-candidate.zip
```

## Remaining human work

No additional major feature is needed for the defined brief. Review the short
answers in your own words, practise the explanation, confirm the deadline/channel,
and choose the final files to submit. Follow `SUBMISSION_CHECKLIST.md`. Neither
the ownership review nor the mock interview is marked complete on your behalf.
The audit itself did not email the assessment or publish a repository. See the
current project README for the subsequent private GitHub handover.
