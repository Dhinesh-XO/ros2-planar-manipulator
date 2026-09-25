# Assessment deliverables

This folder is deliberately tracked in Git, unlike temporary `artifacts/`.
A repository clone contains the required written notes and screen recordings;
no separate download of a local build tree or simulator dependency is needed.

| Requested deliverable | Location |
|---|---|
| Source and commit history | Repository root, `src/`, `scripts/`, and Git history |
| One-page design note | [PDF](design-note.pdf) · [editable Markdown](../docs/DESIGN_NOTE.md) |
| Part 2: simulation to hardware, one page | [PDF](simulation-to-hardware.pdf) · [editable Markdown](../docs/HARDWARE_TRANSITION.md) |
| Primary screen recording, 75.2 seconds | [Software pick-and-place demonstration](media/pick-and-place.mp4) |
| Optional physical-simulation recording, 70.8 seconds | [Gazebo demonstration](media/gazebo-pick-and-place.mp4) |
| Build/run instructions and interfaces | [Project README](../README.md) |
| Validation and limitations | [Submission audit](../docs/SUBMISSION_AUDIT.md) |

The written answers remain explicitly labelled **review drafts** until the
candidate has checked them in their own words. Implementation/test completion
does not replace that requirement in the brief.

## What the recordings show

The primary recording demonstrates pick (4,2), place (-3,3), projection of (7,3),
negative-y rejection, the two software control modes and cancellation. The 3D
view retains the supplied planar geometry. The object finishes at the place
point and the tool retreats above it.

The optional Gazebo recording shows the bridged physical simulation. Its grasp
is an attachment constraint, not frictional contact. The payload settles about
5 mm below the nominal release height onto the support. See [Gazebo notes](../docs/GAZEBO.md).

Both recordings were made at commit `b0bb113`, before final fault-path hardening.
They demonstrate the unchanged normal workflow; the later tests below cover
the hardening at `d42119d`.

## Selected evidence

- [Clean build and 22 tests](evidence/clean-build-and-test.log), followed by
  [build/test of the final candidate ZIP](evidence/final-zip-build-and-test.log).
- [Independent software acceptance](evidence/software-acceptance.json), with
  [position samples](evidence/telemetry-position.csv) and
  [velocity/PID samples](evidence/telemetry-velocity-pid.csv).
- [25 repeat/recovery checks](evidence/reliability.json): repeated sequences,
  cancellation, GUI/controller restart and recorder failure.
- [Gazebo acceptance](evidence/gazebo.json): physical object transfer,
  projection, stalled-clock failure and no automatic replay on clock resume.

These results concern the tested Ubuntu 22.04 / Humble simulation environment,
not real hardware, Jazzy or a fresh operating-system installation. Absolute
workspace paths in raw evidence identify the actual test environment.

## Verify the handover

From the repository root, without ROS running:

```bash
/usr/bin/python3 scripts/prepare_submission.py --check
```

[manifest.json](manifest.json) records evidence hashes and provenance.
This verifies file integrity, not a new execution of the robot tests.

Developers can regenerate the PDFs with `scripts/render_submission_notes.py`,
then export the allowlisted local evidence with `scripts/prepare_submission.py`.
Review and commit those changes; do not silently remove the draft labels or
replace failed evidence with an assertion of success.
