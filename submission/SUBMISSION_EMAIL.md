# Submission format and email

The instruction document accepts a GitHub link or a ZIP containing the repository's
`.git` history. It also requires a one-page design note, a half-page to one-page
simulation-to-hardware answer, and a 1–3 minute screen recording of the fixed scenario.
It does not prescribe an email template or filename convention. The format below
is a suggested way to send those required items.

## Before sending

1. Read the two written answers and confirm that they reflect your understanding.
   Edit anything you would explain differently. Their wording was prepared with
   AI assistance, which the brief permits; the own-words requirement still applies.
2. Make sure the reviewer can access the private repository. If access is not
   arranged, provide the ZIP with `.git` history instead, as an attachment or an
   accessible download link. A private GitHub URL alone does not grant access.
3. Include the two PDFs and the primary video below. The Gazebo video is optional.
   They are also included in the repository and the final ZIP.
4. Confirm the deadline against your correspondence. The brief lists Monday,
   28 September 2026 at 10:00 AM, but does not specify a timezone.

Required files in this folder:

- `design-note.pdf` — one page.
- `simulation-to-hardware.pdf` — one page.
- `media/pick-and-place.mp4` — primary demonstration, 75.2 seconds.
- Repository link or a ZIP with `.git` history.

Use `ROS2_Planar_Manipulator_Submission.zip` for the final full archive, if you
choose the ZIP route. GitHub's ordinary source ZIP download does not contain
`.git` history; use the separately prepared archive instead. To reproduce it
from a reviewed, clean repository, choose a new destination filename:

```bash
/usr/bin/python3 scripts/package_submission.py --tracked-only /path/to/new-submission.zip
```

## Suggested email

To: hr@kineshia.in

Subject: Robotics Software Engineer Assessment Submission — Dhinesh Kannan

Dear Kineshia Robotics Team,

Please find my submission for the Robotics Software Engineer take-home assessment.

Repository: https://github.com/Dhinesh-XO/ros2-planar-manipulator

The submission includes the ROS 2 project and commit history, build and run
instructions, a one-page design note, the simulation-to-hardware answer, and a
short demonstration video.

The video shows pick-and-place from (4,2) to (-3,3) and the outside-workspace
target (7,3). The supplied kinematics file is unchanged. The project was tested
with ROS 2 Humble on Ubuntu 22.04, and the repository includes the validation results.

Thank you for the opportunity. I would be happy to walk through the design and
answer questions about the implementation.

Regards,
Dhinesh Kannan

## Sending note

If using the ZIP route, add its attachment or actual download link before sending.
Do not send inaccessible links or claim that repository access has been granted
until it has been arranged. This is an email template; no email has been sent.
