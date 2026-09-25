# Third-party code

`planar_arm_control/vendor/telemetry_csv.py` is copied **unchanged** from
[Nitin-Chaudhary-081/ros2-robotics-telemetry-suite](https://github.com/Nitin-Chaudhary-081/ros2-robotics-telemetry-suite/blob/7a8186419aa8eaebcecbaa20b4818ecfddb84dd1/robot_telemetry/robot_telemetry/telemetry_csv.py),
commit `7a8186419aa8eaebcecbaa20b4818ecfddb84dd1`.

Upstream path: `robot_telemetry/robot_telemetry/telemetry_csv.py`.
The upstream `robot_telemetry/package.xml` declares **Apache-2.0**. A copy
of that license accompanies this notice as `Apache-2.0.txt`. No separate
upstream NOTICE or file-specific copyright statement was present. Attribution
is to the upstream project and its contributors; no authorship is claimed here.

Only this ROS-independent CSV/JSONL helper is vendored. Our ROS topic adapter,
tests, controller, planner, GUI geometry and simulator model are separate code.
The other reviewed repositories contributed no copied code or mesh assets.

The supplied `planar_arm.py` retains its original evaluation-use terms and is
unchanged. Vendoring this helper does not relicense the assignment starter.
