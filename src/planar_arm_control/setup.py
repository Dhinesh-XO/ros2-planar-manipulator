import os
from glob import glob

from setuptools import find_packages, setup

package_name = "planar_arm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "third_party"), glob("third_party/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kineshia Robotics",
    maintainer_email="hr@kineshia.in",
    description="ROS 2 control + telemetry GUI for a 3-DoF planar manipulator.",
    license="Proprietary — for evaluation use only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller_node = planar_arm_control.controller_node:main",
            "gui_node = planar_arm_control.gui_node:main",
            "telemetry_recorder = planar_arm_control.telemetry_recorder:main",
        ],
    },
)
