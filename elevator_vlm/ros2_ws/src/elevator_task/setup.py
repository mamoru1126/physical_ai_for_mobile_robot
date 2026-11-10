import os
from glob import glob

from setuptools import find_packages, setup

package_name = "elevator_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "plans"), glob("plans/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mamoru1126",
    maintainer_email="noreply@example.com",
    description="Execute image-and-language instructions on a mobile robot with a VLM and odometry skills",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_node = elevator_task.task_node:main",
            "safety_gate = elevator_task.safety_gate_node:main",
        ],
    },
)
