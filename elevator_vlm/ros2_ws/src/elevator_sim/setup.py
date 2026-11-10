import os
from glob import glob

from setuptools import find_packages, setup

package_name = "elevator_sim"


def tree(directory):
    """directory 以下のファイルを share/<pkg>/<directory>/... にインストールする data_files を作る."""
    out = []
    for path, _, files in os.walk(directory):
        if files:
            out.append((os.path.join("share", package_name, path), [os.path.join(path, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ] + tree("models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mamoru1126",
    maintainer_email="noreply@example.com",
    description="Gazebo Harmonic elevator world, elevator controller, referee and mock VLM",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "elevator_controller = elevator_sim.elevator_controller_node:main",
            "mock_vlm = elevator_sim.mock_vlm_node:main",
            "e2e_runner = elevator_sim.e2e_runner:main",
            "generate_world = elevator_sim.world_gen:main",
        ],
    },
)
