from glob import glob
from setuptools import find_packages, setup

package_name = "a0509_vr_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="salim2001",
    maintainer_email="salim2001@example.com",
    description="MetaQuest VR teleoperation pipeline for Doosan A0509 ServoL RT control.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "quest_gateway_node = a0509_vr_teleop.quest_gateway_node:main",
            "mock_quest_input_node = a0509_vr_teleop.mock_quest_input_node:main",
            "vr_frame_mapper_node = a0509_vr_teleop.vr_frame_mapper_node:main",
            "safety_guard_node = a0509_vr_teleop.safety_guard_node:main",
            "servol_rt_streamer_node = a0509_vr_teleop.servol_rt_streamer_node:main",
            "robot_state_monitor_node = a0509_vr_teleop.robot_state_monitor_node:main",
            "rviz_visualizer_node = a0509_vr_teleop.rviz_visualizer_node:main",
        ],
    },
)
