from glob import glob
from setuptools import find_packages, setup


package_name = "quest_a0509_teleop"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="salim2001",
    maintainer_email="salim2001@example.com",
    description="Meta Quest2ROS to Doosan A0509 ServoL RT task-space teleoperation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xyz_mapper_node = quest_a0509_teleop.xyz_mapper_node:main",
            "safety_guard_node = quest_a0509_teleop.safety_guard_node:main",
            "servol_rt_streamer_node = quest_a0509_teleop.servol_rt_streamer_node:main",
            "robot_prep_node = quest_a0509_teleop.robot_prep_node:main",
            "teleop_check_gui = quest_a0509_teleop.teleop_check_gui:main",
            "quest_input_button_node = quest_a0509_teleop.quest_input_button_node:main",
        ],
    },
)
