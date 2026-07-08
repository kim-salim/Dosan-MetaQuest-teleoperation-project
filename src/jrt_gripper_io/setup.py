from glob import glob
from setuptools import find_packages, setup


package_name = "jrt_gripper_io"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/docs", glob("docs/*.md")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="salim2001",
    maintainer_email="salim2001@example.com",
    description="Digital I/O control path for a JRT JEGB gripper on a Doosan A0509 tool flange.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "quest_ab_gripper_mapper_node = jrt_gripper_io.quest_ab_gripper_mapper_node:main",
            "quest_inputs_ab_gripper_mapper_node = jrt_gripper_io.quest_inputs_ab_gripper_mapper_node:main",
            "jrt_tool_io_driver_node = jrt_gripper_io.jrt_tool_io_driver_node:main",
            "joy_button_probe_node = jrt_gripper_io.joy_button_probe_node:main",
        ],
    },
)
