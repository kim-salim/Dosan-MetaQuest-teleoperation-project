from setuptools import find_packages, setup
from setuptools.command.egg_info import egg_info


class LeRobotPluginEggInfo(egg_info):
    """Keep underscores required by LeRobot 0.6's literal plugin prefix scan."""

    @property
    def name(self):
        return self.distribution.get_name()


package_name = "lerobot_robot_doosan_a0509"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "lerobot>=0.6,<0.7"],
    zip_safe=True,
    maintainer="rvlab",
    maintainer_email="rvlab@example.com",
    description="LeRobot ROS 2 integration for the Doosan A0509 MUX-first control stack.",
    license="MIT",
    tests_require=["pytest"],
    cmdclass={"egg_info": LeRobotPluginEggInfo},
)
