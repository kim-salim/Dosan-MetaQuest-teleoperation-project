from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    joy_topic = LaunchConfiguration("joy_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            Node(
                package="jrt_gripper_io",
                executable="joy_button_probe_node",
                name="joy_button_probe_node",
                output="screen",
                parameters=[{"joy_topic": joy_topic}],
            ),
        ]
    )
