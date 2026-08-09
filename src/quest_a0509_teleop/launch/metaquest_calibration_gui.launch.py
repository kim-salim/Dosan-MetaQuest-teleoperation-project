from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    default_config = PathJoinSubstitution(
        [FindPackageShare("quest_a0509_teleop"), "config", "xyz_position_only.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="quest_a0509_teleop",
                executable="metaquest_calibration_gui",
                name="metaquest_calibration_gui",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
