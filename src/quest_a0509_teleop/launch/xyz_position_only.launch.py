from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("quest_a0509_teleop")
    config_file = LaunchConfiguration("config_file")
    dry_run = LaunchConfiguration("dry_run")

    default_config = PathJoinSubstitution(
        [pkg_share, "config", "xyz_position_only.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("dry_run", default_value="true"),
            Node(
                package="quest_a0509_teleop",
                executable="xyz_mapper_node",
                name="xyz_mapper_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="quest_a0509_teleop",
                executable="quest_input_button_node",
                name="quest_input_button_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="quest_a0509_teleop",
                executable="safety_guard_node",
                name="safety_guard_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="quest_a0509_teleop",
                executable="servol_rt_streamer_node",
                name="servol_rt_streamer_node",
                output="screen",
                parameters=[
                    config_file,
                    {"dry_run": ParameterValue(dry_run, value_type=bool)},
                ],
            ),
        ]
    )
