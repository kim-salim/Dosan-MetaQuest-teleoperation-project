from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("quest_a0509_teleop")
    config_file = LaunchConfiguration("config_file")
    dry_run = LaunchConfiguration("dry_run")
    robot_namespace = LaunchConfiguration("robot_namespace")
    doosan_servol_topic = LaunchConfiguration("doosan_servol_topic")
    start_robot_bringup = LaunchConfiguration("start_robot_bringup")
    start_teleop = LaunchConfiguration("start_teleop")
    start_gui = LaunchConfiguration("start_gui")

    default_config = PathJoinSubstitution(
        [pkg_share, "config", "xyz_position_only.yaml"]
    )
    common_overrides = {"robot_namespace": robot_namespace}

    dsr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dsr_bringup2"), "launch", "dsr_bringup2_rviz.launch.py"]
            )
        ),
        launch_arguments={
            "mode": LaunchConfiguration("mode"),
            "host": LaunchConfiguration("host"),
            "rt_host": LaunchConfiguration("rt_host"),
            "port": LaunchConfiguration("port"),
            "model": LaunchConfiguration("model"),
            "name": LaunchConfiguration("name"),
            "color": LaunchConfiguration("color"),
            "remap_tf": LaunchConfiguration("remap_tf"),
        }.items(),
        condition=IfCondition(start_robot_bringup),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("robot_namespace", default_value="/dsr01"),
            DeclareLaunchArgument("doosan_servol_topic", default_value="/dsr01/servol_rt_stream"),
            DeclareLaunchArgument("start_robot_bringup", default_value="true"),
            DeclareLaunchArgument("start_teleop", default_value="true"),
            DeclareLaunchArgument("start_gui", default_value="true"),
            DeclareLaunchArgument("mode", default_value="real"),
            DeclareLaunchArgument("host", default_value="192.168.137.100"),
            DeclareLaunchArgument("rt_host", default_value="192.168.137.10"),
            DeclareLaunchArgument("port", default_value="12345"),
            DeclareLaunchArgument("model", default_value="a0509"),
            DeclareLaunchArgument("name", default_value="dsr01"),
            DeclareLaunchArgument("color", default_value="white"),
            DeclareLaunchArgument("remap_tf", default_value="false"),
            dsr_bringup,
            Node(
                package="quest_a0509_teleop",
                executable="xyz_mapper_node",
                name="xyz_mapper_node",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(start_teleop),
            ),
            Node(
                package="quest_a0509_teleop",
                executable="quest_input_button_node",
                name="quest_input_button_node",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(start_teleop),
            ),
            Node(
                package="quest_a0509_teleop",
                executable="safety_guard_node",
                name="safety_guard_node",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(start_teleop),
            ),
            Node(
                package="quest_a0509_teleop",
                executable="servol_rt_streamer_node",
                name="servol_rt_streamer_node",
                output="screen",
                parameters=[
                    config_file,
                    common_overrides,
                    {
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "doosan_servol_topic": doosan_servol_topic,
                    },
                ],
                condition=IfCondition(start_teleop),
            ),
            Node(
                package="quest_a0509_teleop",
                executable="robot_prep_node",
                name="robot_prep_node",
                output="screen",
                parameters=[config_file, common_overrides],
                condition=IfCondition(start_teleop),
            ),
            Node(
                package="quest_a0509_teleop",
                executable="teleop_check_gui",
                name="teleop_check_gui",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(start_gui),
            ),
        ]
    )
