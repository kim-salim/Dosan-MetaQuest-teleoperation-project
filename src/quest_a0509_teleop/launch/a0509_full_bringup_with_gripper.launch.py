from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("quest_a0509_teleop")
    full_bringup_launch = PathJoinSubstitution(
        [pkg_share, "launch", "a0509_full_bringup.launch.py"]
    )
    default_config = PathJoinSubstitution(
        [pkg_share, "config", "xyz_position_only.yaml"]
    )

    dry_run = LaunchConfiguration("dry_run")
    start_endpoint = LaunchConfiguration("start_endpoint")
    start_gripper = LaunchConfiguration("start_gripper")

    endpoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_tcp_endpoint"), "launch", "endpoint.py"]
            )
        ),
        condition=IfCondition(start_endpoint),
    )

    a0509_full_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(full_bringup_launch),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file"),
            "dry_run": dry_run,
            "robot_namespace": LaunchConfiguration("robot_namespace"),
            "doosan_servol_topic": LaunchConfiguration("doosan_servol_topic"),
            "start_robot_bringup": LaunchConfiguration("start_robot_bringup"),
            "start_teleop": LaunchConfiguration("start_teleop"),
            "start_gui": LaunchConfiguration("start_gui"),
            "mode": LaunchConfiguration("mode"),
            "host": LaunchConfiguration("host"),
            "rt_host": LaunchConfiguration("rt_host"),
            "port": LaunchConfiguration("port"),
            "model": LaunchConfiguration("model"),
            "name": LaunchConfiguration("name"),
            "color": LaunchConfiguration("color"),
            "remap_tf": LaunchConfiguration("remap_tf"),
        }.items(),
    )

    gripper_io = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("jrt_gripper_io"),
                    "launch",
                    "jrt_gripper_robot_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "start_robot_bringup": "false",
            "start_gripper_io": start_gripper,
            "start_quest_inputs_mapper": "true",
            "start_joy_mapper": "false",
            "start_probe": "false",
            "dry_run": dry_run,
            "input_topic": LaunchConfiguration("gripper_input_topic"),
            "a_button_field": LaunchConfiguration("gripper_a_button_field"),
            "b_button_field": LaunchConfiguration("gripper_b_button_field"),
            "button_threshold": LaunchConfiguration(
                "gripper_button_threshold"
            ),
            "watchdog_timeout_sec": LaunchConfiguration(
                "gripper_watchdog_timeout_sec"
            ),
            "command_topic": LaunchConfiguration("gripper_command_topic"),
            "set_tool_do_service": LaunchConfiguration(
                "gripper_set_tool_do_service"
            ),
            "close_do_index": LaunchConfiguration("gripper_close_do_index"),
            "open_do_index": LaunchConfiguration("gripper_open_do_index"),
            "active_value": LaunchConfiguration("gripper_active_value"),
            "inactive_value": LaunchConfiguration("gripper_inactive_value"),
            "service_timeout_sec": LaunchConfiguration(
                "gripper_service_timeout_sec"
            ),
            "command_mode": LaunchConfiguration("gripper_command_mode"),
            "pulse_sec": LaunchConfiguration("gripper_pulse_sec"),
            "interlock_sec": LaunchConfiguration("gripper_interlock_sec"),
            "debounce_sec": LaunchConfiguration("gripper_debounce_sec"),
            "startup_all_off": LaunchConfiguration("gripper_startup_all_off"),
            "shutdown_all_off": LaunchConfiguration(
                "gripper_shutdown_all_off"
            ),
        }.items(),
        condition=IfCondition(start_gripper),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("robot_namespace", default_value="/dsr01"),
            DeclareLaunchArgument(
                "doosan_servol_topic",
                default_value="/dsr01/servol_rt_stream",
            ),
            DeclareLaunchArgument("start_endpoint", default_value="false"),
            DeclareLaunchArgument("start_robot_bringup", default_value="true"),
            DeclareLaunchArgument("start_teleop", default_value="true"),
            DeclareLaunchArgument("start_gui", default_value="true"),
            DeclareLaunchArgument("start_gripper", default_value="true"),
            DeclareLaunchArgument("mode", default_value="real"),
            DeclareLaunchArgument("host", default_value="192.168.137.100"),
            DeclareLaunchArgument("rt_host", default_value="192.168.137.10"),
            DeclareLaunchArgument("port", default_value="12345"),
            DeclareLaunchArgument("model", default_value="a0509"),
            DeclareLaunchArgument("name", default_value="dsr01"),
            DeclareLaunchArgument("color", default_value="white"),
            DeclareLaunchArgument("remap_tf", default_value="false"),
            DeclareLaunchArgument(
                "gripper_input_topic",
                default_value="/q2r_right_hand_inputs",
            ),
            DeclareLaunchArgument(
                "gripper_a_button_field",
                default_value="button_lower",
            ),
            DeclareLaunchArgument(
                "gripper_b_button_field",
                default_value="button_upper",
            ),
            DeclareLaunchArgument(
                "gripper_button_threshold",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "gripper_watchdog_timeout_sec",
                default_value="0.3",
            ),
            DeclareLaunchArgument(
                "gripper_command_topic",
                default_value="/jrt_gripper/cmd",
            ),
            DeclareLaunchArgument(
                "gripper_set_tool_do_service",
                default_value="/dsr01/io/set_tool_digital_output",
            ),
            DeclareLaunchArgument("gripper_close_do_index", default_value="1"),
            DeclareLaunchArgument("gripper_open_do_index", default_value="2"),
            DeclareLaunchArgument("gripper_active_value", default_value="1"),
            DeclareLaunchArgument("gripper_inactive_value", default_value="0"),
            DeclareLaunchArgument(
                "gripper_service_timeout_sec",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "gripper_command_mode",
                default_value="pulse",
            ),
            DeclareLaunchArgument("gripper_pulse_sec", default_value="0.20"),
            DeclareLaunchArgument(
                "gripper_interlock_sec",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "gripper_debounce_sec",
                default_value="0.30",
            ),
            DeclareLaunchArgument(
                "gripper_startup_all_off",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "gripper_shutdown_all_off",
                default_value="true",
            ),
            endpoint_launch,
            a0509_full_bringup,
            gripper_io,
        ]
    )
