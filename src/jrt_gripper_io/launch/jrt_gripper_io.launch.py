from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    joy_topic = LaunchConfiguration("joy_topic")
    a_button_index = LaunchConfiguration("a_button_index")
    b_button_index = LaunchConfiguration("b_button_index")
    watchdog_timeout_sec = LaunchConfiguration("watchdog_timeout_sec")
    command_topic = LaunchConfiguration("command_topic")
    set_tool_do_service = LaunchConfiguration("set_tool_do_service")
    close_do_index = LaunchConfiguration("close_do_index")
    open_do_index = LaunchConfiguration("open_do_index")
    active_value = LaunchConfiguration("active_value")
    inactive_value = LaunchConfiguration("inactive_value")
    service_timeout_sec = LaunchConfiguration("service_timeout_sec")
    command_mode = LaunchConfiguration("command_mode")
    pulse_sec = LaunchConfiguration("pulse_sec")
    interlock_sec = LaunchConfiguration("interlock_sec")
    debounce_sec = LaunchConfiguration("debounce_sec")
    startup_all_off = LaunchConfiguration("startup_all_off")
    shutdown_all_off = LaunchConfiguration("shutdown_all_off")
    dry_run = LaunchConfiguration("dry_run")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            DeclareLaunchArgument("a_button_index", default_value="0"),
            DeclareLaunchArgument("b_button_index", default_value="1"),
            DeclareLaunchArgument("watchdog_timeout_sec", default_value="0.3"),
            DeclareLaunchArgument("command_topic", default_value="/jrt_gripper/cmd"),
            DeclareLaunchArgument(
                "set_tool_do_service",
                default_value="/dsr01/io/set_tool_digital_output",
            ),
            DeclareLaunchArgument("close_do_index", default_value="1"),
            DeclareLaunchArgument("open_do_index", default_value="2"),
            DeclareLaunchArgument("active_value", default_value="1"),
            DeclareLaunchArgument("inactive_value", default_value="0"),
            DeclareLaunchArgument("service_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("command_mode", default_value="pulse"),
            DeclareLaunchArgument("pulse_sec", default_value="0.20"),
            DeclareLaunchArgument("interlock_sec", default_value="0.05"),
            DeclareLaunchArgument("debounce_sec", default_value="0.30"),
            DeclareLaunchArgument("startup_all_off", default_value="true"),
            DeclareLaunchArgument("shutdown_all_off", default_value="true"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            Node(
                package="jrt_gripper_io",
                executable="quest_ab_gripper_mapper_node",
                name="quest_ab_gripper_mapper_node",
                output="screen",
                parameters=[
                    {
                        "joy_topic": joy_topic,
                        "a_button_index": ParameterValue(
                            a_button_index,
                            value_type=int,
                        ),
                        "b_button_index": ParameterValue(
                            b_button_index,
                            value_type=int,
                        ),
                        "watchdog_timeout_sec": ParameterValue(
                            watchdog_timeout_sec,
                            value_type=float,
                        ),
                        "command_topic": command_topic,
                    }
                ],
            ),
            Node(
                package="jrt_gripper_io",
                executable="jrt_tool_io_driver_node",
                name="jrt_tool_io_driver_node",
                output="screen",
                parameters=[
                    {
                        "command_topic": command_topic,
                        "set_tool_do_service": set_tool_do_service,
                        "close_do_index": ParameterValue(
                            close_do_index,
                            value_type=int,
                        ),
                        "open_do_index": ParameterValue(
                            open_do_index,
                            value_type=int,
                        ),
                        "active_value": ParameterValue(
                            active_value,
                            value_type=int,
                        ),
                        "inactive_value": ParameterValue(
                            inactive_value,
                            value_type=int,
                        ),
                        "service_timeout_sec": ParameterValue(
                            service_timeout_sec,
                            value_type=float,
                        ),
                        "command_mode": command_mode,
                        "pulse_sec": ParameterValue(
                            pulse_sec,
                            value_type=float,
                        ),
                        "interlock_sec": ParameterValue(
                            interlock_sec,
                            value_type=float,
                        ),
                        "debounce_sec": ParameterValue(
                            debounce_sec,
                            value_type=float,
                        ),
                        "startup_all_off": ParameterValue(
                            startup_all_off,
                            value_type=bool,
                        ),
                        "shutdown_all_off": ParameterValue(
                            shutdown_all_off,
                            value_type=bool,
                        ),
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                    }
                ],
            ),
        ]
    )
