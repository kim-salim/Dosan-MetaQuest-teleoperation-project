from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_robot_bringup = LaunchConfiguration("start_robot_bringup")
    start_gripper_io = LaunchConfiguration("start_gripper_io")
    start_quest_inputs_mapper = LaunchConfiguration("start_quest_inputs_mapper")
    start_joy_mapper = LaunchConfiguration("start_joy_mapper")
    start_probe = LaunchConfiguration("start_probe")

    input_topic = LaunchConfiguration("input_topic")
    a_button_field = LaunchConfiguration("a_button_field")
    b_button_field = LaunchConfiguration("b_button_field")
    button_threshold = LaunchConfiguration("button_threshold")
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
    quest2ros_python_path = LaunchConfiguration("quest2ros_python_path")
    quest2ros_library_path = LaunchConfiguration("quest2ros_library_path")

    dsr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dsr_bringup2"),
                    "launch",
                    "dsr_bringup2_rviz.launch.py",
                ]
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
            DeclareLaunchArgument("start_robot_bringup", default_value="true"),
            DeclareLaunchArgument("start_gripper_io", default_value="true"),
            DeclareLaunchArgument("start_quest_inputs_mapper", default_value="true"),
            DeclareLaunchArgument("start_joy_mapper", default_value="false"),
            DeclareLaunchArgument("start_probe", default_value="false"),
            DeclareLaunchArgument("mode", default_value="real"),
            DeclareLaunchArgument("host", default_value="192.168.137.100"),
            DeclareLaunchArgument("rt_host", default_value="192.168.137.10"),
            DeclareLaunchArgument("port", default_value="12345"),
            DeclareLaunchArgument("model", default_value="a0509"),
            DeclareLaunchArgument("name", default_value="dsr01"),
            DeclareLaunchArgument("color", default_value="white"),
            DeclareLaunchArgument("remap_tf", default_value="false"),
            DeclareLaunchArgument("input_topic", default_value="/q2r_right_hand_inputs"),
            DeclareLaunchArgument("a_button_field", default_value="button_lower"),
            DeclareLaunchArgument("b_button_field", default_value="button_upper"),
            DeclareLaunchArgument("button_threshold", default_value="0.5"),
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
            DeclareLaunchArgument(
                "quest2ros_python_path",
                default_value=(
                    "/home/salim2001/quest2ros2_ws/install/quest2ros/local/lib/"
                    "python3.10/dist-packages"
                ),
            ),
            DeclareLaunchArgument(
                "quest2ros_library_path",
                default_value="/home/salim2001/quest2ros2_ws/install/quest2ros/lib",
            ),
            dsr_bringup,
            Node(
                package="jrt_gripper_io",
                executable="quest_inputs_ab_gripper_mapper_node",
                name="quest_inputs_ab_gripper_mapper_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "a_button_field": a_button_field,
                        "b_button_field": b_button_field,
                        "button_threshold": ParameterValue(
                            button_threshold,
                            value_type=float,
                        ),
                        "watchdog_timeout_sec": ParameterValue(
                            watchdog_timeout_sec,
                            value_type=float,
                        ),
                        "command_topic": command_topic,
                        "quest2ros_python_path": quest2ros_python_path,
                        "quest2ros_library_path": quest2ros_library_path,
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_gripper_io,
                            "' == 'true' and '",
                            start_quest_inputs_mapper,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
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
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_gripper_io,
                            "' == 'true' and '",
                            start_joy_mapper,
                            "' == 'true'",
                        ]
                    )
                ),
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
                condition=IfCondition(start_gripper_io),
            ),
            Node(
                package="jrt_gripper_io",
                executable="joy_button_probe_node",
                name="joy_button_probe_node",
                output="screen",
                parameters=[{"joy_topic": joy_topic}],
                condition=IfCondition(start_probe),
            ),
        ]
    )
