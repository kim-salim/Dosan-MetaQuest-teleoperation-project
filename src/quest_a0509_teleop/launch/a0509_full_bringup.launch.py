from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("quest_a0509_teleop")
    config_file = LaunchConfiguration("config_file")
    dry_run = LaunchConfiguration("dry_run")
    robot_namespace = LaunchConfiguration("robot_namespace")
    controller_name = LaunchConfiguration("controller_name")
    doosan_servol_topic = LaunchConfiguration("doosan_servol_topic")
    start_robot_bringup = LaunchConfiguration("start_robot_bringup")
    start_teleop = LaunchConfiguration("start_teleop")
    start_metaquest_inputs = LaunchConfiguration("start_metaquest_inputs")
    start_gui = LaunchConfiguration("start_gui")
    start_calibration_gui = LaunchConfiguration("start_calibration_gui")
    use_command_mux = LaunchConfiguration("use_command_mux")
    initial_control_source = LaunchConfiguration("initial_control_source")
    metaquest_target_topic = LaunchConfiguration("metaquest_target_topic")
    metaquest_gripper_topic = LaunchConfiguration("metaquest_gripper_topic")
    metaquest_heartbeat_topic = LaunchConfiguration("metaquest_heartbeat_topic")
    metaquest_calibration_valid_topic = LaunchConfiguration(
        "metaquest_calibration_valid_topic"
    )
    require_metaquest_calibration = LaunchConfiguration(
        "require_metaquest_calibration"
    )
    lerobot_target_topic = LaunchConfiguration("lerobot_target_topic")
    lerobot_gripper_topic = LaunchConfiguration("lerobot_gripper_topic")
    selected_target_topic = LaunchConfiguration("selected_target_topic")
    selected_gripper_topic = LaunchConfiguration("selected_gripper_topic")
    selected_heartbeat_topic = LaunchConfiguration("selected_heartbeat_topic")
    metaquest_timeout_sec = LaunchConfiguration("metaquest_timeout_sec")
    metaquest_gripper_min_pulse_sec = LaunchConfiguration(
        "metaquest_gripper_min_pulse_sec"
    )
    lerobot_timeout_sec = LaunchConfiguration("lerobot_timeout_sec")
    mux_heartbeat_timeout_sec = LaunchConfiguration("mux_heartbeat_timeout_sec")
    stream_ramp_linear_mm_per_tick = LaunchConfiguration(
        "stream_ramp_linear_mm_per_tick"
    )
    stream_ramp_rot_deg_per_tick = LaunchConfiguration(
        "stream_ramp_rot_deg_per_tick"
    )
    mapper_target_topic = PythonExpression(
        [
            "'",
            metaquest_target_topic,
            "' if '",
            use_command_mux,
            "' == 'true' else '",
            selected_target_topic,
            "'",
        ]
    )

    default_config = PathJoinSubstitution(
        [pkg_share, "config", "xyz_position_only.yaml"]
    )
    common_overrides = {
        "robot_namespace": robot_namespace,
        "controller_name": controller_name,
    }

    dsr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, "launch", "doosan_a0509_real.launch.py"]
            )
        ),
        launch_arguments={
            "host": LaunchConfiguration("host"),
            "rt_host": LaunchConfiguration("rt_host"),
            "port": LaunchConfiguration("port"),
            "model": LaunchConfiguration("model"),
            "name": LaunchConfiguration("name"),
            "color": LaunchConfiguration("color"),
            "update_rate": LaunchConfiguration("update_rate"),
            "start_robot_state_publisher": LaunchConfiguration(
                "start_robot_state_publisher"
            ),
            "start_rviz": LaunchConfiguration("start_rviz"),
        }.items(),
        condition=IfCondition(start_robot_bringup),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("robot_namespace", default_value="/dsr01"),
            DeclareLaunchArgument("controller_name", default_value="dsr_controller2"),
            DeclareLaunchArgument(
                "doosan_servol_topic",
                default_value="/dsr01/dsr_controller2/servol_rt_stream",
            ),
            DeclareLaunchArgument("start_robot_bringup", default_value="true"),
            DeclareLaunchArgument("start_teleop", default_value="true"),
            DeclareLaunchArgument("start_metaquest_inputs", default_value="true"),
            DeclareLaunchArgument("start_gui", default_value="false"),
            DeclareLaunchArgument("start_calibration_gui", default_value="false"),
            DeclareLaunchArgument("use_command_mux", default_value="true"),
            DeclareLaunchArgument("initial_control_source", default_value="DISABLED"),
            DeclareLaunchArgument("metaquest_target_topic", default_value="/control/metaquest/target_posx"),
            DeclareLaunchArgument("metaquest_gripper_topic", default_value="/control/metaquest/gripper_cmd"),
            DeclareLaunchArgument(
                "metaquest_heartbeat_topic",
                default_value="/control/metaquest/valid_pose_heartbeat",
            ),
            DeclareLaunchArgument(
                "metaquest_calibration_valid_topic",
                default_value="/vr/metaquest_calibration/valid",
            ),
            DeclareLaunchArgument("require_metaquest_calibration", default_value="true"),
            DeclareLaunchArgument("lerobot_target_topic", default_value="/control/lerobot/target_posx"),
            DeclareLaunchArgument("lerobot_gripper_topic", default_value="/control/lerobot/gripper_target"),
            DeclareLaunchArgument("selected_target_topic", default_value="/vr/target_posx"),
            DeclareLaunchArgument("selected_gripper_topic", default_value="/jrt_gripper/cmd"),
            DeclareLaunchArgument("selected_heartbeat_topic", default_value="/control/selected_command_heartbeat"),
            DeclareLaunchArgument("metaquest_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument(
                "metaquest_gripper_min_pulse_sec",
                default_value="1.50",
            ),
            DeclareLaunchArgument("lerobot_timeout_sec", default_value="0.3"),
            DeclareLaunchArgument("mux_heartbeat_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument(
                "stream_ramp_linear_mm_per_tick", default_value="6.67"
            ),
            DeclareLaunchArgument(
                "stream_ramp_rot_deg_per_tick", default_value="1.0"
            ),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument(
                "start_robot_state_publisher",
                default_value="true",
            ),
            DeclareLaunchArgument("host", default_value="192.168.137.100"),
            DeclareLaunchArgument("rt_host", default_value="192.168.137.100"),
            DeclareLaunchArgument("port", default_value="12345"),
            DeclareLaunchArgument("model", default_value="a0509"),
            DeclareLaunchArgument("name", default_value="dsr01"),
            DeclareLaunchArgument("color", default_value="white"),
            DeclareLaunchArgument("update_rate", default_value="100"),
            dsr_bringup,
            Node(
                package="quest_a0509_teleop",
                executable="a0509_command_mux_node",
                name="a0509_command_mux_node",
                output="screen",
                parameters=[
                    {
                        "initial_control_source": initial_control_source,
                        "metaquest_target_topic": metaquest_target_topic,
                        "metaquest_gripper_topic": metaquest_gripper_topic,
                        "metaquest_heartbeat_topic": metaquest_heartbeat_topic,
                        "metaquest_calibration_valid_topic": metaquest_calibration_valid_topic,
                        "require_metaquest_calibration": ParameterValue(
                            require_metaquest_calibration, value_type=bool
                        ),
                        "lerobot_target_topic": lerobot_target_topic,
                        "lerobot_gripper_topic": lerobot_gripper_topic,
                        "selected_target_topic": selected_target_topic,
                        "selected_gripper_topic": selected_gripper_topic,
                        "selected_heartbeat_topic": selected_heartbeat_topic,
                        "metaquest_timeout_sec": ParameterValue(
                            metaquest_timeout_sec, value_type=float
                        ),
                        "metaquest_gripper_min_pulse_sec": ParameterValue(
                            metaquest_gripper_min_pulse_sec,
                            value_type=float,
                        ),
                        "lerobot_timeout_sec": ParameterValue(
                            lerobot_timeout_sec, value_type=float
                        ),
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_teleop,
                            "' == 'true' and '",
                            use_command_mux,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="quest_a0509_teleop",
                executable="xyz_mapper_node",
                name="xyz_mapper_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "target_posx_topic": mapper_target_topic,
                        "valid_pose_heartbeat_topic": metaquest_heartbeat_topic,
                        "xy_yaw_calibration_valid_topic": metaquest_calibration_valid_topic,
                        "require_xy_yaw_calibration": ParameterValue(
                            require_metaquest_calibration, value_type=bool
                        ),
                    },
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_teleop,
                            "' == 'true' and '",
                            start_metaquest_inputs,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="quest_a0509_teleop",
                executable="quest_input_button_node",
                name="quest_input_button_node",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_teleop,
                            "' == 'true' and '",
                            start_metaquest_inputs,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="quest_a0509_teleop",
                executable="safety_guard_node",
                name="safety_guard_node",
                output="screen",
                parameters=[config_file, {"target_posx_topic": selected_target_topic}],
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
                        "selected_heartbeat_topic": selected_heartbeat_topic,
                        "require_mux_heartbeat": ParameterValue(
                            use_command_mux, value_type=bool
                        ),
                        "mux_heartbeat_timeout_sec": ParameterValue(
                            mux_heartbeat_timeout_sec, value_type=float
                        ),
                        "stream_ramp_linear_mm_per_tick": ParameterValue(
                            stream_ramp_linear_mm_per_tick, value_type=float
                        ),
                        "stream_ramp_rot_deg_per_tick": ParameterValue(
                            stream_ramp_rot_deg_per_tick, value_type=float
                        ),
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
            Node(
                package="quest_a0509_teleop",
                executable="metaquest_calibration_gui",
                name="metaquest_calibration_gui",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(start_calibration_gui),
            ),
        ]
    )
