from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("a0509_vr_teleop")
    config_file = LaunchConfiguration("config_file")
    robot_namespace = LaunchConfiguration("robot_namespace")
    enable_robot_output = LaunchConfiguration("enable_robot_output")
    use_mock_quest_input = LaunchConfiguration("use_mock_quest_input")

    default_config = PathJoinSubstitution([pkg_share, "config", "a0509_vr_teleop.yaml"])

    common_params = [
        config_file,
        {
            "robot_namespace": robot_namespace,
            "servol_topic": [robot_namespace, "/servol_rt_stream"],
            "robot_error_topic": [robot_namespace, "/error"],
            "enable_robot_output": ParameterValue(enable_robot_output, value_type=bool),
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("robot_namespace", default_value="/dsr01"),
            DeclareLaunchArgument("enable_robot_output", default_value="false"),
            DeclareLaunchArgument("use_mock_quest_input", default_value="true"),
            Node(
                package="a0509_vr_teleop",
                executable="mock_quest_input_node",
                name="mock_quest_input_node",
                output="screen",
                parameters=common_params,
                condition=IfCondition(use_mock_quest_input),
            ),
            Node(
                package="a0509_vr_teleop",
                executable="quest_gateway_node",
                name="quest_gateway_node",
                output="screen",
                parameters=common_params,
                condition=UnlessCondition(use_mock_quest_input),
            ),
            Node(
                package="a0509_vr_teleop",
                executable="vr_frame_mapper_node",
                name="vr_frame_mapper_node",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="a0509_vr_teleop",
                executable="safety_guard_node",
                name="safety_guard_node",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="a0509_vr_teleop",
                executable="servol_rt_streamer_node",
                name="servol_rt_streamer_node",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="a0509_vr_teleop",
                executable="robot_state_monitor_node",
                name="robot_state_monitor_node",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="a0509_vr_teleop",
                executable="rviz_visualizer_node",
                name="rviz_visualizer_node",
                output="screen",
                parameters=common_params,
            ),
        ]
    )
