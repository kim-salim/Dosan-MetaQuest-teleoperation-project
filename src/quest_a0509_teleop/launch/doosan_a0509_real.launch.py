from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    name = LaunchConfiguration("name")
    model = LaunchConfiguration("model")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dsr_description2"), "xacro", model]
            ),
            ".urdf.xacro",
            " color:=",
            LaunchConfiguration("color"),
            " namespace:=",
            name,
            " host:=",
            LaunchConfiguration("host"),
            " rt_host:=",
            LaunchConfiguration("rt_host"),
            " port:=",
            LaunchConfiguration("port"),
            " mode:=real",
            " model:=",
            model,
            " update_rate:=",
            LaunchConfiguration("update_rate"),
            " use_gazebo:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dsr_controller2"),
            "config",
            "dsr_controller2.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=name,
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=name,
        parameters=[robot_description],
        output="both",
        condition=IfCondition(LaunchConfiguration("start_robot_state_publisher")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=name,
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("dsr_description2"), "rviz", "default.rviz"]
            ),
        ],
        output="log",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=name,
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
        output="screen",
    )
    robot_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=name,
        arguments=["dsr_controller2", "-c", "controller_manager"],
        output="screen",
    )
    start_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[robot_controller],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="dsr01"),
            DeclareLaunchArgument("host", default_value="192.168.137.100"),
            DeclareLaunchArgument("rt_host", default_value="192.168.137.100"),
            DeclareLaunchArgument("port", default_value="12345"),
            DeclareLaunchArgument("model", default_value="a0509"),
            DeclareLaunchArgument("color", default_value="white"),
            DeclareLaunchArgument("update_rate", default_value="100"),
            DeclareLaunchArgument(
                "start_robot_state_publisher",
                default_value="true",
            ),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            control_node,
            robot_state_publisher,
            rviz,
            joint_state_broadcaster,
            start_robot_controller,
        ]
    )
