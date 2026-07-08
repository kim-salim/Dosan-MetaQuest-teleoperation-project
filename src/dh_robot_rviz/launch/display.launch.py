from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution(
        [FindPackageShare("dh_robot_rviz"), "urdf", "dh_robot.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("dh_robot_rviz"), "rviz", "dh_robot.rviz"]
    )

    robot_description = {"robot_description": Command(["xacro ", model])}

    return LaunchDescription(
        [
            DeclareLaunchArgument("rvizconfig", default_value=rviz_config),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                parameters=[
                    {
                        "zeros": {
                            "q0": 0.0,
                            "q1": 0.6108652381980153,
                            "q2_clockwise": 0.4363323129985824,
                            "q3_clockwise": 0.0,
                        }
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
                output="screen",
            ),
        ]
    )
