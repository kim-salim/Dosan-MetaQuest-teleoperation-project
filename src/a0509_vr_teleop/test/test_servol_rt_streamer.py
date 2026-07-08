import rclpy

from a0509_vr_teleop.servol_rt_streamer_node import (
    ServolRtStreamerNode,
    make_servol_rt_stream,
)


def test_make_servol_rt_stream_uses_doosan_pose_units():
    msg = make_servol_rt_stream(
        [400.0, 0.0, 350.0, 0.0, 150.0, 0.0],
        [0.0] * 6,
        [0.0] * 6,
        0.5,
    )
    assert list(msg.pos) == [400.0, 0.0, 350.0, 0.0, 150.0, 0.0]
    assert list(msg.vel) == [0.0] * 6
    assert list(msg.acc) == [0.0] * 6
    assert msg.time == 0.5


def test_enable_robot_output_false_creates_no_robot_publisher():
    if not rclpy.ok():
        rclpy.init(args=None)
    node = ServolRtStreamerNode()
    try:
        assert node.enable_robot_output is False
        assert node.robot_pub is None
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
