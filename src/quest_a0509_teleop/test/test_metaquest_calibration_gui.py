import queue

import rclpy

from quest_a0509_teleop.metaquest_calibration_gui import MetaQuestCalibrationRos


def test_calibration_gui_ros_node_constructs_service_clients():
    rclpy.init()
    node = None
    try:
        node = MetaQuestCalibrationRos(queue.Queue())
        assert set(node._service_clients) == {"Calibrate", "Recenter", "Reset"}
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
