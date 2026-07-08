"""Mock MetaQuest input for dry-run testing without a headset."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockQuestInputNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_quest_input_node")
        self.declare_parameter("controller_state_topic", "/vr/controller_state")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("center_pose", [400.0, 0.0, 350.0, 0.0, 150.0, 0.0])
        self.declare_parameter("amplitude_mm", [80.0, 80.0, 60.0])
        self.declare_parameter("amplitude_deg", [10.0, 8.0, 10.0])
        self.declare_parameter("deadman", True)
        self.declare_parameter("tracking_ok", True)

        self.publisher = self.create_publisher(
            String, self.get_parameter("controller_state_topic").value, 10
        )
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.seq = 0
        self.start_time = time.monotonic()
        self.timer = self.create_timer(period, self._tick)

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.start_time
        center = [float(v) for v in self.get_parameter("center_pose").value]
        amp_mm = [float(v) for v in self.get_parameter("amplitude_mm").value]
        amp_deg = [float(v) for v in self.get_parameter("amplitude_deg").value]
        pose = center[:]
        pose[0] += amp_mm[0] * math.sin(elapsed * 0.4)
        pose[1] += amp_mm[1] * math.sin(elapsed * 0.33)
        pose[2] += amp_mm[2] * math.sin(elapsed * 0.27)
        pose[3] += amp_deg[0] * math.sin(elapsed * 0.2)
        pose[4] += amp_deg[1] * math.sin(elapsed * 0.23)
        pose[5] += amp_deg[2] * math.sin(elapsed * 0.17)
        data = {
            "seq": self.seq,
            "stamp": time.monotonic(),
            "pose": pose,
            "tracking_ok": bool(self.get_parameter("tracking_ok").value),
            "deadman": bool(self.get_parameter("deadman").value),
            "clutch": False,
        }
        self.seq += 1
        self.publisher.publish(String(data=json.dumps(data, separators=(",", ":"))))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockQuestInputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
