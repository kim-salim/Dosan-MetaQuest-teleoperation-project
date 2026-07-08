"""UDP gateway for MetaQuest controller packets.

Expected UDP payload: UTF-8 JSON object. The node forwards valid packets as
std_msgs/String on /vr/controller_state. The mapper/safety nodes interpret:
    pose: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    tracking_ok: bool
    deadman: bool
    clutch: bool
    stamp: float, optional
    seq: int, optional
"""

from __future__ import annotations

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QuestGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("quest_gateway_node")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 5005)
        self.declare_parameter("controller_state_topic", "/vr/controller_state")

        topic = self.get_parameter("controller_state_topic").value
        self.publisher = self.create_publisher(String, topic, 10)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._thread.start()
        host = self.get_parameter("listen_host").value
        port = int(self.get_parameter("listen_port").value)
        self.get_logger().info(f"Quest UDP gateway listening on {host}:{port}, publishing {topic}")

    def destroy_node(self) -> bool:
        self._stop_event.set()
        return super().destroy_node()

    def _udp_loop(self) -> None:
        host = self.get_parameter("listen_host").value
        port = int(self.get_parameter("listen_port").value)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.2)
        sock.bind((host, port))
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                payload, _addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                self.get_logger().warning(f"UDP receive failed: {exc}")
                continue
            text = payload.decode("utf-8", errors="replace").strip()
            try:
                data = json.loads(text)
                if not isinstance(data, dict):
                    raise ValueError("payload root must be a JSON object")
                data.setdefault("stamp", time.monotonic())
                out = json.dumps(data, separators=(",", ":"), sort_keys=True)
            except Exception as exc:
                self.get_logger().warning(f"Ignoring invalid Quest packet: {exc}")
                continue
            self.publisher.publish(String(data=out))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = QuestGatewayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
