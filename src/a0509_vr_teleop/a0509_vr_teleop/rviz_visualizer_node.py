"""Publish RViz markers for raw/projected/actual TCP and workspace."""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from .pose_utils import normalize_pose6


def mm_to_m(value: float) -> float:
    return float(value) / 1000.0


class RvizVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("rviz_visualizer_node")
        self._declare_parameters()
        self.raw_pose: Optional[list[float]] = None
        self.projected_pose: Optional[list[float]] = None
        self.actual_tcp: Optional[list[float]] = None
        self.status_text = "A0509 VR teleop"

        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("markers_topic").value, 10
        )
        self.raw_sub = self.create_subscription(
            Float64MultiArray,
            self.get_parameter("raw_target_topic").value,
            self._on_raw,
            10,
        )
        self.projected_sub = self.create_subscription(
            Float64MultiArray,
            self.get_parameter("projected_target_topic").value,
            self._on_projected,
            10,
        )
        self.monitor_sub = self.create_subscription(
            String,
            self.get_parameter("monitor_vis_topic").value,
            self._on_monitor,
            10,
        )
        self.status_sub = self.create_subscription(
            String,
            self.get_parameter("safety_status_topic").value,
            self._on_status,
            10,
        )
        self.timer = self.create_timer(0.1, self._publish_markers)

    def _declare_parameters(self) -> None:
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("markers_topic", "/teleop/markers")
        self.declare_parameter("raw_target_topic", "/teleop/raw_target_pose")
        self.declare_parameter("projected_target_topic", "/teleop/projected_target_pose")
        self.declare_parameter("monitor_vis_topic", "/teleop/monitor_vis")
        self.declare_parameter("safety_status_topic", "/teleop/safety_status")
        self.declare_parameter("workspace.x_min_mm", 250.0)
        self.declare_parameter("workspace.x_max_mm", 750.0)
        self.declare_parameter("workspace.y_min_mm", -350.0)
        self.declare_parameter("workspace.y_max_mm", 350.0)
        self.declare_parameter("workspace.z_min_mm", 120.0)
        self.declare_parameter("workspace.z_max_mm", 600.0)

    def _on_raw(self, msg: Float64MultiArray) -> None:
        try:
            self.raw_pose = normalize_pose6(msg.data)
        except ValueError:
            return

    def _on_projected(self, msg: Float64MultiArray) -> None:
        try:
            self.projected_pose = normalize_pose6(msg.data)
        except ValueError:
            return

    def _on_monitor(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            actual = data.get("actual_tcp")
            self.actual_tcp = normalize_pose6(actual) if actual is not None else self.actual_tcp
            state = data.get("robot_state")
            age = data.get("source_age_sec")
            self.status_text = f"robot_state={state} monitor_age={age:.2f}s" if age is not None else str(data)
        except Exception:
            return

    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.status_text = (
                f"safety={data.get('reason')} hold={data.get('hold')} "
                f"projected={data.get('projected')}"
            )
        except Exception:
            self.status_text = msg.data

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        markers.markers.append(self._workspace_marker(0))
        if self.raw_pose is not None:
            markers.markers.append(self._sphere_marker(1, "raw_target", self.raw_pose, (1.0, 0.4, 0.1, 0.85)))
        if self.projected_pose is not None:
            markers.markers.append(
                self._sphere_marker(2, "projected_target", self.projected_pose, (0.1, 0.8, 0.2, 0.95))
            )
        if self.actual_tcp is not None:
            markers.markers.append(self._sphere_marker(3, "actual_tcp", self.actual_tcp, (0.1, 0.4, 1.0, 0.95)))
        markers.markers.append(self._text_marker(4, self.status_text))
        self.marker_pub.publish(markers)

    def _base_marker(self, marker_id: int, ns: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.get_parameter("frame_id").value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.ADD
        return marker

    def _sphere_marker(self, marker_id: int, ns: str, pose: list[float], rgba: tuple[float, float, float, float]) -> Marker:
        marker = self._base_marker(marker_id, ns)
        marker.type = Marker.SPHERE
        marker.pose.position.x = mm_to_m(pose[0])
        marker.pose.position.y = mm_to_m(pose[1])
        marker.pose.position.z = mm_to_m(pose[2])
        marker.scale.x = marker.scale.y = marker.scale.z = 0.04
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def _workspace_marker(self, marker_id: int) -> Marker:
        marker = self._base_marker(marker_id, "workspace")
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.01
        marker.color.r = 0.9
        marker.color.g = 0.9
        marker.color.b = 0.9
        marker.color.a = 0.65
        xs = [
            self.get_parameter("workspace.x_min_mm").value,
            self.get_parameter("workspace.x_max_mm").value,
        ]
        ys = [
            self.get_parameter("workspace.y_min_mm").value,
            self.get_parameter("workspace.y_max_mm").value,
        ]
        zs = [
            self.get_parameter("workspace.z_min_mm").value,
            self.get_parameter("workspace.z_max_mm").value,
        ]
        corners = [(x, y, z) for x in xs for y in ys for z in zs]
        for i, a in enumerate(corners):
            for j, b in enumerate(corners):
                if j <= i:
                    continue
                if sum(1 for k in range(3) if abs(a[k] - b[k]) > 1e-6) == 1:
                    marker.points.append(_point_from_mm(a))
                    marker.points.append(_point_from_mm(b))
        return marker

    def _text_marker(self, marker_id: int, text: str) -> Marker:
        marker = self._base_marker(marker_id, "status_text")
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = 0.4
        marker.pose.position.y = -0.45
        marker.pose.position.z = 0.75
        marker.scale.z = 0.04
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = text
        return marker


def _point_from_mm(values: tuple[float, float, float]):
    from geometry_msgs.msg import Point

    point = Point()
    point.x = mm_to_m(values[0])
    point.y = mm_to_m(values[1])
    point.z = mm_to_m(values[2])
    return point


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RvizVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
