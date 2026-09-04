"""Reference-counted ROS 2 runtime shared by Robot and Teleoperator instances."""

from __future__ import annotations

import os
import threading
from typing import ClassVar

import rclpy
from rclpy.context import Context
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.node import Node
from lerobot_robot_doosan_a0509.runtime_scheduling import pin_current_thread_from_env



def _environment_positive_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    value = int(raw)
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value}")
    return value

class RosRuntime:
    _lock: ClassVar[threading.RLock] = threading.RLock()
    _shared: ClassVar["RosRuntime | None"] = None
    _references: ClassVar[int] = 0

    def __init__(self) -> None:
        self.context = Context()
        rclpy.init(context=self.context)
        executor_threads = _environment_positive_int(
            "LEROBOT_A0509_ROS_EXECUTOR_THREADS", 4
        )
        if executor_threads == 1:
            self.executor = SingleThreadedExecutor(context=self.context)
        else:
            self.executor = MultiThreadedExecutor(
                num_threads=executor_threads, context=self.context
            )
        self._closed = False
        self._thread = threading.Thread(
            target=self._spin,
            name="lerobot-a0509-ros-executor",
            daemon=True,
        )
        self._thread.start()

    @classmethod
    def acquire(cls) -> "RosRuntime":
        with cls._lock:
            if cls._shared is None or cls._shared._closed:
                cls._shared = cls()
                cls._references = 0
            cls._references += 1
            return cls._shared

    @classmethod
    def reference_count(cls) -> int:
        with cls._lock:
            return cls._references

    def create_node(self, name: str) -> Node:
        if self._closed:
            raise RuntimeError("ROS runtime is already closed")
        node = rclpy.create_node(name, context=self.context)
        self.executor.add_node(node)
        return node

    def release_node(self, node: Node | None) -> None:
        if node is not None:
            try:
                self.executor.remove_node(node)
            finally:
                node.destroy_node()
        self.release()

    def release(self) -> None:
        should_shutdown = False
        with type(self)._lock:
            if type(self)._shared is not self or type(self)._references <= 0:
                return
            type(self)._references -= 1
            if type(self)._references == 0:
                type(self)._shared = None
                self._closed = True
                should_shutdown = True
        if should_shutdown:
            self._shutdown()

    def _spin(self) -> None:
        pin_current_thread_from_env("LEROBOT_A0509_ROS_CPU_SET")
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        except Exception:
            if not self._closed:
                raise

    def _shutdown(self) -> None:
        self.executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        if self.context.ok():
            rclpy.shutdown(context=self.context)
