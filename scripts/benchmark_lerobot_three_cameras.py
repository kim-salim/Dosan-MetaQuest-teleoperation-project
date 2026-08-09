#!/usr/bin/env python3
"""Benchmark the default LeRobot RGB cameras without recording or robot motion."""

from __future__ import annotations

import argparse
import json
import os
import time
from collections import defaultdict
from typing import Any

import numpy as np
import psutil
from lerobot.cameras.utils import make_cameras_from_configs

from lerobot_robot_doosan_a0509.config_doosan_a0509_ros import (
    DoosanA0509RosConfig,
)


def _stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"mean": None, "p50": None, "p95": None, "p99": None, "max": None}
    data = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(data)),
        "p50": float(np.percentile(data, 50)),
        "p95": float(np.percentile(data, 95)),
        "p99": float(np.percentile(data, 99)),
        "max": float(np.max(data)),
    }


def benchmark(
    duration_sec: float,
    target_fps: float,
    camera_keys: list[str] | None = None,
    metadata_only: bool = False,
) -> dict[str, Any]:
    if duration_sec <= 0.0:
        raise ValueError("duration_sec must be positive")
    if target_fps <= 0.0:
        raise ValueError("target_fps must be positive")

    config = DoosanA0509RosConfig(
        id="three_camera_benchmark",
        require_fresh_state_on_connect=False,
    )
    if camera_keys:
        unknown = sorted(set(camera_keys) - set(config.cameras))
        if unknown:
            raise ValueError(f"unknown camera keys: {unknown}")
        config.cameras = {key: config.cameras[key] for key in camera_keys}
    cameras = make_cameras_from_configs(config.cameras)
    connected = []
    process = psutil.Process(os.getpid())
    process.cpu_percent(interval=None)
    psutil.cpu_percent(interval=None)

    frame_ages_ms: dict[str, list[float]] = defaultdict(list)
    observed_updates: dict[str, int] = defaultdict(int)
    duplicate_ticks: dict[str, int] = defaultdict(int)
    last_timestamps: dict[str, float | None] = {key: None for key in cameras}
    last_shapes: dict[str, list[int]] = {}
    last_dtypes: dict[str, str] = {}
    loop_read_ms: list[float] = []
    schedule_lateness_ms: list[float] = []
    tick_period_ms: list[float] = []
    previous_tick_time: float | None = None

    try:
        for camera in cameras.values():
            camera.connect()
            connected.append(camera)

        for key, camera in cameras.items():
            frame = camera.read_latest(max_age_ms=500)
            expected_shape = (
                config.cameras[key].height,
                config.cameras[key].width,
                3,
            )
            if frame.shape != expected_shape:
                raise RuntimeError(
                    f"{key} frame shape {frame.shape} != {expected_shape}"
                )
            last_shapes[key] = list(frame.shape)
            last_dtypes[key] = str(frame.dtype)

        period_sec = 1.0 / target_fps
        total_ticks = max(1, int(round(duration_sec * target_fps)))
        start = time.perf_counter()

        for tick_index in range(1, total_ticks + 1):
            scheduled = start + tick_index * period_sec
            remaining = scheduled - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
            tick_time = time.perf_counter()
            schedule_lateness_ms.append(max(0.0, tick_time - scheduled) * 1e3)
            if previous_tick_time is not None:
                tick_period_ms.append((tick_time - previous_tick_time) * 1e3)
            previous_tick_time = tick_time

            read_start = time.perf_counter()
            for key, camera in cameras.items():
                if metadata_only:
                    with camera.frame_lock:
                        timestamp = camera.latest_timestamp
                else:
                    camera.read_latest(max_age_ms=500)
                    with camera.frame_lock:
                        timestamp = camera.latest_timestamp
                if timestamp is None:
                    raise RuntimeError(f"{key} has no capture timestamp")
                frame_age_ms = max(0.0, tick_time - timestamp) * 1e3
                if frame_age_ms > 500.0:
                    raise TimeoutError(f"{key} frame age is {frame_age_ms:.1f} ms")
                frame_ages_ms[key].append(frame_age_ms)
                if timestamp != last_timestamps[key]:
                    observed_updates[key] += 1
                    last_timestamps[key] = timestamp
                else:
                    duplicate_ticks[key] += 1
            loop_read_ms.append((time.perf_counter() - read_start) * 1e3)

        end = time.perf_counter()
        actual_duration = end - start
        process_cpu = process.cpu_percent(interval=None)
        system_cpu = psutil.cpu_percent(interval=None)
        memory = process.memory_info()
        missed_read_deadlines = sum(
            value > period_sec * 1e3 for value in loop_read_ms
        )

        camera_results = {}
        for key in cameras:
            updates = observed_updates[key]
            camera_results[key] = {
                "configured_fps": config.cameras[key].fps,
                "shape": last_shapes[key],
                "dtype": last_dtypes[key],
                "observed_updates": updates,
                "observed_update_hz": updates / actual_duration,
                "duplicate_sample_ticks": duplicate_ticks[key],
                "duplicate_ratio": duplicate_ticks[key] / total_ticks,
                "frame_age_ms": _stats(frame_ages_ms[key]),
            }

        return {
            "status": "ok",
            "duration_requested_sec": duration_sec,
            "duration_actual_sec": actual_duration,
            "target_fps": target_fps,
            "metadata_only": metadata_only,
            "sample_ticks": total_ticks,
            "sample_loop_hz": total_ticks / actual_duration,
            "missed_read_deadlines": missed_read_deadlines,
            "loop_read_ms": _stats(loop_read_ms),
            "tick_period_ms": _stats(tick_period_ms),
            "schedule_lateness_ms": _stats(schedule_lateness_ms),
            "process_cpu_percent": process_cpu,
            "system_cpu_percent": system_cpu,
            "rss_mib": memory.rss / (1024 * 1024),
            "cameras": camera_results,
        }
    finally:
        for camera in reversed(connected):
            camera.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--target-fps", type=float, default=30.0)
    parser.add_argument(
        "--camera",
        action="append",
        choices=("front", "side", "zed_rgb"),
        help="Camera key to benchmark; repeat for a subset (default: all).",
    )
    parser.add_argument(
        "--metadata-only",
        action="store_true",
        help="Poll capture timestamps without copying every frame.",
    )
    args = parser.parse_args()
    print(
        json.dumps(
            benchmark(args.duration_sec, args.target_fps, args.camera, args.metadata_only),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
