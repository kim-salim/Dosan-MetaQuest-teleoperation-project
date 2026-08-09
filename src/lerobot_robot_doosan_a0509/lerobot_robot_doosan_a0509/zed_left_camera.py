"""FFmpeg-backed LeRobot camera exposing the left RGB view from a ZED2."""

from __future__ import annotations

import shutil
import subprocess
import time
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Any

import cv2
import numpy as np
from lerobot.cameras.camera import Camera
from numpy.typing import NDArray

from lerobot_robot_doosan_a0509.config_zed_left_camera import (
    ZedLeftCameraConfig,
)


class ZedLeftCamera(Camera):
    """Continuously drain ZED2 UVC through FFmpeg and retain the latest left RGB frame."""

    def __init__(self, config: ZedLeftCameraConfig):
        super().__init__(config)
        self.config = config
        self.zed_config = config
        self.index_or_path = config.index_or_path
        self.color_mode = config.color_mode
        self.warmup_s = config.warmup_s
        self.frame_lock = Lock()
        self.latest_frame: NDArray[Any] | None = None
        self.latest_timestamp: float | None = None
        self.new_frame_event = Event()
        self._stop_event: Event | None = None
        self._thread: Thread | None = None
        self._process: subprocess.Popen[bytes] | None = None
        self._reader_error: str | None = None

    def __str__(self) -> str:
        return f"ZedLeftCamera({self.index_or_path})"

    @property
    def is_connected(self) -> bool:
        return (
            self._process is not None
            and self._process.poll() is None
            and self._thread is not None
            and self._thread.is_alive()
        )

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        return []

    def _ffmpeg_command(self) -> list[str]:
        fourcc = (self.config.fourcc or "YUYV").upper()
        input_format = {"YUYV": "yuyv422"}.get(fourcc)
        if input_format is None:
            raise ValueError(f"unsupported ZED2 FFmpeg input FOURCC: {fourcc}")
        return [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "v4l2",
            "-input_format",
            input_format,
            "-video_size",
            f"{self.config.capture_width}x{self.config.capture_height}",
            "-framerate",
            str(self.config.fps),
            "-i",
            self.index_or_path,
            "-an",
            "-sn",
            "-dn",
            "-vf",
            f"crop={self.config.width}:{self.config.height}:0:0",
            "-pix_fmt",
            "rgb24",
            "-threads",
            "1",
            "-f",
            "rawvideo",
            "pipe:1",
        ]

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise RuntimeError(f"{self} is already connected")
        if shutil.which("ffmpeg") is None:
            raise ConnectionError("ffmpeg executable is required for ZED2 capture")
        if not Path(self.index_or_path).exists():
            raise ConnectionError(f"ZED2 device path does not exist: {self.index_or_path}")

        self.latest_frame = None
        self.latest_timestamp = None
        self._reader_error = None
        self.new_frame_event.clear()
        self._stop_event = Event()
        self._process = subprocess.Popen(
            self._ffmpeg_command(),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._thread = Thread(
            target=self._read_loop,
            name="zed2_ffmpeg_read_loop",
            daemon=True,
        )
        self._thread.start()

        if warmup:
            timeout_sec = max(1.0, float(self.warmup_s))
            if not self.new_frame_event.wait(timeout=timeout_sec):
                error = self._reader_error or self._stderr_summary()
                self.disconnect()
                raise TimeoutError(
                    f"Timed out waiting {timeout_sec:.1f}s for {self}; {error}"
                )

    def _read_loop(self) -> None:
        assert self._process is not None
        assert self._process.stdout is not None
        assert self._stop_event is not None
        frame_size = int(self.config.width * self.config.height * 3)
        try:
            while not self._stop_event.is_set():
                data = self._read_exact(self._process.stdout, frame_size)
                if data is None:
                    if not self._stop_event.is_set():
                        self._reader_error = self._stderr_summary()
                    return
                frame = np.frombuffer(data, dtype=np.uint8).reshape(
                    self.config.height,
                    self.config.width,
                    3,
                )
                with self.frame_lock:
                    self.latest_frame = frame
                    self.latest_timestamp = time.perf_counter()
                self.new_frame_event.set()
        except Exception as exc:
            if not self._stop_event.is_set():
                self._reader_error = repr(exc)

    @staticmethod
    def _read_exact(stream: Any, size: int) -> bytes | None:
        chunks: list[bytes] = []
        remaining = size
        while remaining:
            chunk = stream.read(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _stderr_summary(self) -> str:
        process = self._process
        if process is None or process.stderr is None or process.poll() is None:
            return "FFmpeg produced no complete frame"
        try:
            rendered = process.stderr.read().decode("utf-8", errors="replace").strip()
        except Exception:
            rendered = ""
        return rendered[-1000:] if rendered else f"FFmpeg exited with code {process.returncode}"

    def read(self) -> NDArray[Any]:
        return self.async_read(timeout_ms=1000)

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        if not self.is_connected:
            raise RuntimeError(f"{self} is not connected")
        self.new_frame_event.clear()
        if not self.new_frame_event.wait(timeout=float(timeout_ms) / 1000.0):
            raise TimeoutError(
                f"Timed out waiting for a new frame from {self}; "
                f"{self._reader_error or 'reader is alive'}"
            )
        with self.frame_lock:
            frame = self.latest_frame
        if frame is None:
            raise RuntimeError(f"{self} signalled a frame without image data")
        return frame

    def read_latest(self, max_age_ms: int = 500) -> NDArray[Any]:
        if not self.is_connected:
            raise RuntimeError(f"{self} is not connected")
        with self.frame_lock:
            frame = self.latest_frame
            timestamp = self.latest_timestamp
        if frame is None or timestamp is None:
            raise RuntimeError(f"{self} has not captured a frame")
        age_ms = (time.perf_counter() - timestamp) * 1e3
        if age_ms > max_age_ms:
            # A multiprocessing image writer can briefly hold the GIL while
            # serializing a large frame. Give the reader thread one bounded
            # scheduling window, then require a genuinely new frame rather
            # than returning stale image data.
            self.new_frame_event.clear()
            self.new_frame_event.wait(timeout=float(max_age_ms) / 1000.0)
            with self.frame_lock:
                frame = self.latest_frame
                timestamp = self.latest_timestamp
            if frame is None or timestamp is None:
                raise RuntimeError(f"{self} lost its latest frame during recovery")
            age_ms = (time.perf_counter() - timestamp) * 1e3
        if age_ms > max_age_ms:
            raise TimeoutError(
                f"{self} latest frame is too old: {age_ms:.1f} ms "
                f"(max allowed: {max_age_ms} ms); "
                f"{self._reader_error or 'FFmpeg reader is alive'}"
            )
        return frame

    def disconnect(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        process = self._process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if process is not None:
            if process.stdout is not None:
                process.stdout.close()
            if process.stderr is not None:
                process.stderr.close()
        self._process = None
        self._thread = None
        self._stop_event = None
        self.new_frame_event.clear()

    def _postprocess_image(self, image: NDArray[Any]) -> NDArray[Any]:
        expected = (
            self.config.capture_height,
            self.config.capture_width,
            3,
        )
        if image.shape != expected:
            raise RuntimeError(
                f"unexpected ZED2 stereo frame shape: {image.shape}; expected {expected}"
            )
        stereo_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(stereo_rgb[:, : self.config.width, :])
