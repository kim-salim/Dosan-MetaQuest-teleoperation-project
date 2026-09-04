#!/usr/bin/env python3
"""Create an immutable A0509 Diffusion dataset with uniform RGB shapes.

The source LeRobot v3 dataset is never modified.  Front/side videos and all
numeric data are copied byte-for-byte.  The ZED left RGB stream is letterboxed
to the requested output shape so stock LeRobot Diffusion can consume all
camera features in one tensor.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import shutil
import subprocess
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_VIDEO_KEY = "observation.images.zed_rgb"
INCOMPLETE_MARKER = "DIFFUSION_DATASET_INCOMPLETE"


def _run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def _probe_video(path: Path, *, count_frames: bool = True) -> dict[str, Any]:
    command = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
    ]
    if count_frames:
        command.append("-count_frames")
    command.extend(
        [
            "-show_entries",
            "stream=width,height,avg_frame_rate,nb_frames,nb_read_frames,pix_fmt,codec_name",
            "-of",
            "json",
            str(path),
        ]
    )
    result = subprocess.run(command, check=True, text=True, capture_output=True)
    streams = json.loads(result.stdout).get("streams", [])
    if len(streams) != 1:
        raise RuntimeError(f"Expected one video stream in {path}, found {len(streams)}")
    return streams[0]


def _frame_count(probe: dict[str, Any]) -> int:
    for key in ("nb_read_frames", "nb_frames"):
        value = probe.get(key)
        if value not in (None, "N/A"):
            return int(value)
    raise RuntimeError(f"Video probe does not contain a frame count: {probe}")


def _copy_source_except_zed(source: Path, output: Path, video_key: str) -> None:
    output.mkdir(parents=True, exist_ok=False)
    for entry in source.iterdir():
        destination = output / entry.name
        if entry.name != "videos":
            if entry.is_dir():
                shutil.copytree(entry, destination, copy_function=shutil.copy2)
            else:
                shutil.copy2(entry, destination)
            continue

        destination.mkdir()
        for camera_dir in entry.iterdir():
            if camera_dir.name == video_key:
                continue
            shutil.copytree(camera_dir, destination / camera_dir.name, copy_function=shutil.copy2)


def _letterbox_video(source: Path, output: Path, width: int, height: int, crf: int) -> dict[str, Any]:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".tmp.mp4")
    if temporary.exists():
        temporary.unlink()

    video_filter = (
        f"scale={width}:{height}:force_original_aspect_ratio=decrease:flags=lanczos,"
        f"pad={width}:{height}:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1"
    )
    _run(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            str(source),
            "-vf",
            video_filter,
            "-c:v",
            "libx264",
            "-preset",
            "fast",
            "-crf",
            str(crf),
            "-pix_fmt",
            "yuv420p",
            "-g",
            "2",
            "-keyint_min",
            "2",
            "-sc_threshold",
            "0",
            "-an",
            "-movflags",
            "+faststart",
            str(temporary),
        ]
    )

    source_probe = _probe_video(source)
    output_probe = _probe_video(temporary)
    if _frame_count(source_probe) != _frame_count(output_probe):
        raise RuntimeError(
            f"Frame count changed while converting {source}: "
            f"{_frame_count(source_probe)} -> {_frame_count(output_probe)}"
        )
    if (int(output_probe["width"]), int(output_probe["height"])) != (width, height):
        raise RuntimeError(f"Unexpected converted shape for {source}: {output_probe}")

    temporary.replace(output)
    return {
        "source": str(source),
        "output": str(output),
        "frames": _frame_count(output_probe),
        "source_shape": [int(source_probe["height"]), int(source_probe["width"]), 3],
        "output_shape": [height, width, 3],
    }


def _nested_channels(values: list[float]) -> list[list[list[float]]]:
    return [[[float(value)]] for value in values]


def _quantile_from_histogram(histogram: np.ndarray, probability: float) -> float:
    cumulative = np.cumsum(histogram)
    threshold = max(1, math.ceil(float(cumulative[-1]) * probability))
    return float(np.searchsorted(cumulative, threshold, side="left")) / 255.0


def _read_exact(stream: Any, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            break
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _compute_sampled_rgb_stats(
    video_paths: list[Path], width: int, height: int, sample_every: int
) -> tuple[dict[str, Any], int]:
    histograms = np.zeros((3, 256), dtype=np.int64)
    decoded_frames = 0
    sampled_frames = 0
    expected_index = 0

    frame_bytes = width * height * 3
    for video_path in video_paths:
        probe = _probe_video(video_path, count_frames=False)
        if (int(probe["width"]), int(probe["height"])) != (width, height):
            raise RuntimeError(f"Unexpected video shape while computing statistics: {video_path}: {probe}")
        process = subprocess.Popen(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-i",
                str(video_path),
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if process.stdout is None or process.stderr is None:
            raise RuntimeError("Failed to open FFmpeg pipes")
        try:
            while True:
                frame_buffer = _read_exact(process.stdout, frame_bytes)
                if not frame_buffer:
                    break
                if len(frame_buffer) != frame_bytes:
                    raise RuntimeError(f"Truncated raw RGB frame from {video_path}")
                if expected_index % sample_every == 0:
                    frame_rgb = np.frombuffer(frame_buffer, dtype=np.uint8).reshape(height, width, 3)
                    for rgb_channel in range(3):
                        histograms[rgb_channel] += np.bincount(
                            frame_rgb[:, :, rgb_channel].reshape(-1), minlength=256
                        )
                    sampled_frames += 1
                expected_index += 1
                decoded_frames += 1
        finally:
            process.stdout.close()
        error_output = process.stderr.read().decode(errors="replace")
        return_code = process.wait()
        if return_code:
            raise RuntimeError(f"FFmpeg RGB decode failed for {video_path}: {error_output}")

    if sampled_frames == 0:
        raise RuntimeError("No ZED frames were sampled for image statistics")

    levels = np.arange(256, dtype=np.float64) / 255.0
    means: list[float] = []
    stds: list[float] = []
    minima: list[float] = []
    maxima: list[float] = []
    quantiles: dict[str, list[float]] = {key: [] for key in ("q01", "q10", "q50", "q90", "q99")}
    probabilities = {"q01": 0.01, "q10": 0.10, "q50": 0.50, "q90": 0.90, "q99": 0.99}

    for histogram in histograms:
        count = int(histogram.sum())
        mean = float(np.dot(histogram, levels) / count)
        second_moment = float(np.dot(histogram, levels * levels) / count)
        means.append(mean)
        stds.append(math.sqrt(max(0.0, second_moment - mean * mean)))
        occupied = np.flatnonzero(histogram)
        minima.append(float(occupied[0]) / 255.0)
        maxima.append(float(occupied[-1]) / 255.0)
        for key, probability in probabilities.items():
            quantiles[key].append(_quantile_from_histogram(histogram, probability))

    stats: dict[str, Any] = {
        "min": _nested_channels(minima),
        "max": _nested_channels(maxima),
        "mean": _nested_channels(means),
        "std": _nested_channels(stds),
        "count": [int(histograms[0].sum())],
    }
    stats.update({key: _nested_channels(values) for key, values in quantiles.items()})
    return stats, decoded_frames


def _update_metadata(
    output: Path,
    video_key: str,
    width: int,
    height: int,
    crf: int,
    zed_stats: dict[str, Any],
) -> None:
    info_path = output / "meta" / "info.json"
    info = json.loads(info_path.read_text())
    feature = info["features"][video_key]
    feature["shape"] = [height, width, 3]
    feature_info = feature.setdefault("info", {})
    feature_info.update(
        {
            "is_depth_map": False,
            "video.height": height,
            "video.width": width,
            "video.codec": "h264",
            "video.pix_fmt": "yuv420p",
            "video.channels": 3,
            "video.crf": crf,
            "video.preset": "fast",
        }
    )
    info_path.write_text(json.dumps(info, indent=2, ensure_ascii=False) + "\n")

    stats_path = output / "meta" / "stats.json"
    stats = json.loads(stats_path.read_text())
    stats[video_key] = zed_stats
    stats_path.write_text(json.dumps(stats, indent=2, ensure_ascii=False) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--video-key", default=DEFAULT_VIDEO_KEY)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--crf", type=int, default=18)
    parser.add_argument("--stats-sample-every", type=int, default=30)
    parser.add_argument("--source-manifest-sha256", default=None)
    args = parser.parse_args()

    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not (source / "meta" / "info.json").is_file():
        raise SystemExit(f"Source is not a LeRobot dataset: {source}")
    if output.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {output}")
    if source == output or source in output.parents:
        raise SystemExit("Output must not be the source or a child of the source dataset")
    if args.width <= 0 or args.height <= 0 or args.width % 2 or args.height % 2:
        raise SystemExit("Output width and height must be positive even integers")
    if args.stats_sample_every <= 0:
        raise SystemExit("--stats-sample-every must be positive")

    info = json.loads((source / "meta" / "info.json").read_text())
    if info.get("codebase_version") != "v3.0":
        raise SystemExit(f"Expected a LeRobot v3.0 dataset, got {info.get('codebase_version')!r}")
    if args.video_key not in info.get("features", {}):
        raise SystemExit(f"Missing video feature {args.video_key!r}")

    _copy_source_except_zed(source, output, args.video_key)
    marker = output / INCOMPLETE_MARKER
    marker.write_text("Conversion in progress. Do not train from this directory.\n")

    source_video_root = source / "videos" / args.video_key
    output_video_root = output / "videos" / args.video_key
    source_videos = sorted(source_video_root.rglob("*.mp4"))
    if not source_videos:
        raise RuntimeError(f"No ZED videos found under {source_video_root}")

    conversions = []
    for source_video in source_videos:
        relative = source_video.relative_to(source_video_root)
        conversions.append(
            _letterbox_video(
                source_video,
                output_video_root / relative,
                args.width,
                args.height,
                args.crf,
            )
        )

    output_videos = sorted(output_video_root.rglob("*.mp4"))
    zed_stats, decoded_frames = _compute_sampled_rgb_stats(
        output_videos, args.width, args.height, args.stats_sample_every
    )
    expected_frames = sum(item["frames"] for item in conversions)
    if decoded_frames != expected_frames:
        raise RuntimeError(f"OpenCV decoded {decoded_frames} frames, expected {expected_frames}")

    _update_metadata(output, args.video_key, args.width, args.height, args.crf, zed_stats)
    provenance = {
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "source_dataset": str(source),
        "source_manifest_sha256": args.source_manifest_sha256,
        "video_key": args.video_key,
        "transform": "aspect-preserving resize followed by centered black letterbox padding",
        "output_shape_hwc": [args.height, args.width, 3],
        "ffmpeg_codec": "libx264",
        "ffmpeg_crf": args.crf,
        "stats_sample_every": args.stats_sample_every,
        "converted_videos": conversions,
    }
    (output / "meta" / "diffusion_derivation.json").write_text(
        json.dumps(provenance, indent=2, ensure_ascii=False) + "\n"
    )
    marker.unlink()

    print(
        json.dumps(
            {
                "status": "complete",
                "source": str(source),
                "output": str(output),
                "episodes": info["total_episodes"],
                "frames": info["total_frames"],
                "zed_frames_verified": expected_frames,
                "camera_shape_hwc": [args.height, args.width, 3],
            },
            indent=2,
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
