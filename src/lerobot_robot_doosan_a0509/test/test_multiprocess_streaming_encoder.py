import time

import av
import numpy as np

from lerobot.configs.video import RGBEncoderConfig
from lerobot.datasets import lerobot_dataset

from lerobot_robot_doosan_a0509.multiprocess_streaming_encoder import (
    SharedMemoryStreamingVideoEncoder,
    install_shared_memory_streaming_encoder,
)


def test_installer_replaces_dataset_factory_encoder():
    previous = lerobot_dataset.StreamingVideoEncoder
    try:
        returned = install_shared_memory_streaming_encoder()
        assert returned is previous
        assert lerobot_dataset.StreamingVideoEncoder is SharedMemoryStreamingVideoEncoder
    finally:
        lerobot_dataset.StreamingVideoEncoder = previous


def test_spawned_shared_memory_encoder_preserves_every_frame(tmp_path):
    config = RGBEncoderConfig(
        vcodec="h264",
        pix_fmt="yuv420p",
        crf=30,
        preset="ultrafast",
    )
    encoder = SharedMemoryStreamingVideoEncoder(
        fps=10,
        rgb_encoder=config,
        queue_maxsize=10,
        shared_memory_slots=16,
    )
    key = "observation.images.test"
    try:
        assert encoder.worker_info["start_method"] == "spawn"
        encoder.start_episode([key], tmp_path)
        for index in range(10):
            image = np.full((64, 64, 3), index * 10, dtype=np.uint8)
            encoder.feed_frame(key, image)
            time.sleep(0.1)
        results = encoder.finish_episode()
        video_path, stats = results[key]
        assert video_path.exists()
        assert stats is not None
        with av.open(str(video_path)) as container:
            frames = list(container.decode(video=0))
        assert len(frames) == 10
        assert encoder.last_episode_metrics["submitted"] == {key: 10}
        assert encoder.last_episode_metrics["processed"] == {key: 10}
        assert encoder.last_episode_metrics["drops"] == {}
    finally:
        encoder.close()
