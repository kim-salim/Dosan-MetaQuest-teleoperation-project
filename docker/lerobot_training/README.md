# A0509 LeRobot training container

This directory is for ACT training on an x86_64 GPU server. ROS 2, the Doosan
driver, MetaQuest, and the A0509 hardware plugin are intentionally not installed:
training consumes only a finalized LeRobot dataset.

The image is pinned to LeRobot 0.6.0 and the same PyTorch/CUDA line used by the
Jetson runtime. Docker Compose requests physical GPU 0 only, and the container
also sets both NVIDIA and CUDA visibility to device 0. `verify_gpu0.sh` fails if
the container sees more than one GPU or a different configured UUID.

## Server setup

```bash
cp .env.example .env
mkdir -p data outputs cache user-home
docker compose build trainer
docker compose run --rm trainer /opt/lerobot/bin/verify_gpu0.sh
```

The dataset mount is read-only. Copy a finalized dataset so that its tree is:

```text
data/a0509_metaquest_dataset/meta/
data/a0509_metaquest_dataset/data/
data/a0509_metaquest_dataset/videos/
```

## Dataset load check

```bash
docker compose run --rm trainer python - <<'PY'
from lerobot.datasets import LeRobotDataset

dataset = LeRobotDataset(
    repo_id="rvlab/a0509_metaquest_dataset",
    root="/workspace/data/a0509_metaquest_dataset",
)
print("episodes", dataset.num_episodes)
print("frames", dataset.num_frames)
print("fps", dataset.fps)
print("features", dataset.features)
PY
```

## 1,000-step smoke training

```bash
docker compose run --rm \
  -e DATASET_REPO_ID=rvlab/a0509_metaquest_dataset \
  -e RUN_NAME=act_a0509_smoke \
  -e STEPS=1000 \
  trainer /opt/lerobot/bin/train_act.sh
```

## Full ACT training

```bash
docker compose run --rm \
  -e DATASET_REPO_ID=rvlab/a0509_metaquest_dataset \
  -e RUN_NAME=act_a0509 \
  -e STEPS=100000 \
  -e EVAL_SPLIT=0.1 \
  -e EVAL_STEPS=5000 \
  trainer /opt/lerobot/bin/train_act.sh
```

Outputs remain on the host under `outputs/`. Return the entire directory below
to the Jetson, not only the model weights:

```text
outputs/act_a0509/checkpoints/last/pretrained_model/
```

## Interactive shell

```bash
docker compose run --rm trainer bash
```

Do not replace the GPU reservation with `gpus: all`; GPU 1 is outside this
project's allocation.
