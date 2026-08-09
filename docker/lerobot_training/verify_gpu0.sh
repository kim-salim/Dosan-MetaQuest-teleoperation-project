#!/usr/bin/env bash
set -euo pipefail

mapfile -t gpu_uuids < <(
    nvidia-smi --query-gpu=uuid --format=csv,noheader | sed '/^[[:space:]]*$/d'
)

if [[ "${#gpu_uuids[@]}" -ne 1 ]]; then
    echo "ERROR: container must see exactly one GPU; found ${#gpu_uuids[@]}" >&2
    exit 1
fi

if [[ -n "${EXPECTED_GPU_UUID:-}" && "${gpu_uuids[0]}" != "${EXPECTED_GPU_UUID}" ]]; then
    echo "ERROR: visible GPU UUID does not match the configured physical GPU 0" >&2
    exit 1
fi

nvidia-smi \
    --query-gpu=index,name,uuid,memory.total,memory.free,driver_version \
    --format=csv,noheader

python - <<'PY'
import importlib.metadata as metadata
import subprocess

import torch

assert torch.cuda.is_available(), "CUDA is unavailable"
assert torch.cuda.device_count() == 1, torch.cuda.device_count()

print("lerobot", metadata.version("lerobot"))
print("torch", torch.__version__)
print("torchvision", metadata.version("torchvision"))
print("torchcodec", metadata.version("torchcodec"))
print("av", metadata.version("av"))
print("cuda_runtime", torch.version.cuda)
print("visible_cuda_devices", torch.cuda.device_count())
print("device_0", torch.cuda.get_device_name(0))

x = torch.randn(2048, 2048, device="cuda")
y = x @ x
torch.cuda.synchronize()
print("cuda_matmul_ok", tuple(y.shape))

subprocess.run(["ffmpeg", "-version"], check=True, stdout=subprocess.DEVNULL)
print("ffmpeg_ok", True)
PY
