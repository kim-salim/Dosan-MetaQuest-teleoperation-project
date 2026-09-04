#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
checkpoint="${CHECKPOINT_PATH:?Set CHECKPOINT_PATH to a Diffusion pretrained_model directory}"
dataset_root="${DATASET_ROOT:-}"
dataset_repo_id="${DATASET_REPO_ID:-local/a0509_blue_block_v1_20260810_170226}"
runs="${RUNS:-20}"
warmup_runs="${WARMUP_RUNS:-3}"

source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot-diffusion/bin/activate
source "${project_root}/install/setup.bash"
set -u

arguments=(
    --checkpoint "${checkpoint}"
    --device cuda
    --runs "${runs}"
    --warmup-runs "${warmup_runs}"
)
if [[ -n "${dataset_root}" ]]; then
    arguments+=(
        --dataset-root "${dataset_root}"
        --dataset-repo-id "${dataset_repo_id}"
    )
fi

exec python "${project_root}/scripts/benchmark_a0509_diffusion_policy.py" "${arguments[@]}"
