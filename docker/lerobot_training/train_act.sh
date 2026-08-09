#!/usr/bin/env bash
set -euo pipefail

: "${DATASET_REPO_ID:?Set DATASET_REPO_ID, for example rvlab/a0509_metaquest_dataset}"

DATASET_ROOT="${DATASET_ROOT:-/workspace/data/a0509_metaquest_dataset}"
RUN_NAME="${RUN_NAME:-act_a0509_smoke}"
STEPS="${STEPS:-1000}"
BATCH_SIZE="${BATCH_SIZE:-8}"
EVAL_SPLIT="${EVAL_SPLIT:-0.0}"
EVAL_STEPS="${EVAL_STEPS:-0}"
NUM_WORKERS="${NUM_WORKERS:-4}"

test -d "${DATASET_ROOT}/meta"
test -d "${DATASET_ROOT}/data"

exec lerobot-train \
    --dataset.repo_id="${DATASET_REPO_ID}" \
    --dataset.root="${DATASET_ROOT}" \
    --dataset.eval_split="${EVAL_SPLIT}" \
    --policy.type=act \
    --policy.device=cuda \
    --output_dir="/workspace/outputs/${RUN_NAME}" \
    --job_name="${RUN_NAME}" \
    --batch_size="${BATCH_SIZE}" \
    --steps="${STEPS}" \
    --eval_steps="${EVAL_STEPS}" \
    --num_workers="${NUM_WORKERS}" \
    --wandb.enable=false \
    --policy.push_to_hub=false
