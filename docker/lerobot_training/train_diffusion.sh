#!/usr/bin/env bash
set -euo pipefail

: "${DATASET_REPO_ID:?Set DATASET_REPO_ID}"

DATASET_ROOT="${DATASET_ROOT:-/workspace/data/a0509_blue_block_v1_diffusion_rgb640_letterbox_v1}"
RUN_NAME="${RUN_NAME:-diffusion_a0509_blue_block_v1_smoke}"
STEPS="${STEPS:-1000}"
BATCH_SIZE="${BATCH_SIZE:-4}"
EVAL_SPLIT="${EVAL_SPLIT:-0.0}"
EVAL_STEPS="${EVAL_STEPS:-0}"
NUM_WORKERS="${NUM_WORKERS:-4}"
SAVE_FREQ="${SAVE_FREQ:-10000}"
LOG_FREQ="${LOG_FREQ:-100}"
SEED="${SEED:-100000}"

test -d "${DATASET_ROOT}/meta"
test -d "${DATASET_ROOT}/data"
test -d "${DATASET_ROOT}/videos"
test ! -e "${DATASET_ROOT}/DIFFUSION_DATASET_INCOMPLETE"

exec lerobot-train \
    --dataset.repo_id="${DATASET_REPO_ID}" \
    --dataset.root="${DATASET_ROOT}" \
    --dataset.eval_split="${EVAL_SPLIT}" \
    --policy.type=diffusion \
    --policy.device=cuda \
    --policy.use_amp=true \
    --policy.n_obs_steps=2 \
    --policy.horizon=16 \
    --policy.n_action_steps=8 \
    --policy.drop_n_last_frames=7 \
    --policy.vision_backbone=resnet18 \
    --policy.use_separate_rgb_encoder_per_camera=true \
    --policy.noise_scheduler_type=DDIM \
    --policy.num_train_timesteps=100 \
    --policy.num_inference_steps=5 \
    --policy.compile_model=false \
    --policy.push_to_hub=false \
    --output_dir="/workspace/outputs/${RUN_NAME}" \
    --job_name="${RUN_NAME}" \
    --seed="${SEED}" \
    --batch_size="${BATCH_SIZE}" \
    --steps="${STEPS}" \
    --eval_steps="${EVAL_STEPS}" \
    --num_workers="${NUM_WORKERS}" \
    --log_freq="${LOG_FREQ}" \
    --save_checkpoint=true \
    --save_freq="${SAVE_FREQ}" \
    --wandb.enable=false
