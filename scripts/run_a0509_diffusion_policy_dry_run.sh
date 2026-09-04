#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
model_path="${MODEL_PATH:-/home/rvlab/lerobot_models/diffusion_a0509_blue_block_v1_rgb640_ddim5_bs16_50k_20260811_0226/050000/pretrained_model}"
task_description="${TASK_DESCRIPTION:-Pick up the blue block on the table}"
duration_s="${DURATION_S:-60}"
policy_cpu_set="${POLICY_CPU_SET:-6-13}"
queue_threshold="${QUEUE_THRESHOLD:-9}"
overlap_steps="${OVERLAP_STEPS:-3}"
noise_correlation="${NOISE_CORRELATION:-1.0}"
ensemble_history_size="${ENSEMBLE_HISTORY_SIZE:-3}"
ensemble_weight_decay="${ENSEMBLE_WEIGHT_DECAY:-0.5}"
gripper_support_threshold="${GRIPPER_SUPPORT_THRESHOLD:-0.25}"
gripper_close_threshold="${GRIPPER_CLOSE_THRESHOLD:-0.4}"
gripper_window_ticks="${GRIPPER_WINDOW_TICKS:-4}"
gripper_support_ticks="${GRIPPER_SUPPORT_TICKS:-2}"
warmup_inferences="${WARMUP_INFERENCES:-2}"
max_observation_age_sec="${MAX_OBSERVATION_AGE_SEC:-0.25}"
max_action_age_sec="${MAX_ACTION_AGE_SEC:-0.50}"
ros_cpu_set="${ROS_CPU_SET:-6}"
main_cpu_set="${MAIN_CPU_SET:-7-8}"
inference_cpu_set="${INFERENCE_CPU_SET:-9-13}"

if [[ ! -f "${model_path}/model.safetensors" ]]; then
    echo "ERROR: checkpoint not found: ${model_path}" >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot-diffusion/bin/activate
source "${project_root}/install/setup.bash"
set -u

export LEROBOT_A0509_DIFFUSION_WARMUP_INFERENCES="${warmup_inferences}"
export LEROBOT_A0509_DIFFUSION_OVERLAP_STEPS="${overlap_steps}"
export LEROBOT_A0509_DIFFUSION_NOISE_CORRELATION="${noise_correlation}"
export LEROBOT_A0509_DIFFUSION_ENSEMBLE_HISTORY_SIZE="${ensemble_history_size}"
export LEROBOT_A0509_DIFFUSION_ENSEMBLE_WEIGHT_DECAY="${ensemble_weight_decay}"
export LEROBOT_A0509_DIFFUSION_GRIPPER_SUPPORT_THRESHOLD="${gripper_support_threshold}"
export LEROBOT_A0509_DIFFUSION_GRIPPER_CLOSE_THRESHOLD="${gripper_close_threshold}"
export LEROBOT_A0509_DIFFUSION_GRIPPER_WINDOW_TICKS="${gripper_window_ticks}"
export LEROBOT_A0509_DIFFUSION_GRIPPER_SUPPORT_TICKS="${gripper_support_ticks}"
export LEROBOT_A0509_DIFFUSION_MAX_OBSERVATION_AGE_SEC="${max_observation_age_sec}"
export LEROBOT_A0509_DIFFUSION_MAX_ACTION_AGE_SEC="${max_action_age_sec}"
export LEROBOT_A0509_ROS_EXECUTOR_THREADS=1
export LEROBOT_A0509_ROS_CPU_SET="${ros_cpu_set}"
export LEROBOT_A0509_MAIN_CPU_SET="${main_cpu_set}"
export LEROBOT_A0509_DIFFUSION_INFERENCE_CPU_SET="${inference_cpu_set}"

echo "mode=policy_dry_run live_output=false robot_command_publish=false"
echo "model_path=${model_path} n_obs_steps=2 n_action_steps=15 fps=30"
echo "task_description=${task_description}"
echo "async_latest_chunk=true queue_threshold=${queue_threshold} overlap_steps=${overlap_steps}"
echo "noise_correlation=${noise_correlation} ensemble_history=${ensemble_history_size} ensemble_decay=${ensemble_weight_decay}"
echo "diffusion_gripper=window:${gripper_window_ticks} peak>${gripper_close_threshold} support>=${gripper_support_threshold}x${gripper_support_ticks} close_latch=live"
echo "max_observation_age_sec=${max_observation_age_sec} max_action_age_sec=${max_action_age_sec}"
echo "warmup_inferences=${warmup_inferences} policy_cpu_set=${policy_cpu_set} duration_s=${duration_s}"
echo "thread_cpu_sets=ros:${ros_cpu_set} main_camera:${main_cpu_set} diffusion:${inference_cpu_set}"

exec taskset -c "${policy_cpu_set}" \
    python -m lerobot_robot_doosan_a0509.diffusion_rollout_entrypoint \
    --strategy.type=base \
    --inference.type=rtc \
    --inference.rtc.enabled=false \
    --inference.queue_threshold="${queue_threshold}" \
    --policy.path="${model_path}" \
    --policy.n_action_steps=15 \
    --robot.type=doosan_a0509_diffusion_ros \
    --robot.id=a0509_diffusion_async_dry_run \
    --robot.mode=policy_dry_run \
    --robot.require_camera=true \
    --robot.require_fresh_state_on_connect=true \
    --robot.connect_timeout_sec=10 \
    --fps=30 \
    --duration="${duration_s}" \
    --device=cuda \
    --task="${task_description}" \
    --display_data=false \
    --play_sounds=false \
    --return_to_initial_position=false
