#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
model_path="${MODEL_PATH:-${HOME}/lerobot_models/act_a0509_red_block_50k_20260809_202117/020000/pretrained_model}"
task_description="${TASK_DESCRIPTION:-Pick up the red block on the table}"
duration_s="${DURATION_S:-60}"
policy_cpu_set="${POLICY_CPU_SET:-6-13}"
queue_threshold="${QUEUE_THRESHOLD:-20}"
overlap_steps="${OVERLAP_STEPS:-15}"
warmup_inferences="${WARMUP_INFERENCES:-2}"
ros_cpu_set="${ROS_CPU_SET:-6}"
main_cpu_set="${MAIN_CPU_SET:-7-8}"
inference_cpu_set="${INFERENCE_CPU_SET:-9-13}"

if [[ ! -f "${model_path}/model.safetensors" ]]; then
    echo "ERROR: checkpoint not found: ${model_path}" >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "${HOME}/venvs/lerobot/bin/activate"
source "${project_root}/install/setup.bash"
set -u

export LEROBOT_A0509_ACT_WARMUP_INFERENCES="${warmup_inferences}"
export LEROBOT_A0509_ACT_OVERLAP_STEPS="${overlap_steps}"
export LEROBOT_A0509_ROS_EXECUTOR_THREADS=1
export LEROBOT_A0509_ROS_CPU_SET="${ros_cpu_set}"
export LEROBOT_A0509_MAIN_CPU_SET="${main_cpu_set}"
export LEROBOT_A0509_ACT_INFERENCE_CPU_SET="${inference_cpu_set}"

echo "mode=policy_dry_run live_output=false mux_source=DISABLED"
echo "model_path=${model_path} n_action_steps=100 fps=30"
echo "task_description=${task_description}"
echo "async_overlap=true queue_threshold=${queue_threshold} overlap_steps=${overlap_steps} warmup_inferences=${warmup_inferences}"
echo "policy_cpu_set=${policy_cpu_set} duration_s=${duration_s}"
echo "thread_cpu_sets=ros:${ros_cpu_set} main_camera:${main_cpu_set} act:${inference_cpu_set}"

exec taskset -c "${policy_cpu_set}" \
    python -m lerobot_robot_doosan_a0509.rollout_entrypoint \
    --strategy.type=base \
    --inference.type=rtc \
    --inference.rtc.enabled=false \
    --inference.queue_threshold="${queue_threshold}" \
    --policy.path="${model_path}" \
    --policy.n_action_steps=100 \
    --robot.type=doosan_a0509_ros \
    --robot.id=a0509_act_async_dry_run \
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
