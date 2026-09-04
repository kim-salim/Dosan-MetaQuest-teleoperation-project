#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
checkpoint_a="${CHECKPOINT_A:-/home/rvlab/lerobot_models/act_a0509_blue_block_bs16_30k_20260810_181114/030000/pretrained_model}"
checkpoint_b="${CHECKPOINT_B:-/home/rvlab/lerobot_models/act_a0509_blue_block_v2_bs16_30k_20260810_200240/030000/pretrained_model}"
runtime_manifest="${RUNTIME_MANIFEST:-/home/rvlab/lerobot_datasets/task_c_bridge_v0_closed_holding_20260813/runtime_transition_manifest.json}"
run_stamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${OUTPUT_DIR:-/home/rvlab/lerobot_datasets/task_c_shadow_latency_${run_stamp}}"
duration_s="${DURATION_S:-600}"
max_trials="${MAX_TRIALS:-100}"
warmup_inferences="${WARMUP_INFERENCES:-2}"
stale_after_s="${STALE_AFTER_S:-1.5}"
trial_interval_s="${TRIAL_INTERVAL_S:-0.5}"
trial_timeout_s="${TRIAL_TIMEOUT_S:-15}"
closed_stable_frames="${CLOSED_STABLE_FRAMES:-3}"
policy_cpu_set="${POLICY_CPU_SET:-6-13}"
ros_cpu_set="${ROS_CPU_SET:-6}"
main_cpu_set="${MAIN_CPU_SET:-7-8}"

for required_file in \
    "${checkpoint_a}/model.safetensors" \
    "${checkpoint_b}/model.safetensors" \
    "${runtime_manifest}"; do
    if [[ ! -f "${required_file}" ]]; then
        echo "ERROR: required file not found: ${required_file}" >&2
        exit 1
    fi
done
if [[ -e "${output_dir}/shadow_latency.jsonl" || -e "${output_dir}/shadow_latency.summary.json" ]]; then
    echo "ERROR: refusing to overwrite an existing shadow run: ${output_dir}" >&2
    exit 1
fi
mkdir -p "${output_dir}"

source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot/bin/activate
source "${project_root}/install/setup.bash"
set -u

export PYTHONPATH="${project_root}:${PYTHONPATH:-}"
export LEROBOT_A0509_ROS_EXECUTOR_THREADS=1
export LEROBOT_A0509_ROS_CPU_SET="${ros_cpu_set}"
export LEROBOT_A0509_MAIN_CPU_SET="${main_cpu_set}"

echo "mode=policy_shadow command_publishers=false send_action_guard=true"
echo "ACT-A=${checkpoint_a}"
echo "ACT-B=${checkpoint_b}"
echo "runtime_manifest=${runtime_manifest}"
echo "output=${output_dir} duration_s=${duration_s} max_trials=${max_trials}"
echo "stale_after_s=${stale_after_s} warmup_inferences=${warmup_inferences}"
echo "IMPORTANT: gripper must be stably closed for semantic trial eligibility; this runner never closes it."

exec taskset -c "${policy_cpu_set}" \
    python -m lerobot_robot_doosan_a0509.task_c_shadow_entrypoint \
    --strategy.type=task_c_shadow \
    --strategy.runtime_manifest="${runtime_manifest}" \
    --strategy.checkpoint_b="${checkpoint_b}" \
    --strategy.jsonl_path="${output_dir}/shadow_latency.jsonl" \
    --strategy.summary_path="${output_dir}/shadow_latency.summary.json" \
    --strategy.warmup_inferences="${warmup_inferences}" \
    --strategy.stale_after_s="${stale_after_s}" \
    --strategy.trial_interval_s="${trial_interval_s}" \
    --strategy.trial_timeout_s="${trial_timeout_s}" \
    --strategy.closed_stable_frames="${closed_stable_frames}" \
    --strategy.max_trials="${max_trials}" \
    --inference.type=sync \
    --policy.path="${checkpoint_a}" \
    --policy.n_action_steps=100 \
    --robot.type=doosan_a0509_ros \
    --robot.id=task_c_latency_shadow \
    --robot.mode=policy_shadow \
    --robot.require_camera=true \
    --robot.require_fresh_state_on_connect=true \
    --robot.connect_timeout_sec=10 \
    --fps=30 \
    --duration="${duration_s}" \
    --device=cuda \
    --task="Pick up the blue block on the table" \
    --display_data=false \
    --play_sounds=false \
    --return_to_initial_position=false
