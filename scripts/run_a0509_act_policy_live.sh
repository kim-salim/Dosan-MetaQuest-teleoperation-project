#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
model_path="${MODEL_PATH:-/home/rvlab/lerobot_models/act_a0509_red_block_50k_20260809_202117/020000/pretrained_model}"
task_description="${TASK_DESCRIPTION:-Pick up the red block on the table}"
duration_s="${DURATION_S:-300}"
policy_cpu_set="${POLICY_CPU_SET:-6-13}"
queue_threshold="${QUEUE_THRESHOLD:-20}"
overlap_steps="${OVERLAP_STEPS:-15}"
warmup_inferences="${WARMUP_INFERENCES:-2}"
ros_cpu_set="${ROS_CPU_SET:-6}"
main_cpu_set="${MAIN_CPU_SET:-7-8}"
inference_cpu_set="${INFERENCE_CPU_SET:-9-13}"
rollout_pid=""

if [[ ! -f "${model_path}/model.safetensors" ]]; then
    echo "ERROR: checkpoint not found: ${model_path}" >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot/bin/activate
source "${project_root}/install/setup.bash"
set -u

force_safe_state() {
    set +e
    timeout 5 ros2 service call /vr/set_live_robot_output \
        std_srvs/srv/SetBool '{data: false}'
    timeout 5 ros2 service call /control/select_disabled \
        std_srvs/srv/Trigger '{}'
}

cleanup() {
    status=$?
    trap - EXIT INT TERM
    if [[ -n "${rollout_pid}" ]] && kill -0 "${rollout_pid}" 2>/dev/null; then
        kill -INT "${rollout_pid}" 2>/dev/null
        wait "${rollout_pid}" 2>/dev/null
    fi
    force_safe_state
    exit "${status}"
}
trap cleanup EXIT INT TERM

echo "Forcing Live OFF and MUX DISABLED before loading the policy."
force_safe_state

export LEROBOT_A0509_ACT_WARMUP_INFERENCES="${warmup_inferences}"
export LEROBOT_A0509_ACT_OVERLAP_STEPS="${overlap_steps}"
export LEROBOT_A0509_ROS_EXECUTOR_THREADS=1
export LEROBOT_A0509_ROS_CPU_SET="${ros_cpu_set}"
export LEROBOT_A0509_MAIN_CPU_SET="${main_cpu_set}"
export LEROBOT_A0509_ACT_INFERENCE_CPU_SET="${inference_cpu_set}"

echo "mode=policy_live initial_live_output=false initial_mux_source=DISABLED"
echo "model_path=${model_path} n_action_steps=100 fps=30"
echo "task_description=${task_description}"
echo "async_overlap=true queue_threshold=${queue_threshold} overlap_steps=${overlap_steps}"
echo "warmup_inferences=${warmup_inferences} fresh_queue_on_live=true"
echo "policy_cpu_set=${policy_cpu_set} duration_s=${duration_s}"
echo "thread_cpu_sets=ros:${ros_cpu_set} main_camera:${main_cpu_set} act:${inference_cpu_set}"
echo "The script does not select LEROBOT or enable Live automatically."

taskset -c "${policy_cpu_set}" \
    python -m lerobot_robot_doosan_a0509.rollout_entrypoint \
    --strategy.type=base \
    --inference.type=rtc \
    --inference.rtc.enabled=false \
    --inference.queue_threshold="${queue_threshold}" \
    --policy.path="${model_path}" \
    --policy.n_action_steps=100 \
    --robot.type=doosan_a0509_ros \
    --robot.id=a0509_act_async_live \
    --robot.mode=policy_live \
    --robot.require_camera=true \
    --robot.require_fresh_state_on_connect=true \
    --robot.connect_timeout_sec=10 \
    --fps=30 \
    --duration="${duration_s}" \
    --device=cuda \
    --task="${task_description}" \
    --display_data=false \
    --play_sounds=false \
    --return_to_initial_position=false &
rollout_pid=$!
wait "${rollout_pid}"
