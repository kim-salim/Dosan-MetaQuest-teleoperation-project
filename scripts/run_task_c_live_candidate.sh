#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
checkpoint_a="${CHECKPOINT_A:-/home/rvlab/lerobot_models/act_a0509_blue_block_bs16_30k_20260810_181114/030000/pretrained_model}"
checkpoint_b="${CHECKPOINT_B:-/home/rvlab/lerobot_models/act_a0509_blue_block_v2_bs16_30k_20260810_200240/030000/pretrained_model}"
runtime_manifest="${RUNTIME_MANIFEST:-/home/rvlab/lerobot_datasets/task_c_bridge_v0_closed_holding_20260813/runtime_transition_manifest.json}"
representative_boundary_enabled="${REPRESENTATIVE_BOUNDARY_ENABLED:-false}"
moving_overlap_primary_enabled="${MOVING_OVERLAP_PRIMARY_ENABLED:-false}"
stopped_endpoint_direct_handoff_enabled="${STOPPED_ENDPOINT_DIRECT_HANDOFF_ENABLED:-false}"
stopped_endpoint_position_bridge_enabled="${STOPPED_ENDPOINT_POSITION_BRIDGE_ENABLED:-false}"
run_stamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${OUTPUT_DIR:-/home/rvlab/lerobot_datasets/task_c_live_candidate_${run_stamp}}"
duration_s="${DURATION_S:-180}"
queue_threshold="${QUEUE_THRESHOLD:-20}"
overlap_steps="${OVERLAP_STEPS:-15}"
warmup_inferences="${WARMUP_INFERENCES:-2}"
policy_cpu_set="${POLICY_CPU_SET:-6-13}"
ros_cpu_set="${ROS_CPU_SET:-6}"
main_cpu_set="${MAIN_CPU_SET:-7-8}"
inference_cpu_set="${INFERENCE_CPU_SET:-9-13}"
rollout_pid=""

for required_file in \
    "${checkpoint_a}/model.safetensors" \
    "${checkpoint_b}/model.safetensors" \
    "${runtime_manifest}"; do
    if [[ ! -f "${required_file}" ]]; then
        echo "ERROR: required file not found: ${required_file}" >&2
        exit 1
    fi
done
if [[ -e "${output_dir}/task_c_live_events.jsonl" ]]; then
    echo "ERROR: refusing to overwrite an existing run: ${output_dir}" >&2
    exit 1
fi
mkdir -p "${output_dir}"

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

echo "Forcing Live OFF and MUX DISABLED before loading both policies."
force_safe_state

export PYTHONPATH="${project_root}:${PYTHONPATH:-}"
export LEROBOT_A0509_ACT_WARMUP_INFERENCES="${warmup_inferences}"
export LEROBOT_A0509_ACT_OVERLAP_STEPS="${overlap_steps}"
export LEROBOT_A0509_ROS_EXECUTOR_THREADS=1
export LEROBOT_A0509_ROS_CPU_SET="${ros_cpu_set}"
export LEROBOT_A0509_MAIN_CPU_SET="${main_cpu_set}"
export LEROBOT_A0509_ACT_INFERENCE_CPU_SET="${inference_cpu_set}"

echo "mode=task_c_live initial_live_output=false initial_mux_source=DISABLED"
echo "ACT-A=${checkpoint_a}"
echo "ACT-B=${checkpoint_b}"
echo "runtime_manifest=${runtime_manifest}"
echo "representative_boundary_enabled=${representative_boundary_enabled}"
echo "moving_overlap_primary_enabled=${moving_overlap_primary_enabled}"
echo "stopped_endpoint_direct_handoff_enabled=${stopped_endpoint_direct_handoff_enabled}"
echo "stopped_endpoint_position_bridge_enabled=${stopped_endpoint_position_bridge_enabled}"
echo "events=${output_dir}/task_c_live_events.jsonl"
echo "This process cannot select LEROBOT or enable Live; use the separate gate."

taskset -c "${policy_cpu_set}" \
    python -m lerobot_robot_doosan_a0509.task_c_live_entrypoint \
    --strategy.type=task_c_live \
    --strategy.runtime_manifest="${runtime_manifest}" \
    --strategy.checkpoint_b="${checkpoint_b}" \
    --strategy.event_jsonl_path="${output_dir}/task_c_live_events.jsonl" \
    --strategy.acknowledge_uncertified_manifest=true \
    --strategy.representative_boundary_enabled="${representative_boundary_enabled}" \
    --strategy.b_moving_overlap_primary_enabled="${moving_overlap_primary_enabled}" \
    --strategy.b_stopped_endpoint_direct_handoff_enabled="${stopped_endpoint_direct_handoff_enabled}" \
    --strategy.b_stopped_endpoint_position_bridge_enabled="${stopped_endpoint_position_bridge_enabled}" \
    --inference.type=rtc \
    --inference.rtc.enabled=false \
    --inference.queue_threshold="${queue_threshold}" \
    --policy.path="${checkpoint_a}" \
    --policy.n_action_steps=100 \
    --robot.type=doosan_a0509_ros \
    --robot.id=task_c_bounded_live_candidate \
    --robot.mode=policy_live \
    --robot.require_camera=true \
    --robot.require_fresh_state_on_connect=true \
    --robot.connect_timeout_sec=10 \
    --fps=30 \
    --duration="${duration_s}" \
    --device=cuda \
    --task="Pick up the blue block on the table" \
    --display_data=false \
    --play_sounds=false \
    --return_to_initial_position=false &
rollout_pid=$!
wait "${rollout_pid}"
