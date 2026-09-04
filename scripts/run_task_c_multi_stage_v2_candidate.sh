#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
checkpoint_a="${CHECKPOINT_A:-}"
runtime_manifest="${RUNTIME_MANIFEST:-}"
multi_stage_plan="${MULTI_STAGE_PLAN:-}"
duration_s="${DURATION_S:-300}"
policy_cpu_set="${POLICY_CPU_SET:-6-13}"
ros_cpu_set="${ROS_CPU_SET:-6}"
main_cpu_set="${MAIN_CPU_SET:-7-8}"
inference_cpu_set="${INFERENCE_CPU_SET:-9-12}"
queue_threshold="${QUEUE_THRESHOLD:-20}"
overlap_steps="${OVERLAP_STEPS:-15}"
warmup_inferences="${WARMUP_INFERENCES:-2}"
handoff_window_steps="${HANDOFF_WINDOW_STEPS:-24}"
b_prefix_steps="${B_PREFIX_STEPS:-15}"
crossfade_steps="${CROSSFADE_STEPS:-15}"
max_b_result_age_sec="${MAX_B_RESULT_AGE_SEC:-0.30}"
bridge_admission_mode="${BRIDGE_ADMISSION_MODE:-flexible_level2}"
if [[ "${bridge_admission_mode}" == "flexible_level2" ]]; then
    adaptive_b_max_splice_index="${ADAPTIVE_B_MAX_SPLICE_INDEX:-8}"
    adaptive_b_max_candidates="${ADAPTIVE_B_MAX_CANDIDATES:-6}"
else
    adaptive_b_max_splice_index="${ADAPTIVE_B_MAX_SPLICE_INDEX:-0}"
    adaptive_b_max_candidates="${ADAPTIVE_B_MAX_CANDIDATES:-1}"
fi
flexible_bridge_max_result_age_sec="${FLEXIBLE_BRIDGE_MAX_RESULT_AGE_SEC:-0.30}"
flexible_bridge_max_candidates="${FLEXIBLE_BRIDGE_MAX_CANDIDATES:-64}"
flexible_bridge_max_search_time_s="${FLEXIBLE_BRIDGE_MAX_SEARCH_TIME_S:-0.50}"
flexible_bridge_worker_backend="${FLEXIBLE_BRIDGE_WORKER_BACKEND:-process}"
flexible_bridge_cpu_set="${FLEXIBLE_BRIDGE_CPU_SET:-13}"
flexible_bridge_worker_nice="${FLEXIBLE_BRIDGE_WORKER_NICE:-10}"
flexible_bridge_rebase_retry_interval_s="${FLEXIBLE_BRIDGE_REBASE_RETRY_INTERVAL_S:-0.10}"
flexible_bridge_template_max_attempts="${FLEXIBLE_BRIDGE_TEMPLATE_MAX_ATTEMPTS:-3}"
flexible_bridge_template_setup_timeout_s="${FLEXIBLE_BRIDGE_TEMPLATE_SETUP_TIMEOUT_S:-2.0}"
flexible_predictive_source_splice="${FLEXIBLE_PREDICTIVE_SOURCE_SPLICE:-true}"
flexible_source_splice_nominal_latency_s="${FLEXIBLE_SOURCE_SPLICE_NOMINAL_LATENCY_S:-0.12}"
flexible_source_splice_margin_steps="${FLEXIBLE_SOURCE_SPLICE_MARGIN_STEPS:-2}"
flexible_source_splice_min_lookahead_steps="${FLEXIBLE_SOURCE_SPLICE_MIN_LOOKAHEAD_STEPS:-4}"
flexible_source_splice_max_lookahead_steps="${FLEXIBLE_SOURCE_SPLICE_MAX_LOOKAHEAD_STEPS:-8}"
flexible_source_splice_queue_snapshot_steps="${FLEXIBLE_SOURCE_SPLICE_QUEUE_SNAPSHOT_STEPS:-12}"
flexible_source_splice_late_tolerance_steps="${FLEXIBLE_SOURCE_SPLICE_LATE_TOLERANCE_STEPS:-1}"
flexible_source_splice_velocity_window_steps="${FLEXIBLE_SOURCE_SPLICE_VELOCITY_WINDOW_STEPS:-5}"
command_acceleration_limit_mm_s2="${COMMAND_ACCELERATION_LIMIT_MM_S2:-4000}"
bridge_acceleration_limit_mm_s2="${BRIDGE_ACCELERATION_LIMIT_MM_S2:-${command_acceleration_limit_mm_s2}}"
bridge_jerk_limit_mm_s3="${BRIDGE_JERK_LIMIT_MM_S3:-4000}"
bridge_integrated_squared_jerk_limit="${BRIDGE_INTEGRATED_SQUARED_JERK_LIMIT:-10000000}"
max_crossfade_command_acceleration_mm_s2="${MAX_CROSSFADE_COMMAND_ACCELERATION_MM_S2:-${command_acceleration_limit_mm_s2}}"
max_prefix_velocity_mm_s="${MAX_PREFIX_VELOCITY_MM_S:-300}"
runtime_command_profile_id="${RUNTIME_COMMAND_PROFILE_ID:-}"
downstream_linear_ramp_mm_per_tick="${DOWNSTREAM_LINEAR_RAMP_MM_PER_TICK:-7.5}"
downstream_orientation_ramp_deg_per_tick="${DOWNSTREAM_ORIENTATION_RAMP_DEG_PER_TICK:-1.25}"
bridge_ack_mode="${BRIDGE_ACK_MODE:-bounded_pipeline}"
bridge_max_ack_lag_steps="${BRIDGE_MAX_ACK_LAG_STEPS:-1}"
transition_timeout_s="${TRANSITION_TIMEOUT_S:-12}"
cut_timeout_s="${CUT_TIMEOUT_S:-120}"
b_execution_timeout_s="${B_EXECUTION_TIMEOUT_S:-120}"
b_execution_steps="${B_EXECUTION_STEPS:-2000}"
record_task_c_dataset="${RECORD_TASK_C_DATASET:-false}"
request_next_service="${REQUEST_NEXT_SERVICE:-/control/task_c/request_next_stage}"
complete_service="${COMPLETE_SERVICE:-/control/task_c/complete_episode}"
status_service="${STATUS_SERVICE:-/control/task_c/multi_stage_status}"
run_stamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${OUTPUT_DIR:-/home/rvlab/lerobot_datasets/task_c_multi_v2_${run_stamp}}"
rollout_pid=""

for required_name in CHECKPOINT_A RUNTIME_MANIFEST MULTI_STAGE_PLAN; do
    if [[ -z "${!required_name:-}" ]]; then
        echo "ERROR: set ${required_name}" >&2
        exit 2
    fi
done
for required_file in \
    "${checkpoint_a}/model.safetensors" \
    "${runtime_manifest}" \
    "${multi_stage_plan}"; do
    if [[ ! -f "${required_file}" ]]; then
        echo "ERROR: required file not found: ${required_file}" >&2
        exit 1
    fi
done
if [[ "${record_task_c_dataset}" != "true" && "${record_task_c_dataset}" != "false" ]]; then
    echo "ERROR: RECORD_TASK_C_DATASET must be true or false" >&2
    exit 2
fi
if [[ "${flexible_predictive_source_splice}" != "true" && "${flexible_predictive_source_splice}" != "false" ]]; then
    echo "ERROR: FLEXIBLE_PREDICTIVE_SOURCE_SPLICE must be true or false" >&2
    exit 2
fi
if [[ "${bridge_ack_mode}" != "stop_and_wait" && "${bridge_ack_mode}" != "bounded_pipeline" ]]; then
    echo "ERROR: BRIDGE_ACK_MODE must be stop_and_wait or bounded_pipeline" >&2
    exit 2
fi
if [[ "${bridge_admission_mode}" != "strict_level2" && "${bridge_admission_mode}" != "flexible_level2" ]]; then
    echo "ERROR: BRIDGE_ADMISSION_MODE must be strict_level2 or flexible_level2" >&2
    exit 2
fi
if [[ "${flexible_bridge_worker_backend}" != "thread" && "${flexible_bridge_worker_backend}" != "process" ]]; then
    echo "ERROR: FLEXIBLE_BRIDGE_WORKER_BACKEND must be thread or process" >&2
    exit 2
fi
if [[ "${bridge_ack_mode}" == "bounded_pipeline" && "${bridge_max_ack_lag_steps}" != "1" ]]; then
    echo "ERROR: bounded_pipeline requires BRIDGE_MAX_ACK_LAG_STEPS=1" >&2
    exit 2
fi
if [[ -e "${output_dir}" ]]; then
    echo "ERROR: refusing to overwrite output directory: ${output_dir}" >&2
    exit 1
fi
mkdir -p "${output_dir}"

set +u
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

echo "Forcing Live OFF and MUX DISABLED before loading multi-stage V2."
force_safe_state

export PYTHONPATH="${project_root}/src/lerobot_robot_doosan_a0509:${project_root}:${PYTHONPATH:-}"
export LEROBOT_A0509_ACT_WARMUP_INFERENCES="${warmup_inferences}"
export LEROBOT_A0509_ACT_OVERLAP_STEPS="${overlap_steps}"
export LEROBOT_A0509_ROS_EXECUTOR_THREADS=1
export LEROBOT_A0509_ROS_CPU_SET="${ros_cpu_set}"
export LEROBOT_A0509_MAIN_CPU_SET="${main_cpu_set}"
export LEROBOT_A0509_ACT_INFERENCE_CPU_SET="${inference_cpu_set}"
export LEROBOT_A0509_FLEXIBLE_BRIDGE_CPU_SET="${flexible_bridge_cpu_set}"

dataset_args=()
if [[ "${record_task_c_dataset}" == "true" ]]; then
    dataset_repo_id="${DATASET_REPO_ID:-local/rollout_task_c_multi_v2}"
    dataset_root="${DATASET_ROOT:-${output_dir}/lerobot_dataset}"
    task_description="${TASK_DESCRIPTION:-Multi-stage composed Task-C demonstration}"
    dataset_args+=(
        --dataset.repo_id="${dataset_repo_id}"
        --dataset.root="${dataset_root}"
        --dataset.fps=30
        --dataset.single_task="${task_description}"
        --dataset.video=true
        --dataset.push_to_hub=false
        --dataset.streaming_encoding=true
        --dataset.num_image_writer_processes=0
        --dataset.num_image_writer_threads_per_camera=0
    )
fi

echo "mode=task_c_multi_live_v2 handoff_mode=async_window_v2"
echo "initial_policy=${checkpoint_a}"
echo "multi_stage_plan=${multi_stage_plan}"
echo "shared_runtime_safety_manifest=${runtime_manifest}"
echo "handoff_window_steps=${handoff_window_steps} prefix_steps=${b_prefix_steps} crossfade_steps=${crossfade_steps}"
echo "bridge_admission_mode=${bridge_admission_mode} adaptive_b_max_splice_index=${adaptive_b_max_splice_index} adaptive_b_max_candidates=${adaptive_b_max_candidates}"
echo "cpu_sets=ros:${ros_cpu_set} main_camera:${main_cpu_set} act:${inference_cpu_set} bridge:${flexible_bridge_cpu_set}"
echo "flexible_bridge_worker=${flexible_bridge_worker_backend} nice=${flexible_bridge_worker_nice} heavy_search=cache_once fresh_work=adaptive_entry"
echo "nominal_template=pre_live max_attempts=${flexible_bridge_template_max_attempts} setup_timeout_s=${flexible_bridge_template_setup_timeout_s}"
echo "source_splice=predictive:${flexible_predictive_source_splice} nominal_latency_s:${flexible_source_splice_nominal_latency_s} lookahead:${flexible_source_splice_min_lookahead_steps}-${flexible_source_splice_max_lookahead_steps} margin_steps:${flexible_source_splice_margin_steps} queue_snapshot_steps:${flexible_source_splice_queue_snapshot_steps} late_tolerance_steps:${flexible_source_splice_late_tolerance_steps}"
axis_velocity_mm_s="$(awk -v step="${downstream_linear_ramp_mm_per_tick}" 'BEGIN { printf "%.3f", step * 30.0 }')"
echo "command_profile=id:${runtime_command_profile_id:-unbound_legacy} linear_tick:${downstream_linear_ramp_mm_per_tick} rotation_tick:${downstream_orientation_ramp_deg_per_tick} axis_velocity_mm_s:${axis_velocity_mm_s} cartesian_velocity_mm_s:${max_prefix_velocity_mm_s}"
echo "command_dynamics=acceleration:${bridge_acceleration_limit_mm_s2} jerk:${bridge_jerk_limit_mm_s3} integrated_squared_jerk:${bridge_integrated_squared_jerk_limit} crossfade_acceleration:${max_crossfade_command_acceleration_mm_s2}"
echo "bridge_ack_mode=${bridge_ack_mode} max_ack_lag_steps=${bridge_max_ack_lag_steps}"
echo "endpoint_fallback=edge_endpoint_hold_fresh_successor"
echo "request_next_service=${request_next_service}"
echo "complete_service=${complete_service}"
echo "status_service=${status_service}"
echo "trace=${output_dir}/task_c_multi_v2_trace.jsonl"
echo "Each stage cut follows multi_stage_plan: external_planner or phase_supervisor."
echo "policy_inference_end is a configured finite horizon, not a learned ACT done token."
echo "This process does not select LEROBOT and does not enable Live."

taskset -c "${policy_cpu_set}" \
    python -m lerobot_robot_doosan_a0509.task_c_multi_live_v2_entrypoint \
    --strategy.type=task_c_multi_live_v2 \
    --strategy.handoff_mode=async_window_v2 \
    --strategy.bridge_admission_mode="${bridge_admission_mode}" \
    --strategy.runtime_manifest="${runtime_manifest}" \
    --strategy.multi_stage_plan="${multi_stage_plan}" \
    --strategy.event_jsonl_path="${output_dir}/task_c_multi_v2_setup_placeholder.jsonl" \
    --strategy.v2_trace_jsonl_path="${output_dir}/task_c_multi_v2_trace.jsonl" \
    --strategy.v2_trace_csv_path="${output_dir}/task_c_multi_v2_trace.csv" \
    --strategy.acknowledge_uncertified_manifest=true \
    --strategy.representative_boundary_enabled=true \
    --strategy.semantic_authority=external_planner \
    --strategy.source_trigger_mode=median_sphere \
    --strategy.successor_runtime_phase_gate=false \
    --strategy.b_release_position_gate_enabled=false \
    --strategy.b_completion_mode=successor_owned \
    --strategy.b_completion_position_gate_enabled=false \
    --strategy.cut_timeout_s="${cut_timeout_s}" \
    --strategy.b_execution_timeout_s="${b_execution_timeout_s}" \
    --strategy.b_execution_steps="${b_execution_steps}" \
    --strategy.transition_timeout_s="${transition_timeout_s}" \
    --strategy.handoff_window_steps="${handoff_window_steps}" \
    --strategy.b_prefix_steps="${b_prefix_steps}" \
    --strategy.crossfade_steps="${crossfade_steps}" \
    --strategy.runtime_command_profile_id="${runtime_command_profile_id}" \
    --strategy.max_b_result_age_sec="${max_b_result_age_sec}" \
    --strategy.max_prefix_velocity_mm_s="${max_prefix_velocity_mm_s}" \
    --strategy.downstream_linear_ramp_mm_per_tick="${downstream_linear_ramp_mm_per_tick}" \
    --strategy.downstream_orientation_ramp_deg_per_tick="${downstream_orientation_ramp_deg_per_tick}" \
    --strategy.bridge_acceleration_limit_mm_s2="${bridge_acceleration_limit_mm_s2}" \
    --strategy.bridge_jerk_limit_mm_s3="${bridge_jerk_limit_mm_s3}" \
    --strategy.bridge_integrated_squared_jerk_limit="${bridge_integrated_squared_jerk_limit}" \
    --strategy.max_crossfade_command_acceleration_mm_s2="${max_crossfade_command_acceleration_mm_s2}" \
    --strategy.max_inflight_b_requests=1 \
    --strategy.adaptive_b_max_splice_index="${adaptive_b_max_splice_index}" \
    --strategy.adaptive_b_max_candidates="${adaptive_b_max_candidates}" \
    --strategy.flexible_bridge_max_result_age_sec="${flexible_bridge_max_result_age_sec}" \
    --strategy.flexible_bridge_max_candidates="${flexible_bridge_max_candidates}" \
    --strategy.flexible_bridge_max_search_time_s="${flexible_bridge_max_search_time_s}" \
    --strategy.flexible_bridge_worker_backend="${flexible_bridge_worker_backend}" \
    --strategy.flexible_bridge_cpu_set="${flexible_bridge_cpu_set}" \
    --strategy.flexible_bridge_worker_nice="${flexible_bridge_worker_nice}" \
    --strategy.flexible_bridge_rebase_retry_interval_s="${flexible_bridge_rebase_retry_interval_s}" \
    --strategy.flexible_bridge_template_max_attempts="${flexible_bridge_template_max_attempts}" \
    --strategy.flexible_bridge_template_setup_timeout_s="${flexible_bridge_template_setup_timeout_s}" \
    --strategy.flexible_predictive_source_splice="${flexible_predictive_source_splice}" \
    --strategy.flexible_source_splice_nominal_latency_s="${flexible_source_splice_nominal_latency_s}" \
    --strategy.flexible_source_splice_margin_steps="${flexible_source_splice_margin_steps}" \
    --strategy.flexible_source_splice_min_lookahead_steps="${flexible_source_splice_min_lookahead_steps}" \
    --strategy.flexible_source_splice_max_lookahead_steps="${flexible_source_splice_max_lookahead_steps}" \
    --strategy.flexible_source_splice_queue_snapshot_steps="${flexible_source_splice_queue_snapshot_steps}" \
    --strategy.flexible_source_splice_late_tolerance_steps="${flexible_source_splice_late_tolerance_steps}" \
    --strategy.flexible_source_splice_velocity_window_steps="${flexible_source_splice_velocity_window_steps}" \
    --strategy.allow_intermediate_release=false \
    --strategy.enable_soft_handoff=true \
    --strategy.enable_endpoint_fallback=true \
    --strategy.bridge_ack_mode="${bridge_ack_mode}" \
    --strategy.bridge_max_ack_lag_steps="${bridge_max_ack_lag_steps}" \
    --strategy.request_next_service="${request_next_service}" \
    --strategy.complete_service="${complete_service}" \
    --strategy.status_service="${status_service}" \
    --strategy.record_task_c_dataset="${record_task_c_dataset}" \
    --inference.type=rtc \
    --inference.rtc.enabled=false \
    --inference.queue_threshold="${queue_threshold}" \
    --policy.path="${checkpoint_a}" \
    --policy.n_action_steps=100 \
    --robot.type=doosan_a0509_ros \
    --robot.id=task_c_multi_stage_v2 \
    --robot.mode=policy_live \
    --robot.require_camera=true \
    --robot.require_fresh_state_on_connect=true \
    --robot.connect_timeout_sec=10 \
    --fps=30 \
    --duration="${duration_s}" \
    --device=cuda \
    --task="Task-C multi-stage async handoff V2" \
    --display_data=false \
    --play_sounds=false \
    --return_to_initial_position=false \
    "${dataset_args[@]}" &
rollout_pid=$!
wait "${rollout_pid}"
