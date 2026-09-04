#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
checkpoint_a="${CHECKPOINT_A:-}"
checkpoint_b="${CHECKPOINT_B:-}"
runtime_manifest="${RUNTIME_MANIFEST:-}"
handoff_manifest="${HANDOFF_EPISODE_MANIFEST:-}"
duration_s="${DURATION_S:-180}"
queue_threshold="${QUEUE_THRESHOLD:-20}"
overlap_steps="${OVERLAP_STEPS:-15}"
warmup_inferences="${WARMUP_INFERENCES:-2}"
policy_cpu_set="${POLICY_CPU_SET:-6-13}"
ros_cpu_set="${ROS_CPU_SET:-6}"
main_cpu_set="${MAIN_CPU_SET:-7-8}"
bridge_admission_mode="${BRIDGE_ADMISSION_MODE:-strict_level2}"
default_inference_cpu_set="9-13"
if [[ "${bridge_admission_mode}" == "flexible_level2" ]]; then
    default_inference_cpu_set="9-12"
    default_bridge_jerk_limit_mm_s3="4000"
    default_bridge_integrated_squared_jerk_limit="10000000"
    default_downstream_linear_ramp_mm_per_tick="7.5"
    default_downstream_orientation_ramp_deg_per_tick="1.25"
else
    default_bridge_jerk_limit_mm_s3="800"
    default_bridge_integrated_squared_jerk_limit="1000000"
    default_downstream_linear_ramp_mm_per_tick="6.67"
    default_downstream_orientation_ramp_deg_per_tick="1.0"
fi
inference_cpu_set="${INFERENCE_CPU_SET:-${default_inference_cpu_set}}"
flexible_bridge_worker_backend="${FLEXIBLE_BRIDGE_WORKER_BACKEND:-process}"
flexible_bridge_cpu_set="${FLEXIBLE_BRIDGE_CPU_SET:-13}"
flexible_bridge_worker_nice="${FLEXIBLE_BRIDGE_WORKER_NICE:-10}"
flexible_bridge_rebase_retry_interval_s="${FLEXIBLE_BRIDGE_REBASE_RETRY_INTERVAL_S:-0.10}"
flexible_bridge_template_max_attempts="${FLEXIBLE_BRIDGE_TEMPLATE_MAX_ATTEMPTS:-3}"
flexible_bridge_template_setup_timeout_s="${FLEXIBLE_BRIDGE_TEMPLATE_SETUP_TIMEOUT_S:-2.0}"
handoff_window_steps="${HANDOFF_WINDOW_STEPS:-24}"
b_prefix_steps="${B_PREFIX_STEPS:-15}"
crossfade_steps="${CROSSFADE_STEPS:-15}"
max_b_result_age_sec="${MAX_B_RESULT_AGE_SEC:-0.30}"
max_first_xyz_axis_delta_mm="${MAX_FIRST_XYZ_AXIS_DELTA_MM:-}"
max_first_rotation_delta_deg="${MAX_FIRST_ROTATION_DELTA_DEG:-}"
max_prefix_velocity_mm_s="${MAX_PREFIX_VELOCITY_MM_S:-}"
max_prefix_acceleration_mm_s2="${MAX_PREFIX_ACCELERATION_MM_S2:-}"
command_acceleration_limit_mm_s2="${COMMAND_ACCELERATION_LIMIT_MM_S2:-4000}"
bridge_acceleration_limit_mm_s2="${BRIDGE_ACCELERATION_LIMIT_MM_S2:-${command_acceleration_limit_mm_s2}}"
bridge_jerk_limit_mm_s3="${BRIDGE_JERK_LIMIT_MM_S3:-${default_bridge_jerk_limit_mm_s3}}"
bridge_integrated_squared_jerk_limit="${BRIDGE_INTEGRATED_SQUARED_JERK_LIMIT:-${default_bridge_integrated_squared_jerk_limit}}"
downstream_linear_ramp_mm_per_tick="${DOWNSTREAM_LINEAR_RAMP_MM_PER_TICK:-${default_downstream_linear_ramp_mm_per_tick}}"
downstream_orientation_ramp_deg_per_tick="${DOWNSTREAM_ORIENTATION_RAMP_DEG_PER_TICK:-${default_downstream_orientation_ramp_deg_per_tick}}"
max_crossfade_command_acceleration_mm_s2="${MAX_CROSSFADE_COMMAND_ACCELERATION_MM_S2:-${command_acceleration_limit_mm_s2}}"
max_bridge_prefix_velocity_mismatch_mm_s="${MAX_BRIDGE_PREFIX_VELOCITY_MISMATCH_MM_S:-}"
max_crossfade_xyz_axis_step_mm="${MAX_CROSSFADE_XYZ_AXIS_STEP_MM:-}"
max_crossfade_rotation_step_deg="${MAX_CROSSFADE_ROTATION_STEP_DEG:-}"
enable_endpoint_fallback="${ENABLE_ENDPOINT_FALLBACK:-true}"
b_release_position_gate_enabled="${B_RELEASE_POSITION_GATE_ENABLED:-false}"
b_release_workspace_min_xyz_mm="${B_RELEASE_WORKSPACE_MIN_XYZ_MM:-}"
b_release_workspace_max_xyz_mm="${B_RELEASE_WORKSPACE_MAX_XYZ_MM:-}"
b_completion_mode="${B_COMPLETION_MODE:-successor_owned}"
b_completion_position_gate_enabled="${B_COMPLETION_POSITION_GATE_ENABLED:-false}"
b_completion_workspace_min_xyz_mm="${B_COMPLETION_WORKSPACE_MIN_XYZ_MM:-}"
b_completion_workspace_max_xyz_mm="${B_COMPLETION_WORKSPACE_MAX_XYZ_MM:-}"
transition_timeout_s="${TRANSITION_TIMEOUT_S:-12}"
bridge_ack_mode="${BRIDGE_ACK_MODE:-stop_and_wait}"
bridge_max_ack_lag_steps="${BRIDGE_MAX_ACK_LAG_STEPS:-0}"
record_task_c_dataset="${RECORD_TASK_C_DATASET:-false}"
source_trigger_mode="${SOURCE_TRIGGER_MODE:-median_sphere}"
source_phase_artifact="${SOURCE_PHASE_ARTIFACT:-}"
source_phase_half_width="${SOURCE_PHASE_HALF_WIDTH:-0.05}"
source_phase_prearm_extra="${SOURCE_PHASE_PREARM_EXTRA:-0.02}"
source_phase_persistence_ticks="${SOURCE_PHASE_PERSISTENCE_TICKS:-3}"
source_phase_local_search_radius="${SOURCE_PHASE_LOCAL_SEARCH_RADIUS:-5}"
source_phase_backward_tolerance="${SOURCE_PHASE_BACKWARD_TOLERANCE:-0.02}"
source_phase_deadline_extra="${SOURCE_PHASE_DEADLINE_EXTRA:-0.0}"
source_support_distance_threshold_mm="${SOURCE_SUPPORT_DISTANCE_THRESHOLD_MM:-}"
source_support_loo_quantile="${SOURCE_SUPPORT_LOO_QUANTILE:-0.95}"
run_stamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${OUTPUT_DIR:-/home/rvlab/lerobot_datasets/task_c_async_v2_${run_stamp}}"
rollout_pid=""

for required_name in CHECKPOINT_A CHECKPOINT_B RUNTIME_MANIFEST HANDOFF_EPISODE_MANIFEST; do
    if [[ -z "${!required_name:-}" ]]; then
        echo "ERROR: set ${required_name}" >&2
        exit 2
    fi
done
for required_file in \
    "${checkpoint_a}/model.safetensors" \
    "${checkpoint_b}/model.safetensors" \
    "${runtime_manifest}" \
    "${handoff_manifest}"; do
    if [[ ! -f "${required_file}" ]]; then
        echo "ERROR: required file not found: ${required_file}" >&2
        exit 1
    fi
done
if [[ "${record_task_c_dataset}" != "true" && "${record_task_c_dataset}" != "false" ]]; then
    echo "ERROR: RECORD_TASK_C_DATASET must be true or false" >&2
    exit 2
fi
if [[ "${enable_endpoint_fallback}" != "true" && "${enable_endpoint_fallback}" != "false" ]]; then
    echo "ERROR: ENABLE_ENDPOINT_FALLBACK must be true or false" >&2
    exit 2
fi
if [[ "${b_release_position_gate_enabled}" != "true" && "${b_release_position_gate_enabled}" != "false" ]]; then
    echo "ERROR: B_RELEASE_POSITION_GATE_ENABLED must be true or false" >&2
    exit 2
fi
if [[ "${b_completion_position_gate_enabled}" != "true" && "${b_completion_position_gate_enabled}" != "false" ]]; then
    echo "ERROR: B_COMPLETION_POSITION_GATE_ENABLED must be true or false" >&2
    exit 2
fi
if [[ "${b_completion_mode}" != "legacy_release_settle" && "${b_completion_mode}" != "successor_owned" ]]; then
    echo "ERROR: B_COMPLETION_MODE must be legacy_release_settle or successor_owned" >&2
    exit 2
fi
if [[ "${b_completion_mode}" == "successor_owned" && "${b_completion_position_gate_enabled}" == "true" ]]; then
    echo "ERROR: successor_owned cannot use B_COMPLETION_POSITION_GATE_ENABLED=true" >&2
    exit 2
fi
if [[ "${source_trigger_mode}" != "median_sphere" && "${source_trigger_mode}" != "phase_support" ]]; then
    echo "ERROR: SOURCE_TRIGGER_MODE must be median_sphere or phase_support" >&2
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
if [[ "${bridge_ack_mode}" != "stop_and_wait" && "${bridge_ack_mode}" != "bounded_pipeline" ]]; then
    echo "ERROR: BRIDGE_ACK_MODE must be stop_and_wait or bounded_pipeline" >&2
    exit 2
fi
if [[ "${bridge_max_ack_lag_steps}" != "0" && "${bridge_max_ack_lag_steps}" != "1" ]]; then
    echo "ERROR: BRIDGE_MAX_ACK_LAG_STEPS must be 0 or 1" >&2
    exit 2
fi
if [[ "${bridge_ack_mode}" == "bounded_pipeline" && "${bridge_max_ack_lag_steps}" != "1" ]]; then
    echo "ERROR: bounded_pipeline requires BRIDGE_MAX_ACK_LAG_STEPS=1" >&2
    exit 2
fi
if [[ "${source_trigger_mode}" == "phase_support" ]]; then
    if [[ -z "${source_phase_artifact}" || ! -f "${source_phase_artifact}" ]]; then
        echo "ERROR: phase_support requires an existing SOURCE_PHASE_ARTIFACT" >&2
        exit 2
    fi
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

echo "Forcing Live OFF and MUX DISABLED before loading V2."
force_safe_state

export PYTHONPATH="${project_root}/src/lerobot_robot_doosan_a0509:${project_root}:${PYTHONPATH:-}"
export LEROBOT_A0509_ACT_WARMUP_INFERENCES="${warmup_inferences}"
export LEROBOT_A0509_ACT_OVERLAP_STEPS="${overlap_steps}"
export LEROBOT_A0509_ROS_EXECUTOR_THREADS=1
export LEROBOT_A0509_ROS_CPU_SET="${ros_cpu_set}"
export LEROBOT_A0509_MAIN_CPU_SET="${main_cpu_set}"
export LEROBOT_A0509_ACT_INFERENCE_CPU_SET="${inference_cpu_set}"
export LEROBOT_A0509_FLEXIBLE_BRIDGE_CPU_SET="${flexible_bridge_cpu_set}"

compatibility_args=()
[[ -n "${max_first_xyz_axis_delta_mm}" ]] && compatibility_args+=(--strategy.max_first_xyz_axis_delta_mm="${max_first_xyz_axis_delta_mm}")
[[ -n "${max_first_rotation_delta_deg}" ]] && compatibility_args+=(--strategy.max_first_rotation_delta_deg="${max_first_rotation_delta_deg}")
[[ -n "${max_prefix_velocity_mm_s}" ]] && compatibility_args+=(--strategy.max_prefix_velocity_mm_s="${max_prefix_velocity_mm_s}")
[[ -n "${max_prefix_acceleration_mm_s2}" ]] && compatibility_args+=(--strategy.max_prefix_acceleration_mm_s2="${max_prefix_acceleration_mm_s2}")
compatibility_args+=(--strategy.max_crossfade_command_acceleration_mm_s2="${max_crossfade_command_acceleration_mm_s2}")
[[ -n "${max_bridge_prefix_velocity_mismatch_mm_s}" ]] && compatibility_args+=(--strategy.max_bridge_prefix_velocity_mismatch_mm_s="${max_bridge_prefix_velocity_mismatch_mm_s}")
[[ -n "${max_crossfade_xyz_axis_step_mm}" ]] && compatibility_args+=(--strategy.max_crossfade_xyz_axis_step_mm="${max_crossfade_xyz_axis_step_mm}")
[[ -n "${max_crossfade_rotation_step_deg}" ]] && compatibility_args+=(--strategy.max_crossfade_rotation_step_deg="${max_crossfade_rotation_step_deg}")

source_phase_args=(
    --strategy.source_trigger_mode="${source_trigger_mode}"
    --strategy.source_phase_half_width="${source_phase_half_width}"
    --strategy.source_phase_prearm_extra="${source_phase_prearm_extra}"
    --strategy.source_phase_persistence_ticks="${source_phase_persistence_ticks}"
    --strategy.source_phase_local_search_radius_indices="${source_phase_local_search_radius}"
    --strategy.source_phase_backward_tolerance="${source_phase_backward_tolerance}"
    --strategy.source_phase_deadline_extra="${source_phase_deadline_extra}"
    --strategy.source_support_loo_quantile="${source_support_loo_quantile}"
    --strategy.successor_runtime_phase_gate=false
)
if [[ "${source_trigger_mode}" == "phase_support" ]]; then
    source_phase_args+=(--strategy.source_phase_artifact_path="${source_phase_artifact}")
fi
[[ -n "${source_support_distance_threshold_mm}" ]] && source_phase_args+=(--strategy.source_support_distance_threshold_mm="${source_support_distance_threshold_mm}")

semantic_completion_args=(
    --strategy.b_release_position_gate_enabled="${b_release_position_gate_enabled}"
    --strategy.b_completion_mode="${b_completion_mode}"
    --strategy.b_completion_position_gate_enabled="${b_completion_position_gate_enabled}"
)
if [[ "${b_release_position_gate_enabled}" == "true" ]]; then
    [[ -n "${b_release_workspace_min_xyz_mm}" ]] && semantic_completion_args+=(--strategy.b_release_workspace_min_xyz_mm="${b_release_workspace_min_xyz_mm}")
    [[ -n "${b_release_workspace_max_xyz_mm}" ]] && semantic_completion_args+=(--strategy.b_release_workspace_max_xyz_mm="${b_release_workspace_max_xyz_mm}")
fi
if [[ "${b_completion_position_gate_enabled}" == "true" ]]; then
    [[ -n "${b_completion_workspace_min_xyz_mm}" ]] && semantic_completion_args+=(--strategy.b_completion_workspace_min_xyz_mm="${b_completion_workspace_min_xyz_mm}")
    [[ -n "${b_completion_workspace_max_xyz_mm}" ]] && semantic_completion_args+=(--strategy.b_completion_workspace_max_xyz_mm="${b_completion_workspace_max_xyz_mm}")
fi

dataset_args=()
if [[ "${record_task_c_dataset}" == "true" ]]; then
    dataset_repo_id="${DATASET_REPO_ID:-local/rollout_task_c_async_v2}"
    dataset_root="${DATASET_ROOT:-${output_dir}/lerobot_dataset}"
    task_description="${TASK_DESCRIPTION:-Composed Task-C demonstration}"
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

echo "mode=task_c_live_v2 handoff_mode=async_window_v2"
echo "ACT-A=${checkpoint_a}"
echo "ACT-B=${checkpoint_b}"
echo "legacy_endpoint_fallback_manifest=${runtime_manifest}"
echo "episode_handoff_manifest=${handoff_manifest}"
echo "handoff_window_steps=${handoff_window_steps} prefix_steps=${b_prefix_steps} crossfade_steps=${crossfade_steps}"
echo "bridge_admission_mode=${bridge_admission_mode}"
echo "command_profile=linear_tick:${downstream_linear_ramp_mm_per_tick} rotation_tick:${downstream_orientation_ramp_deg_per_tick}"
echo "bridge_dynamics=acceleration:${bridge_acceleration_limit_mm_s2} jerk:${bridge_jerk_limit_mm_s3} integrated_squared_jerk:${bridge_integrated_squared_jerk_limit}"
echo "cpu_sets=ros:${ros_cpu_set} main_camera:${main_cpu_set} act:${inference_cpu_set} bridge:${flexible_bridge_cpu_set}"
echo "bridge_ack_mode=${bridge_ack_mode} max_ack_lag_steps=${bridge_max_ack_lag_steps}"
echo "endpoint_fallback=${enable_endpoint_fallback}"
echo "source_trigger_mode=${source_trigger_mode}"
echo "source_phase_artifact=${source_phase_artifact:-none}"
echo "source_phase_args=${source_phase_args[*]}"
echo "successor_runtime_phase_gate=false (phase metadata only)"
echo "compatibility_overrides=${compatibility_args[*]:-none}"
echo "b_release_position_gate_enabled=${b_release_position_gate_enabled}"
echo "b_completion_mode=${b_completion_mode}"
echo "b_completion_position_gate_enabled=${b_completion_position_gate_enabled}"
echo "semantic_completion_overrides=${semantic_completion_args[*]:-none}"
echo "raw_prefix_acceleration=diagnostic_only command_acceleration_limit_mm_s2=bridge:${bridge_acceleration_limit_mm_s2} crossfade:${max_crossfade_command_acceleration_mm_s2}"
echo "trace=${output_dir}/task_c_v2_trace.jsonl"
echo "record_task_c_dataset=${record_task_c_dataset}"
echo "This process does not select LEROBOT and does not enable Live."

taskset -c "${policy_cpu_set}" \
    python -m lerobot_robot_doosan_a0509.task_c_live_v2_entrypoint \
    --strategy.type=task_c_live_v2 \
    --strategy.handoff_mode=async_window_v2 \
    --strategy.bridge_admission_mode="${bridge_admission_mode}" \
    --strategy.runtime_manifest="${runtime_manifest}" \
    --strategy.handoff_episode_manifest="${handoff_manifest}" \
    --strategy.checkpoint_b="${checkpoint_b}" \
    --strategy.event_jsonl_path="${output_dir}/task_c_v2_setup_placeholder.jsonl" \
    --strategy.v2_trace_jsonl_path="${output_dir}/task_c_v2_trace.jsonl" \
    --strategy.v2_trace_csv_path="${output_dir}/task_c_v2_trace.csv" \
    --strategy.acknowledge_uncertified_manifest=true \
    --strategy.representative_boundary_enabled=true \
    --strategy.b_moving_overlap_primary_enabled=false \
    --strategy.b_stopped_endpoint_direct_handoff_enabled=true \
    --strategy.b_stopped_endpoint_position_bridge_enabled=true \
    --strategy.bridge_acceleration_limit_mm_s2="${bridge_acceleration_limit_mm_s2}" \
    --strategy.bridge_jerk_limit_mm_s3="${bridge_jerk_limit_mm_s3}" \
    --strategy.bridge_integrated_squared_jerk_limit="${bridge_integrated_squared_jerk_limit}" \
    --strategy.downstream_linear_ramp_mm_per_tick="${downstream_linear_ramp_mm_per_tick}" \
    --strategy.downstream_orientation_ramp_deg_per_tick="${downstream_orientation_ramp_deg_per_tick}" \
    --strategy.handoff_window_steps="${handoff_window_steps}" \
    --strategy.b_prefix_steps="${b_prefix_steps}" \
    --strategy.crossfade_steps="${crossfade_steps}" \
    --strategy.max_b_result_age_sec="${max_b_result_age_sec}" \
    --strategy.transition_timeout_s="${transition_timeout_s}" \
    --strategy.bridge_ack_mode="${bridge_ack_mode}" \
    --strategy.bridge_max_ack_lag_steps="${bridge_max_ack_lag_steps}" \
    --strategy.max_inflight_b_requests=1 \
    --strategy.flexible_bridge_worker_backend="${flexible_bridge_worker_backend}" \
    --strategy.flexible_bridge_cpu_set="${flexible_bridge_cpu_set}" \
    --strategy.flexible_bridge_worker_nice="${flexible_bridge_worker_nice}" \
    --strategy.flexible_bridge_rebase_retry_interval_s="${flexible_bridge_rebase_retry_interval_s}" \
    --strategy.flexible_bridge_template_max_attempts="${flexible_bridge_template_max_attempts}" \
    --strategy.flexible_bridge_template_setup_timeout_s="${flexible_bridge_template_setup_timeout_s}" \
    --strategy.enable_soft_handoff=true \
    --strategy.enable_endpoint_fallback="${enable_endpoint_fallback}" \
    "${source_phase_args[@]}" \
    "${compatibility_args[@]}" \
    "${semantic_completion_args[@]}" \
    --strategy.record_task_c_dataset="${record_task_c_dataset}" \
    --inference.type=rtc \
    --inference.rtc.enabled=false \
    --inference.queue_threshold="${queue_threshold}" \
    --policy.path="${checkpoint_a}" \
    --policy.n_action_steps=100 \
    --robot.type=doosan_a0509_ros \
    --robot.id=task_c_async_handoff_v2 \
    --robot.mode=policy_live \
    --robot.require_camera=true \
    --robot.require_fresh_state_on_connect=true \
    --robot.connect_timeout_sec=10 \
    --fps=30 \
    --duration="${duration_s}" \
    --device=cuda \
    --task="Task-C async handoff V2" \
    --display_data=false \
    --play_sounds=false \
    --return_to_initial_position=false \
    "${dataset_args[@]}" &
rollout_pid=$!
wait "${rollout_pid}"
