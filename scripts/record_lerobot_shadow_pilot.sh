#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
record_cpu_set="${LEROBOT_RECORD_CPUSET:-6-9}"
record_nice="${LEROBOT_RECORD_NICE:-10}"
encoder_cpu_set="${LEROBOT_ENCODER_CPUSET:-10-13}"

if [[ ! "${record_nice}" =~ ^([0-9]|1[0-9])$ ]]; then
    echo "ERROR: LEROBOT_RECORD_NICE must be an integer from 0 to 19" >&2
    exit 2
fi

if [[ "${LEROBOT_RECORD_SCHEDULING_APPLIED:-false}" != "true" ]]; then
    if ! command -v taskset >/dev/null 2>&1; then
        echo "ERROR: taskset is required for recorder CPU isolation" >&2
        exit 1
    fi
    exec env LEROBOT_RECORD_SCHEDULING_APPLIED=true \
        LEROBOT_RECORD_CPUSET="${record_cpu_set}" \
        LEROBOT_RECORD_NICE="${record_nice}" \
        taskset -c "${record_cpu_set}" "$0" "$@"
fi

current_nice="$(ps -o ni= -p "$$" | tr -d '[:space:]')"
if (( current_nice > record_nice )); then
    if ! command -v pkexec >/dev/null 2>&1; then
        echo "ERROR: raising recorder priority from nice ${current_nice} to ${record_nice} requires pkexec" >&2
        exit 1
    fi
    echo "Recorder priority requires administrator authentication: nice ${current_nice} -> ${record_nice}"
    pkexec /usr/bin/renice -n "${record_nice}" -p "$$"
elif (( current_nice < record_nice )); then
    /usr/bin/renice -n "${record_nice}" -p "$$" >/dev/null
fi

printf 'runtime_scheduling=record cpu_set=%s nice=%s\n' \
    "$(taskset -cp "$$" | sed 's/.*: //')" \
    "$(ps -o ni= -p "$$" | tr -d '[:space:]')"

baseline_file="${project_root}/src/quest_a0509_teleop/config/a0509_metaquest_accepted_2026-08-07.yaml"
baseline_sha256="856c62c933c12258f396d4d7777583d0df3cc40b977393a987e1d8e03ac23efa"

printf '%s  %s\n' "${baseline_sha256}" "${baseline_file}" | sha256sum -c -

source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
source /home/rvlab/venvs/lerobot/bin/activate

set -u

# The main recorder validates and owns all cameras. Opening them in preflight and
# immediately reopening them can leave UVC devices waiting for their next frame.
preflight_camera_check="${PREFLIGHT_CAMERA_CHECK:-false}"
connect_timeout_sec="${CONNECT_TIMEOUT_SEC:-10.0}"
episode_reset_orchestration="${EPISODE_RESET_ORCHESTRATION:-true}"
episode_initial_gripper_state="${EPISODE_INITIAL_GRIPPER_STATE:-open}"
gripper_initialize_timeout_sec="${GRIPPER_INITIALIZE_TIMEOUT_SEC:-10.0}"
preflight_args=()
if [[ "${preflight_camera_check}" == "false" ]]; then
    preflight_args+=(--skip-cameras)
elif [[ "${preflight_camera_check}" != "true" ]]; then
    echo "ERROR: PREFLIGHT_CAMERA_CHECK must be true or false" >&2
    exit 2
fi
if [[ "${episode_reset_orchestration}" != "true" && "${episode_reset_orchestration}" != "false" ]]; then
    echo "ERROR: EPISODE_RESET_ORCHESTRATION must be true or false" >&2
    exit 2
fi
if [[ "${episode_initial_gripper_state}" != "open" && "${episode_initial_gripper_state}" != "close" && "${episode_initial_gripper_state}" != "none" ]]; then
    echo "ERROR: EPISODE_INITIAL_GRIPPER_STATE must be open, close, or none" >&2
    exit 2
fi
if [[ "${episode_reset_orchestration}" == "true" ]]; then
    preflight_args+=(--allow-unprepared-teleop)
fi
if [[ "${episode_initial_gripper_state}" != "none" ]]; then
    preflight_args+=(
        --initial-gripper-state "${episode_initial_gripper_state}"
        --gripper-initialize-timeout-sec "${gripper_initialize_timeout_sec}"
    )
fi
preflight_args+=(--connect-timeout-sec "${connect_timeout_sec}")
python "${project_root}/scripts/preflight_lerobot_shadow_record.py" "${preflight_args[@]}"

recording_tag="$(date +%Y%m%d_%H%M%S)"
dataset_repo_id="${DATASET_REPO_ID:-local/a0509_metaquest_pick_place_pilot}"
dataset_root="${DATASET_ROOT:-${HOME}/lerobot_datasets/a0509_metaquest_pick_place_pilot_${recording_tag}}"
task_description="${TASK_DESCRIPTION:-Pick up the object and place it in the target area.}"
episode_time_s="${EPISODE_TIME_S:-30}"
reset_time_s="${RESET_TIME_S:-15}"
num_episodes="${NUM_EPISODES:-1}"
image_writer_processes="${IMAGE_WRITER_PROCESSES:-0}"
image_writer_threads_per_camera="${IMAGE_WRITER_THREADS_PER_CAMERA:-0}"
streaming_encoding="${STREAMING_ENCODING:-true}"
multiprocess_encoder="${MULTIPROCESS_ENCODER:-true}"
absolute_deadline_scheduler="${ABSOLUTE_DEADLINE_SCHEDULER:-true}"
encoder_shm_slots="${ENCODER_SHM_SLOTS:-32}"
rgb_encoder_vcodec="${RGB_ENCODER_VCODEC:-h264_nvenc}"
rgb_encoder_preset="${RGB_ENCODER_PRESET:-12}"
rgb_encoder_quality="${RGB_ENCODER_QUALITY:-30}"
teleop_max_age_sec="${TELEOP_MAX_AGE_SEC:-1.0}"
episode_reset_before_first="${EPISODE_RESET_BEFORE_FIRST:-true}"

if [[ "${streaming_encoding}" != "true" && "${streaming_encoding}" != "false" ]]; then
    echo "ERROR: STREAMING_ENCODING must be true or false" >&2
    exit 2
fi
if [[ "${multiprocess_encoder}" != "true" && "${multiprocess_encoder}" != "false" ]]; then
    echo "ERROR: MULTIPROCESS_ENCODER must be true or false" >&2
    exit 2
fi
if [[ "${absolute_deadline_scheduler}" != "true" && "${absolute_deadline_scheduler}" != "false" ]]; then
    echo "ERROR: ABSOLUTE_DEADLINE_SCHEDULER must be true or false" >&2
    exit 2
fi
if [[ "${episode_reset_before_first}" != "true" && "${episode_reset_before_first}" != "false" ]]; then
    echo "ERROR: EPISODE_RESET_BEFORE_FIRST must be true or false" >&2
    exit 2
fi
teleop_require_fresh_action_on_connect=true
teleop_defer_calibration_on_connect=false
if [[ "${episode_reset_orchestration}" == "true" ]]; then
    teleop_require_fresh_action_on_connect=false
    teleop_defer_calibration_on_connect=true
fi
if [[ ! "${encoder_shm_slots}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: ENCODER_SHM_SLOTS must be a positive integer" >&2
    exit 2
fi
if ! taskset -c "${encoder_cpu_set}" /usr/bin/true; then
    echo "ERROR: invalid or unavailable LEROBOT_ENCODER_CPUSET: ${encoder_cpu_set}" >&2
    exit 2
fi

if [[ -e "${dataset_root}" ]]; then
    echo "ERROR: dataset root already exists: ${dataset_root}" >&2
    echo "Set DATASET_ROOT to a new path; pilot recording never overwrites data." >&2
    exit 1
fi
mkdir -p "$(dirname "${dataset_root}")"

printf 'dataset_repo_id=%s\n' "${dataset_repo_id}"
printf 'dataset_root=%s\n' "${dataset_root}"
printf 'task_description=%s\n' "${task_description}"
printf 'num_episodes=%s episode_time_s=%s reset_time_s=%s\n' \
    "${num_episodes}" "${episode_time_s}" "${reset_time_s}"
printf 'preflight_camera_check=%s image_writer_processes=%s image_writer_threads_per_camera=%s\n' \
    "${preflight_camera_check}" "${image_writer_processes}" \
    "${image_writer_threads_per_camera}"
printf 'streaming_encoding=%s rgb_encoder=%s preset=%s quality=%s\n' \
    "${streaming_encoding}" "${rgb_encoder_vcodec}" \
    "${rgb_encoder_preset}" "${rgb_encoder_quality}"
printf 'multiprocess_encoder=%s encoder_cpu_set=%s encoder_shm_slots=%s\n' \
    "${multiprocess_encoder}" "${encoder_cpu_set}" "${encoder_shm_slots}"
printf 'absolute_deadline_scheduler=%s\n' "${absolute_deadline_scheduler}"
printf 'episode_reset_orchestration=%s reset_before_first=%s initial_gripper_state=%s\n' \
    "${episode_reset_orchestration}" "${episode_reset_before_first}" \
    "${episode_initial_gripper_state}"
if [[ "${episode_reset_orchestration}" == "true" ]]; then
    printf 'reset_policy=record-end n/r review -> commit/clear -> operator-confirmed prepare -> object reset -> Quest recenter -> gripper init; reset_time_s is ignored\n'
fi
printf 'teleop_max_age_sec=%s\n' "${teleop_max_age_sec}"
printf 'teleop_require_fresh_action_on_connect=%s\n' "${teleop_require_fresh_action_on_connect}"
printf 'teleop_defer_calibration_on_connect=%s\n' "${teleop_defer_calibration_on_connect}"
printf 'connect_timeout_sec=%s\n' "${connect_timeout_sec}"

exec env \
    LEROBOT_A0509_MULTIPROCESS_ENCODER="${multiprocess_encoder}" \
    LEROBOT_A0509_ABSOLUTE_DEADLINE="${absolute_deadline_scheduler}" \
    LEROBOT_A0509_EPISODE_RESET="${episode_reset_orchestration}" \
    EPISODE_RESET_BEFORE_FIRST="${episode_reset_before_first}" \
    EPISODE_INITIAL_GRIPPER_STATE="${episode_initial_gripper_state}" \
    LEROBOT_ENCODER_CPUSET="${encoder_cpu_set}" \
    LEROBOT_ENCODER_SHM_SLOTS="${encoder_shm_slots}" \
    python -m lerobot_robot_doosan_a0509.record_entrypoint \
    --robot.type=doosan_a0509_ros \
    --robot.id=a0509_shadow_teacher \
    --robot.mode=shadow_record \
    --robot.require_camera=true \
    --robot.require_fresh_state_on_connect=true \
    --robot.state_max_age_sec=0.5 \
    --robot.connect_timeout_sec="${connect_timeout_sec}" \
    --teleop.type=metaquest_a0509 \
    --teleop.id=metaquest_right_teacher \
    --teleop.require_calibration=true \
    --teleop.require_fresh_action_on_connect="${teleop_require_fresh_action_on_connect}" \
    --teleop.defer_calibration_on_connect="${teleop_defer_calibration_on_connect}" \
    --teleop.max_age_sec="${teleop_max_age_sec}" \
    --teleop.connect_timeout_sec="${connect_timeout_sec}" \
    --teleop.allow_latched_state_in_shadow_record=true \
    --dataset.repo_id="${dataset_repo_id}" \
    --dataset.root="${dataset_root}" \
    --dataset.single_task="${task_description}" \
    --dataset.fps=30 \
    --dataset.episode_time_s="${episode_time_s}" \
    --dataset.reset_time_s="${reset_time_s}" \
    --dataset.num_episodes="${num_episodes}" \
    --dataset.video=true \
    --dataset.push_to_hub=false \
    --dataset.streaming_encoding="${streaming_encoding}" \
    --dataset.rgb_encoder.vcodec="${rgb_encoder_vcodec}" \
    --dataset.rgb_encoder.preset="${rgb_encoder_preset}" \
    --dataset.rgb_encoder.crf="${rgb_encoder_quality}" \
    --dataset.num_image_writer_processes="${image_writer_processes}" \
    --dataset.num_image_writer_threads_per_camera="${image_writer_threads_per_camera}" \
    --display_data=false \
    --play_sounds=false
