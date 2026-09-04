#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
control_dry_run="${CONTROL_DRY_RUN:-true}"
motion_authorization="${MOTION_AUTHORIZATION:-}"
config_file="${CONFIG_FILE:-${project_root}/src/quest_a0509_teleop/config/a0509_metaquest_accepted_2026-08-07.yaml}"
stream_ramp_linear_mm_per_tick="${TASK_C_STREAM_RAMP_LINEAR_MM_PER_TICK:-7.5}"
stream_ramp_rot_deg_per_tick="${TASK_C_STREAM_RAMP_ROT_DEG_PER_TICK:-1.25}"

if [[ "${control_dry_run}" != "true" && "${control_dry_run}" != "false" ]]; then
    echo "ERROR: CONTROL_DRY_RUN must be true or false" >&2
    exit 1
fi
if [[ "${control_dry_run}" == "false" ]]; then
    required="I_ACKNOWLEDGE_TASK_C_REAL_ROBOT_MOTION"
    if [[ "${motion_authorization}" != "${required}" ]]; then
        echo "ERROR: real control bringup requires MOTION_AUTHORIZATION=${required}" >&2
        exit 1
    fi
fi

source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u

echo "Task-C control bringup: dry_run=${control_dry_run}"
echo "Task-C streamer profile: linear=${stream_ramp_linear_mm_per_tick} mm/tick rotation=${stream_ramp_rot_deg_per_tick} deg/tick"
echo "MetaQuest endpoint/input nodes=false GUI=false MUX source=DISABLED Live=false"
echo "This script never calls Prepare Robot, selects LEROBOT, or enables Live."

exec ros2 launch quest_a0509_teleop a0509_full_bringup_with_gripper.launch.py \
    config_file:="${config_file}" \
    dry_run:="${control_dry_run}" \
    start_endpoint:=false \
    start_robot_bringup:=true \
    start_teleop:=true \
    start_metaquest_inputs:=false \
    start_gui:=false \
    start_calibration_gui:=false \
    start_gripper:=true \
    use_command_mux:=true \
    initial_control_source:=DISABLED \
    stream_ramp_linear_mm_per_tick:="${stream_ramp_linear_mm_per_tick}" \
    stream_ramp_rot_deg_per_tick:="${stream_ramp_rot_deg_per_tick}" \
    start_rviz:=false
