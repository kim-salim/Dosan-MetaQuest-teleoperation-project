#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export TASK_C_STREAM_RAMP_LINEAR_MM_PER_TICK=7.5
export TASK_C_STREAM_RAMP_ROT_DEG_PER_TICK=1.25

echo "Task-C rollback command profile: a0509_ramp7p5_ack_span2_v1"
exec "${project_root}/scripts/run_task_c_control_bringup.sh" "$@"
