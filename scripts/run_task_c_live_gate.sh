#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_stamp="$(date +%Y%m%d_%H%M%S)"
report_json="${REPORT_JSON:-/home/rvlab/lerobot_datasets/task_c_live_gate_${run_stamp}.json}"
execute="${EXECUTE:-0}"
motion_authorization="${MOTION_AUTHORIZATION:-}"

source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot/bin/activate
source "${project_root}/install/setup.bash"
set -u

export PYTHONPATH="${project_root}:${PYTHONPATH:-}"

arguments=(
    --report-json "${report_json}"
)
if [[ "${execute}" == "1" ]]; then
    required="I_ACKNOWLEDGE_TASK_C_REAL_ROBOT_MOTION"
    if [[ "${motion_authorization}" != "${required}" ]]; then
        echo "ERROR: EXECUTE=1 requires MOTION_AUTHORIZATION=${required}" >&2
        exit 1
    fi
    arguments+=(
        --execute
        --motion-authorization "${motion_authorization}"
    )
    echo "Task-C gate mode=AUTHORIZED_FULL_REAL_MOTION"
else
    echo "Task-C gate mode=PREFLIGHT_ONLY; MUX remains DISABLED and Live remains OFF"
fi
echo "gate_report=${report_json}"

exec python "${project_root}/scripts/a0509_task_c_live_trial_gate.py" \
    "${arguments[@]}"
