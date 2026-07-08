#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"${SCRIPT_DIR}/gripper_all_off.sh"
"${SCRIPT_DIR}/gripper_open_pulse.sh"
sleep 1
"${SCRIPT_DIR}/gripper_close_pulse.sh"
sleep 1
"${SCRIPT_DIR}/gripper_open_pulse.sh"
"${SCRIPT_DIR}/gripper_all_off.sh"
echo "gripper test sequence complete"
