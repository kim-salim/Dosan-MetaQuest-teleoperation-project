#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot/bin/activate
source "${project_root}/install/setup.bash"
set -u

export PYTHONPATH="${project_root}:${project_root}/src/lerobot_robot_doosan_a0509:${PYTHONPATH:-}"

exec python "${project_root}/scripts/a0509_multi_stage_live_trial_gate.py" "$@"
