#!/usr/bin/env bash
set -eo pipefail

required="I_CONFIRM_A0509_WORKSPACE_CLEAR_FOR_PREP"
if [[ "${A0509_PREP_AUTHORIZATION:-}" != "${required}" ]]; then
    echo "ERROR: A0509_PREP_AUTHORIZATION=${required} is required" >&2
    exit 2
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot/bin/activate
source "${project_root}/install/setup.bash"
set -u

echo "WEB_PREP_STATE=FORCING_SAFE"
timeout 5 ros2 service call /vr/set_live_robot_output \
    std_srvs/srv/SetBool '{data: false}'
timeout 5 ros2 service call /control/select_disabled \
    std_srvs/srv/Trigger '{}'

echo "WEB_PREP_STATE=MOVING_TO_PREP_POSE"
timeout 120 ros2 service call /vr/prepare_robot \
    std_srvs/srv/Trigger '{}'

echo "WEB_PREP_STATE=VERIFYING_INITIAL_GRIPPER_OPEN"
python -c 'from lerobot_robot_doosan_a0509.recording_gripper_initializer import ensure_recording_gripper_state as ensure; print(ensure("open", 15.0))'

echo "WEB_PREP_STATE=COMPLETE"
