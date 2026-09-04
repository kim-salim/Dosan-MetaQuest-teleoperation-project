#!/usr/bin/env bash
set -o pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/rvlab/venvs/lerobot/bin/activate
source "${project_root}/install/setup.bash"
set -u

status=0
timeout 5 ros2 service call /vr/set_live_robot_output \
    std_srvs/srv/SetBool '{data: false}' || status=1
timeout 5 ros2 service call /control/select_disabled \
    std_srvs/srv/Trigger '{}' || status=1

if [[ "${1:-}" == "--emergency" ]]; then
    timeout 5 ros2 service call /vr/stop_robot \
        std_srvs/srv/Trigger '{}' || status=1
fi

exit "${status}"
