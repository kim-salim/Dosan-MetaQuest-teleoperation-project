#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-/dsr01/io/set_tool_digital_output}"
SERVICE_TYPE="${SERVICE_TYPE:-dsr_msgs2/srv/SetToolDigitalOutput}"
CLOSE_INDEX="${CLOSE_INDEX:-1}"
OPEN_INDEX="${OPEN_INDEX:-2}"
OFF_VALUE=0

check_service() {
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 command not found. Source ROS2 and workspace setup first." >&2
    exit 1
  fi

  local actual_type
  if ! actual_type="$(ros2 service type "${SERVICE_NAME}" 2>/dev/null)"; then
    echo "ERROR: service is unavailable: ${SERVICE_NAME}" >&2
    exit 1
  fi
  if [[ "${actual_type}" != "${SERVICE_TYPE}" ]]; then
    echo "ERROR: service type mismatch for ${SERVICE_NAME}: ${actual_type} != ${SERVICE_TYPE}" >&2
    exit 1
  fi
}

set_do() {
  local index="$1"
  local value="$2"
  ros2 service call "${SERVICE_NAME}" "${SERVICE_TYPE}" "{index: ${index}, value: ${value}}" >/dev/null
}

check_service
set_do "${CLOSE_INDEX}" "${OFF_VALUE}"
set_do "${OPEN_INDEX}" "${OFF_VALUE}"
echo "gripper all off: close=${CLOSE_INDEX}, open=${OPEN_INDEX}, value=${OFF_VALUE}"
