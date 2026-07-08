#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-/dsr01/io/set_tool_digital_output}"
SERVICE_TYPE="${SERVICE_TYPE:-dsr_msgs2/srv/SetToolDigitalOutput}"
CLOSE_INDEX="${CLOSE_INDEX:-1}"
OPEN_INDEX="${OPEN_INDEX:-2}"
PULSE_SEC="${PULSE_SEC:-0.20}"
OFF_VALUE=0
ON_VALUE=1

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

all_off() {
  set_do "${CLOSE_INDEX}" "${OFF_VALUE}" || true
  set_do "${OPEN_INDEX}" "${OFF_VALUE}" || true
}

check_service
trap all_off EXIT
set_do "${OPEN_INDEX}" "${OFF_VALUE}"
set_do "${CLOSE_INDEX}" "${ON_VALUE}"
sleep "${PULSE_SEC}"
set_do "${CLOSE_INDEX}" "${OFF_VALUE}"
echo "gripper close pulse: close=${CLOSE_INDEX}, pulse=${PULSE_SEC}s"
