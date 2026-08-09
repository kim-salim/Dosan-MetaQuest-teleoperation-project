#!/usr/bin/env bash
set -euo pipefail

if (( EUID != 0 )); then
  echo "Run this script with sudo:"
  echo "  sudo bash scripts/setup_realtime_permissions.sh"
  exit 1
fi

target_user="${SUDO_USER:-${1:-}}"
if [[ -z "${target_user}" || "${target_user}" == "root" ]]; then
  echo "Could not determine the non-root target user."
  echo "Run with sudo, or pass the username explicitly when already logged in as root."
  exit 1
fi

if ! id "${target_user}" >/dev/null 2>&1; then
  echo "User does not exist: ${target_user}"
  exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
limits_source="${script_dir}/../config/realtime/99-realtime.conf"
limits_target="/etc/security/limits.d/99-realtime.conf"

if [[ ! -f "${limits_source}" ]]; then
  echo "Limits template is missing: ${limits_source}"
  exit 1
fi

if ! getent group realtime >/dev/null; then
  groupadd realtime
  echo "Created group: realtime"
else
  echo "Group already exists: realtime"
fi

usermod -a -G realtime "${target_user}"
install -o root -g root -m 0644 "${limits_source}" "${limits_target}"

echo "Added ${target_user} to the realtime group."
echo "Installed ${limits_target}."
echo
echo "A complete logout/login or reboot is required before the new limits apply."
echo "After logging in again, run:"
echo "  bash scripts/verify_realtime_permissions.sh"
