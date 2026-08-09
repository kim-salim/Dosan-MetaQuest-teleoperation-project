#!/usr/bin/env bash
set -uo pipefail

failed=0
rtprio="$(ulimit -r)"
memlock="$(ulimit -l)"

echo "User: $(id -un)"
echo "Groups: $(id -nG)"
echo "Max realtime priority: ${rtprio}"
echo "Max locked memory: ${memlock}"

if id -nG | tr ' ' '\n' | grep -Fxq realtime; then
  echo "PASS: the current login session has the realtime group"
else
  echo "FAIL: the current login session does not have the realtime group"
  failed=1
fi

case "${rtprio}" in
  unlimited)
    echo "PASS: realtime priority limit allows priority 50"
    ;;
  ''|*[!0-9]*)
    echo "FAIL: unexpected realtime priority limit: ${rtprio}"
    failed=1
    ;;
  *)
    if (( rtprio >= 50 )); then
      echo "PASS: realtime priority limit allows priority 50"
    else
      echo "FAIL: realtime priority limit must be at least 50"
      failed=1
    fi
    ;;
esac

if [[ "${memlock}" == "unlimited" ]]; then
  echo "PASS: locked-memory limit is unlimited"
else
  echo "FAIL: locked-memory limit is not unlimited"
  failed=1
fi

if ! command -v chrt >/dev/null 2>&1; then
  echo "FAIL: chrt is not installed (expected from util-linux)"
  failed=1
elif chrt --fifo 50 true >/dev/null 2>&1; then
  echo "PASS: SCHED_FIFO priority 50 can be created"
else
  echo "FAIL: SCHED_FIFO priority 50 is not permitted in this login session"
  failed=1
fi

if (( failed != 0 )); then
  echo
  echo "If setup was just applied, completely log out and back in (or reboot), then retry."
  exit 1
fi

echo
echo "Realtime scheduling permissions are ready for ros2_control."
