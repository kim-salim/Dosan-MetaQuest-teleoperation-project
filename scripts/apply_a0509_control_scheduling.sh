#!/usr/bin/env bash
set -euo pipefail

control_cpu_set="${A0509_CONTROL_CPUSET:-0-5}"
control_nice="${A0509_CONTROL_NICE:-0}"
launch_pid="${1:-}"

if [[ ! "${control_nice}" =~ ^([0-9]|1[0-9])$ ]]; then
    echo "ERROR: A0509_CONTROL_NICE must be an integer from 0 to 19" >&2
    exit 2
fi

if [[ -z "${launch_pid}" ]]; then
    mapfile -t candidates < <(
        pgrep -f 'ros2 launch quest_a0509_teleop a0509_full_bringup_with_gripper.launch.py' || true
    )
    if (( ${#candidates[@]} != 1 )); then
        echo "ERROR: expected exactly one A0509 full bringup, found ${#candidates[@]}" >&2
        echo "Pass the ros2 launch PID explicitly: $0 <launch_pid>" >&2
        exit 1
    fi
    launch_pid="${candidates[0]}"
fi

if [[ ! "${launch_pid}" =~ ^[0-9]+$ ]] || [[ ! -r "/proc/${launch_pid}/cmdline" ]]; then
    echo "ERROR: invalid or missing launch PID: ${launch_pid}" >&2
    exit 2
fi

launch_command="$(tr '\0' ' ' < "/proc/${launch_pid}/cmdline")"
if [[ "${launch_command}" != *"ros2 launch quest_a0509_teleop a0509_full_bringup_with_gripper.launch.py"* ]]; then
    echo "ERROR: PID ${launch_pid} is not the expected A0509 full bringup" >&2
    echo "command=${launch_command}" >&2
    exit 2
fi

script_path="$(readlink -f "$0")"
if (( EUID != 0 )); then
    if ! command -v pkexec >/dev/null 2>&1; then
        echo "ERROR: pkexec is required to raise control priority" >&2
        exit 1
    fi
    echo "Control scheduling requires administrator authentication."
    exec pkexec env \
        A0509_CONTROL_CPUSET="${control_cpu_set}" \
        A0509_CONTROL_NICE="${control_nice}" \
        "${script_path}" "${launch_pid}"
fi

collect_process_tree() {
    local root_pid="$1"
    local -a queue=("${root_pid}")
    local index=0
    local pid child

    while (( index < ${#queue[@]} )); do
        pid="${queue[index]}"
        index=$((index + 1))
        [[ -d "/proc/${pid}" ]] || continue
        printf '%s\n' "${pid}"
        while read -r child; do
            [[ -n "${child}" ]] && queue+=("${child}")
        done < <(pgrep -P "${pid}" || true)
    done
}

mapfile -t process_ids < <(collect_process_tree "${launch_pid}" | sort -nu)
if (( ${#process_ids[@]} == 0 )); then
    echo "ERROR: no live processes found under launch PID ${launch_pid}" >&2
    exit 1
fi

thread_count=0
for pid in "${process_ids[@]}"; do
    [[ -d "/proc/${pid}/task" ]] || continue
    taskset -apc "${control_cpu_set}" "${pid}" >/dev/null
    for task_path in "/proc/${pid}/task/"*; do
        [[ -d "${task_path}" ]] || continue
        tid="${task_path##*/}"
        /usr/bin/renice -n "${control_nice}" -p "${tid}" >/dev/null
        thread_count=$((thread_count + 1))
    done
done

bad_nice=0
bad_affinity=0
for pid in "${process_ids[@]}"; do
    [[ -d "/proc/${pid}/task" ]] || continue
    for task_path in "/proc/${pid}/task/"*; do
        [[ -d "${task_path}" ]] || continue
        tid="${task_path##*/}"
        actual_nice="$(ps -o ni= -p "${tid}" | tr -d '[:space:]')"
        actual_cpu_set="$(taskset -cp "${tid}" 2>/dev/null | sed 's/.*: //')"
        [[ "${actual_nice}" == "${control_nice}" ]] || bad_nice=$((bad_nice + 1))
        [[ "${actual_cpu_set}" == "${control_cpu_set}" ]] || bad_affinity=$((bad_affinity + 1))
    done
done

printf 'runtime_scheduling=control launch_pid=%s processes=%s threads=%s cpu_set=%s nice=%s bad_nice=%s bad_affinity=%s\n' \
    "${launch_pid}" "${#process_ids[@]}" "${thread_count}" \
    "${control_cpu_set}" "${control_nice}" "${bad_nice}" "${bad_affinity}"

if (( bad_nice != 0 || bad_affinity != 0 )); then
    exit 1
fi
