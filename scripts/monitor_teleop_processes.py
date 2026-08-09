#!/usr/bin/env python3
"""Collect low-overhead process/thread scheduling samples and write after capture."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import signal
import time


HEADER = (
    "sample",
    "wall_ns",
    "monotonic_ns",
    "kind",
    "label",
    "owner_pid",
    "tid",
    "alive",
    "state",
    "cpu_percent",
    "cpu_ticks",
    "processor",
    "nice",
    "num_threads",
    "rt_priority",
    "policy",
    "voluntary_context_switches",
    "nonvoluntary_context_switches",
    "load1",
    "load5",
    "load15",
)


def parse_process_spec(value: str) -> tuple[str, int]:
    label, separator, pid_text = value.partition("=")
    if not separator or not label:
        raise argparse.ArgumentTypeError("process must use LABEL=PID")
    try:
        pid = int(pid_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("PID must be an integer") from exc
    if pid < 0:
        raise argparse.ArgumentTypeError("PID must be >= 0")
    return label, pid


def read_proc_stat(path: Path) -> dict[str, int | str]:
    text = path.read_text(encoding="utf-8")
    close_paren = text.rfind(")")
    if close_paren < 0:
        raise ValueError(f"invalid proc stat: {path}")
    fields = text[close_paren + 2 :].split()
    return {
        "state": fields[0],
        "cpu_ticks": int(fields[11]) + int(fields[12]),
        "nice": int(fields[16]),
        "num_threads": int(fields[17]),
        "processor": int(fields[36]),
        "rt_priority": int(fields[37]),
        "policy": int(fields[38]),
    }


def read_context_switches(path: Path) -> tuple[int, int]:
    voluntary = 0
    nonvoluntary = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(":")
        if not separator:
            continue
        if key == "voluntary_ctxt_switches":
            voluntary = int(value.strip())
        elif key == "nonvoluntary_ctxt_switches":
            nonvoluntary = int(value.strip())
    return voluntary, nonvoluntary


def iter_entities(
    processes: list[tuple[str, int]],
    thread_owner_pid: int | None,
) -> list[tuple[str, str, int, int, Path]]:
    entities: list[tuple[str, str, int, int, Path]] = []
    for label, pid in processes:
        resolved_pid = os.getpid() if pid == 0 else pid
        entities.append(
            ("process", label, resolved_pid, resolved_pid, Path(f"/proc/{resolved_pid}"))
        )
    if thread_owner_pid is not None:
        task_dir = Path(f"/proc/{thread_owner_pid}/task")
        try:
            tids = sorted(int(path.name) for path in task_dir.iterdir())
        except (FileNotFoundError, ProcessLookupError):
            tids = []
        for tid in tids:
            entities.append(
                (
                    "thread",
                    "streamer_thread",
                    thread_owner_pid,
                    tid,
                    task_dir / str(tid),
                )
            )
    return entities


def write_rows(output: Path, rows: list[tuple[object, ...]]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(HEADER)
        writer.writerows(rows)
    temporary.replace(output)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--interval-sec", type=float, default=0.1)
    parser.add_argument(
        "--process",
        action="append",
        type=parse_process_spec,
        default=[],
        help="Process to monitor as LABEL=PID; PID 0 means this monitor.",
    )
    parser.add_argument("--thread-owner-pid", type=int)
    args = parser.parse_args()
    if args.duration_sec <= 0.0:
        parser.error("--duration-sec must be > 0")
    if args.interval_sec <= 0.0:
        parser.error("--interval-sec must be > 0")

    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rows: list[tuple[object, ...]] = []
    previous: dict[tuple[int, int], tuple[int, int]] = {}
    clock_ticks = os.sysconf("SC_CLK_TCK")
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(args.duration_sec * 1_000_000_000)
    interval_ns = int(args.interval_sec * 1_000_000_000)
    next_sample_ns = start_ns
    sample_index = 0

    while not stop_requested and time.monotonic_ns() < deadline_ns:
        now_mono_ns = time.monotonic_ns()
        if now_mono_ns < next_sample_ns:
            time.sleep((next_sample_ns - now_mono_ns) / 1_000_000_000)
            continue
        now_mono_ns = time.monotonic_ns()
        now_wall_ns = time.time_ns()
        sample_index += 1
        load1, load5, load15 = os.getloadavg()
        entities = iter_entities(args.process, args.thread_owner_pid)
        for kind, label, owner_pid, tid, base_path in entities:
            # The process aggregate and its main thread share the same numeric
            # PID/TID. Keep their baselines separate or the process CPU delta
            # is compared with the main-thread delta and becomes enormous.
            key = (kind, owner_pid, tid)
            try:
                stat = read_proc_stat(base_path / "stat")
                voluntary, nonvoluntary = read_context_switches(base_path / "status")
                cpu_ticks = int(stat["cpu_ticks"])
                previous_sample = previous.get(key)
                cpu_percent = ""
                if previous_sample is not None:
                    previous_ns, previous_ticks = previous_sample
                    elapsed_sec = (now_mono_ns - previous_ns) / 1_000_000_000
                    if elapsed_sec > 0.0:
                        cpu_percent = (
                            (cpu_ticks - previous_ticks)
                            / clock_ticks
                            / elapsed_sec
                            * 100.0
                        )
                previous[key] = (now_mono_ns, cpu_ticks)
                rows.append(
                    (
                        sample_index,
                        now_wall_ns,
                        now_mono_ns,
                        kind,
                        label,
                        owner_pid,
                        tid,
                        1,
                        stat["state"],
                        cpu_percent,
                        cpu_ticks,
                        stat["processor"],
                        stat["nice"],
                        stat["num_threads"],
                        stat["rt_priority"],
                        stat["policy"],
                        voluntary,
                        nonvoluntary,
                        load1,
                        load5,
                        load15,
                    )
                )
            except (FileNotFoundError, ProcessLookupError):
                rows.append(
                    (
                        sample_index,
                        now_wall_ns,
                        now_mono_ns,
                        kind,
                        label,
                        owner_pid,
                        tid,
                        0,
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        load1,
                        load5,
                        load15,
                    )
                )
        next_sample_ns += interval_ns
        if next_sample_ns < now_mono_ns:
            missed = (now_mono_ns - next_sample_ns) // interval_ns + 1
            next_sample_ns += missed * interval_ns

    write_rows(args.output, rows)
    print(f"PROCESS_MONITOR_RESULT path={args.output} rows={len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
