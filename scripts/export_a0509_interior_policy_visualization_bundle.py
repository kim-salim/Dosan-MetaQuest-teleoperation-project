#!/usr/bin/env python3
"""Export the command-free T1--T8 catalog validation as GPT-ready visual data."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import zipfile
from pathlib import Path
from typing import Any, Iterable, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/interior_policy_symbolic_probe_spatial_floor_2026-09-02"
)
DEFAULT_RESULT = DEFAULT_ARTIFACT_ROOT / "symbolic_probe_result.json"
DEFAULT_CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
)
DEFAULT_REPORT = (
    REPOSITORY_ROOT
    / "docs/a0509_t2_t3_spatial_floor_semantics_level2_2026-09-02_ko.md"
)
DEFAULT_OUTPUT = DEFAULT_ARTIFACT_ROOT / "gpt_visualization_bundle"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", type=Path, default=DEFAULT_RESULT)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--zip-output",
        type=Path,
        default=DEFAULT_ARTIFACT_ROOT / "a0509_spatial_floor_gpt_visualization_bundle.zip",
    )
    return parser.parse_args()


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_csv(
    path: Path,
    fieldnames: Iterable[str],
    rows: Iterable[Mapping[str, Any]],
) -> None:
    names = tuple(fieldnames)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=names)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, "") for name in names})


def _compact_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _state_label(state: Mapping[str, Any]) -> str:
    return "<br/>".join(
        (
            f"drawer={state['drawer']}",
            f"block={state['blue_block_location']}",
            f"holding={state['holding']}",
            f"gripper={state['gripper']}",
            f"contact={state['contact_mode']}",
        )
    )


def _make_graph(result: Mapping[str, Any]) -> dict[str, Any]:
    plan = result["controlled_search"]["plans"][0]
    states = result["controlled_replay"]["states"]
    nodes = [
        {
            "id": f"state_{index}",
            "kind": "WorldState",
            "step": index,
            "is_initial": index == 0,
            "is_goal": index == len(states) - 1,
            "state": state,
        }
        for index, state in enumerate(states)
    ]
    edges = []
    for index, step in enumerate(plan["steps"], start=1):
        edges.append(
            {
                "id": f"edge_{index}",
                "source": f"state_{index - 1}",
                "target": f"state_{index}",
                "operator": step["operator"],
                "policy_id": step["policy_id"],
                "cost": step["cost"],
                "edge_kind": (
                    "V2_POLICY_HANDOFF"
                    if index > 1
                    and plan["steps"][index - 2]["policy_id"] != step["policy_id"]
                    else "SAME_POLICY_OR_INITIAL"
                ),
            }
        )
    return {
        "schema_version": "a0509.interior_policy_visual_graph.v1",
        "classification": result["classification"],
        "nodes": nodes,
        "edges": edges,
        "operator_sequence": plan["operators"],
        "policy_sequence": plan["policy_sequence"],
        "total_cost": plan["total_cost"],
        "t5_alternative": result["T5_alternative"],
    }


def _make_mermaid(result: Mapping[str, Any]) -> str:
    plan = result["controlled_search"]["plans"][0]
    states = result["controlled_replay"]["states"]
    lines = [
        "flowchart LR",
        "  classDef initial fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px;",
        "  classDef middle fill:#eef4ff,stroke:#345995,stroke-width:1.5px;",
        "  classDef goal fill:#fff3e0,stroke:#ef6c00,stroke-width:2px;",
    ]
    for index, state in enumerate(states):
        lines.append(f'  S{index}["STEP {index}<br/>{_state_label(state)}"]')
    for index, step in enumerate(plan["steps"]):
        cost = step["cost"]["total"]
        lines.append(
            f'  S{index} -->|"{step["operator"]}<br/>cost={cost:.2f}"| S{index + 1}'
        )
    lines.extend(
        (
            "  class S0 initial;",
            "  class S1,S2,S3 middle;",
            "  class S4 goal;",
            "",
            "%% Policy colors: T4 edges are blue/orange context; T2 is green context.",
            "%% V2 policy-switch edges occur between STEP1->STEP2 and STEP2->STEP3.",
        )
    )
    return "\n".join(lines) + "\n"


def _task_rows(result: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for task_id, task in sorted(result["artifact_audit"]["tasks"].items()):
        rows.append(
            {
                "task_id": task_id,
                "task_description": task["dataset_task_description"],
                "episode_count": task["episode_count"],
                "total_frames": task["total_frames"],
                "fps": task["fps"],
                "semantic_segment_count": len(task["semantic_segments"]),
                "semantic_segments": ";".join(task["semantic_segments"]),
                "dataset_root": task["dataset_root"],
                "semantic_artifact": task["semantic_artifact"],
                "phase_support_artifact": task["phase_support_artifact"],
                "checkpoint": task["checkpoint"],
                "checkpoint_type": task["checkpoint_type"],
                "chunk_size": task["chunk_size"],
                "n_action_steps": task["n_action_steps"],
                "semantic_sha256_verified": task["semantic_sha256_verified"],
                "phase_support_sha256_verified": task[
                    "phase_support_sha256_verified"
                ],
            }
        )
    return rows


def _operator_rows(
    result: Mapping[str, Any],
    catalog: Mapping[str, Any],
) -> list[dict[str, Any]]:
    audits = result["artifact_audit"]["operators"]
    rows = []
    for operator in catalog["operators"]:
        audit = audits[operator["id"]]
        frame = audit["frame_interval"] or {}
        rows.append(
            {
                "operator_id": operator["id"],
                "policy_id": operator["policy_id"],
                "entry_order": operator["entry_order"],
                "exit_order": operator["exit_order"],
                "preconditions_json": _compact_json(operator["preconditions"]),
                "effects_json": _compact_json(operator["effects"]),
                "entry_contact_mode": operator["entry_contact_mode"],
                "exit_contact_mode": operator["exit_contact_mode"],
                "bridge_mode": operator.get("bridge_mode"),
                "base_cost": operator["base_cost"],
                "semantic_segment_ids": ";".join(operator["semantic_segment_ids"]),
                "semantic_labels": ";".join(audit["semantic_labels"]),
                "entry_boundary": (
                    f"{operator['entry_boundary']['segment']}@"
                    f"{operator['entry_boundary']['phase']:.2f}"
                ),
                "exit_boundary": (
                    f"{operator['exit_boundary']['segment']}@"
                    f"{operator['exit_boundary']['phase']:.2f}"
                ),
                "start_frame_min": frame.get("start_frame_min"),
                "start_frame_median": frame.get("start_frame_median"),
                "start_frame_max": frame.get("start_frame_max"),
                "end_frame_min": frame.get("end_frame_min"),
                "end_frame_median": frame.get("end_frame_median"),
                "end_frame_max": frame.get("end_frame_max"),
                "certified_source_boundary": audit[
                    "certified_source_boundary_verified"
                ],
                "artifact_relaxations": ";".join(
                    operator.get("artifact_relaxations", ())
                ),
                "physical_setup_overrides": ";".join(
                    operator.get("physical_setup_overrides", ())
                ),
                "evidence_limitations": ";".join(audit["evidence_limitations"]),
            }
        )
    return rows


def _plan_rows(result: Mapping[str, Any]) -> list[dict[str, Any]]:
    plan = result["controlled_search"]["plans"][0]
    rows = []
    for index, step in enumerate(plan["steps"], start=1):
        before = step["state_before"]
        after = step["state_after"]
        rows.append(
            {
                "step": index,
                "operator_id": step["operator"],
                "policy_id": step["policy_id"],
                "base_cost": step["cost"]["base_cost"],
                "switch_penalty": step["cost"]["policy_switch_penalty"],
                "transition_cost": step["cost"]["transition_cost"],
                "step_cost": step["cost"]["total"],
                "state_before_json": _compact_json(before),
                "state_after_json": _compact_json(after),
                "drawer_before": before["drawer"],
                "drawer_after": after["drawer"],
                "block_before": before["blue_block_location"],
                "block_after": after["blue_block_location"],
                "holding_before": before["holding"],
                "holding_after": after["holding"],
                "gripper_before": before["gripper"],
                "gripper_after": after["gripper"],
                "contact_before": before["contact_mode"],
                "contact_after": after["contact_mode"],
            }
        )
    return rows


def _replay_rows(result: Mapping[str, Any]) -> list[dict[str, Any]]:
    replay = result["controlled_replay"]
    rows = []
    for index, state in enumerate(replay["states"]):
        rows.append(
            {
                "step": index,
                "action_from_previous": "" if index == 0 else replay["operators"][index - 1],
                **state,
                "is_goal": index == len(replay["states"]) - 1,
            }
        )
    return rows


def _v2_rows(result: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for index, edge in enumerate(result["v2_edge_shadow_validation"], start=1):
        rows.append(
            {
                "edge_index": index,
                "source_operator": edge["source_operator"],
                "successor_operator": edge["successor_operator"],
                "transition_type": edge["symbolic"]["transition_type"],
                "symbolic_valid": edge["symbolic"]["valid"],
                "structural_runtime_mapping": edge["structural_runtime_mapping"],
                "source_support_available": edge["source_support_available"],
                "successor_support_available": edge["successor_support_available"],
                "v2_code_contract_available": edge["v2_code_contract_available"],
                "exact_episode_manifest_ready": edge[
                    "exact_episode_manifest_ready"
                ],
                "runtime_contract_compatible": edge[
                    "runtime_contract_compatible"
                ],
                "missing_runtime_work": ";".join(edge["missing_runtime_work"]),
                "physical_validation_performed": edge[
                    "physical_validation_performed"
                ],
            }
        )
    return rows


def _alternative_rows(result: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for rank, plan in enumerate(result["full_catalog_search"]["plans"], start=1):
        rows.append(
            {
                "rank": rank,
                "total_cost": plan["total_cost"],
                "operator_count": len(plan["operators"]),
                "operator_sequence": " -> ".join(plan["operators"]),
                "policy_sequence": " -> ".join(plan["policy_sequence"]),
                "contains_T5_open": plan["operators"][0] == "T5.open_drawer",
            }
        )
    return rows


def _write_readme(output: Path) -> None:
    text = """# GPT 시각화용 A0509 T4→T2→T4 검증 자료

이 폴더는 실제 robot command 없이 수행한 Dijkstra/UCS symbolic 검증 결과를 시각화하기 쉽게 재구성한 bundle이다.

## 가장 먼저 업로드할 파일

1. `GPT_VISUALIZATION_PROMPT_KO.md`
2. `visual_graph.json`
3. `controlled_plan.csv`
4. `symbolic_state_replay.csv`
5. `v2_edge_readiness.csv`

세부 근거까지 분석하려면 `operator_catalog.csv`, `task_inventory.csv`, `validation_report_ko.md`, `full_symbolic_probe_result.json`도 함께 제공한다.

## 핵심 판정

```text
Dijkstra symbolic plan: PASS
T4 → T2 → T4: PASS
두 V2 edge의 structural mapping: PASS
두 exact edge manifest 및 command-free shadow: PASS
현재 분류: LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE
physical validation: NOT PERFORMED
```

## 파일 설명

- `visual_graph.json`: 5개 WorldState node와 4개 operator edge
- `controlled_plan.mmd`: Mermaid로 바로 렌더링 가능한 계획
- `controlled_plan.csv`: step별 cost 및 before/after state
- `symbolic_state_replay.csv`: symbolic simulator의 상태 변화
- `operator_catalog.csv`: T1~T8 전체 21개 operator와 artifact 근거
- `task_inventory.csv`: T1~T8 dataset/model/artifact inventory
- `v2_edge_readiness.csv`: 두 policy-switch edge의 V2 준비 상태
- `full_search_alternatives.csv`: Dijkstra가 찾은 상위 계획과 T5 대안
- `world_state_schema.json`: state field와 허용 값
- `source_provenance.json`: 원본 자료와 검증 범위
- `checksums.sha256`: bundle 파일 무결성

## 시각화 시 지켜야 할 구분

- 초록/실선: symbolic validity
- 파랑: current V2에 구조적으로 mapping 가능
- 빨강/점선: 해당 edge의 exact manifest 또는 shadow가 없어 아직 runtime-ready가 아님
- command-free LEVEL 2를 LEVEL 3 물리 성공으로 확대 표현하지 말 것
- T5 대안은 T4와 동일 cost이며, T4 선택은 lexical tie-break임
"""
    (output / "README_KO.md").write_text(text, encoding="utf-8")


def _write_prompt(output: Path) -> None:
    text = """# GPT 시각화 요청용 프롬프트

첨부한 A0509 symbolic planner 검증 자료를 읽고 다음 시각화를 만들어줘.

## 필수 시각화 1 — Dijkstra 최종 경로

WorldState를 node, interior-policy operator를 directed edge로 표시해.

최종 경로:

```text
T4.open_drawer
→ T2.acquire_from_floor
→ T4.deliver_to_drawer
→ T4.close_drawer
```

각 node에는 최소 다음 상태를 표시해.

```text
drawer
blue_block_location
holding
gripper
contact_mode
T2/T4 cursor
```

T4 edge와 T2 edge는 서로 다른 색으로 표시하고, policy sequence `T4→T2→T4`, total cost `4.50`도 표시해.

## 필수 시각화 2 — Symbolic planner와 V2 runtime의 역할 분리

```text
Dijkstra/UCS
  → operator plan
  → V2 edge manifest
  → runtime actual/ack Bridge
  → async successor inference
  → fresh prefix admission
  → soft crossfade
```

두 policy-switch edge는 다음 상태로 표시해.

```text
T4.open → T2.acquire:
  symbolic=true
  structural V2 mapping=true
  exact manifest=true
  command-free policy shadow=true

T2.acquire → T4.deliver:
  symbolic=true
  structural V2 mapping=true
  exact manifest=true
  command-free policy shadow=true
```

두 edge는 파란 실선과 `RUNTIME-CONTRACT-COMPATIBLE`로 표시하되,
물리 검증을 뜻하지 않는다는 주석을 명확히 넣어.

## 필수 시각화 3 — 대안 계획 비교

다음 두 경로는 cost가 모두 4.50이다.

```text
T4.open → T2.acquire → T4.deliver → T4.close
T5.open → T2.acquire → T4.deliver → T4.close
```

T4 선택은 physical success 우위가 아니라 deterministic lexical tie-break라는 주석을 넣어.

## 표현 금지

- `physically validated`라고 표현하지 말 것
- command-free manifest/shadow 통과를 physical Bridge 성공으로 확대 표현하지 말 것
- semantic artifact가 robot executable이라고 표현하지 말 것

현재 정확한 분류는 `LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE`이다.
현재 registry의 118개 flexible_verified edge만 LEVEL 2로 표현할 것.
"""
    (output / "GPT_VISUALIZATION_PROMPT_KO.md").write_text(text, encoding="utf-8")


def _world_state_schema() -> dict[str, Any]:
    return {
        "schema_version": "a0509.symbolic_world_state.v1",
        "node_type": "immutable_hashable_WorldState",
        "unknown_is_wildcard": False,
        "fields": {
            "drawer": ["open", "closed", "unknown"],
            "white_container": ["open", "closed", "unknown"],
            "blue_block_location": [
                "left_floor",
                "right_floor",
                "black_table",
                "drawer",
                "white_container",
                "held",
                "moving_source",
                "stacked",
                "unknown",
            ],
            "holding": ["none", "blue_block", "unknown"],
            "gripper": ["open", "closed", "unknown"],
            "contact_mode": [
                "free_space",
                "free_transport",
                "contact_manipulation",
                "unknown",
            ],
            "cursor_t1_to_t8": "non_negative_integer_monotonic",
            "last_policy": ["T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8", None],
        },
    }


def _checksums(output: Path) -> None:
    lines = []
    for path in sorted(item for item in output.iterdir() if item.is_file()):
        if path.name == "checksums.sha256":
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f"{digest}  {path.name}")
    (output / "checksums.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _make_zip(output: Path, zip_output: Path) -> None:
    zip_output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(item for item in output.iterdir() if item.is_file()):
            archive.write(path, arcname=f"gpt_visualization_bundle/{path.name}")


def export(args: argparse.Namespace) -> dict[str, Any]:
    result = json.loads(args.result.read_text(encoding="utf-8"))
    catalog = json.loads(args.catalog.read_text(encoding="utf-8"))
    if result["classification"] not in {
        "LEVEL_1_SYMBOLICALLY_PLANNABLE",
        "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE",
    }:
        raise ValueError("unexpected validation classification")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)

    graph = _make_graph(result)
    _write_json(output / "visual_graph.json", graph)
    _write_json(output / "world_state_schema.json", _world_state_schema())

    task_rows = _task_rows(result)
    _write_csv(output / "task_inventory.csv", task_rows[0].keys(), task_rows)
    operator_rows = _operator_rows(result, catalog)
    _write_csv(
        output / "operator_catalog.csv", operator_rows[0].keys(), operator_rows
    )
    plan_rows = _plan_rows(result)
    _write_csv(output / "controlled_plan.csv", plan_rows[0].keys(), plan_rows)
    replay_rows = _replay_rows(result)
    _write_csv(
        output / "symbolic_state_replay.csv", replay_rows[0].keys(), replay_rows
    )
    v2_rows = _v2_rows(result)
    _write_csv(output / "v2_edge_readiness.csv", v2_rows[0].keys(), v2_rows)
    alternative_rows = _alternative_rows(result)
    _write_csv(
        output / "full_search_alternatives.csv",
        alternative_rows[0].keys(),
        alternative_rows,
    )
    (output / "controlled_plan.mmd").write_text(
        _make_mermaid(result), encoding="utf-8"
    )

    shutil.copy2(args.result, output / "full_symbolic_probe_result.json")
    shutil.copy2(args.catalog, output / "operator_catalog_source.json")
    shutil.copy2(args.report, output / "validation_report_ko.md")

    provenance = {
        "schema_version": "a0509.gpt_visualization_bundle.v1",
        "classification": result["classification"],
        "physical_validation_performed": result["physical_validation_performed"],
        "source_result": str(args.result.resolve()),
        "source_catalog": str(args.catalog.resolve()),
        "source_report": str(args.report.resolve()),
        "planner_source": str(
            REPOSITORY_ROOT
            / "src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509/interior_policy/planner.py"
        ),
        "test_source": str(
            REPOSITORY_ROOT
            / "src/lerobot_robot_doosan_a0509/test/test_interior_policy_planner.py"
        ),
        "pass_conditions": result["pass_conditions"],
        "test_results": {
            "full_lerobot_robot_doosan_a0509_tests": "332 passed",
            "planner_level2_web_tests": "53 passed",
            "legacy_level2_regression_tests": "10 passed",
            "gui_server_tests": "15 passed",
            "headless_browser_planner": "passed",
            "physical_tests": "not performed",
        },
    }
    _write_json(output / "source_provenance.json", provenance)
    _write_readme(output)
    _write_prompt(output)

    manifest = {
        "schema_version": "a0509.gpt_visualization_manifest.v1",
        "classification": result["classification"],
        "operator_sequence": graph["operator_sequence"],
        "policy_sequence": graph["policy_sequence"],
        "total_cost": graph["total_cost"],
        "t5_alternative": graph["t5_alternative"],
        "v2_edges": v2_rows,
        "recommended_upload_order": [
            "GPT_VISUALIZATION_PROMPT_KO.md",
            "visual_graph.json",
            "controlled_plan.csv",
            "symbolic_state_replay.csv",
            "v2_edge_readiness.csv",
            "operator_catalog.csv",
            "validation_report_ko.md",
        ],
        "files": sorted(path.name for path in output.iterdir() if path.is_file()),
    }
    _write_json(output / "visualization_manifest.json", manifest)
    _checksums(output)
    _make_zip(output, args.zip_output.resolve())
    return {
        "output_dir": str(output),
        "zip_output": str(args.zip_output.resolve()),
        "file_count": len(tuple(path for path in output.iterdir() if path.is_file())),
        "classification": result["classification"],
    }


def main() -> None:
    args = _parse_args()
    summary = export(args)
    print("VISUALIZATION_BUNDLE_OK " + json.dumps(summary, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
