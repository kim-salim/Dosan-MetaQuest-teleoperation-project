"""Generate semantic-only V3 graphs for the A0509 T2 through T8 tasks.

The model path is recorded for lineage, but semantics are extracted from the
corresponding LeRobot demonstrations. No representative episode, averaged
frame number, spherical runtime boundary, or transition fitness is produced.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from offline_tools.semantic_segmentation.semantic_phase_core import (
    SemanticSegmentSpec,
    aggregate_standardized_segment,
    detect_gripper_event_sequence,
    resample_cartesian_span,
    resolve_anchor_index,
)
from offline_tools.task_c_bridge_v0.dataset_io import load_lerobot_trajectories


COLORS = ("#4C78A8", "#F58518", "#54A24B", "#E45756", "#72B7B2")


@dataclass(frozen=True)
class StateNode:
    node_id: str
    summary: str
    semantic_state: dict[str, str]


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    dataset_root: Path
    dataset_repo_id: str
    associated_model_path: Path
    task_description: str
    task_semantic_label: str
    event_sequence: tuple[str, ...]
    events: dict[str, dict[str, str]]
    segments: tuple[SemanticSegmentSpec, ...]
    parent_labels_ko: dict[str, str]
    state_nodes: tuple[StateNode, ...]
    semantic_notes: tuple[str, ...] = ()


def _segment(
    segment_id: str,
    parent: str,
    label: str,
    label_ko: str,
    start: str,
    end: str,
    gripper: str,
    target: str | None,
    entry: dict[str, str],
    exit_state: dict[str, str],
    evidence: tuple[str, ...],
    confidence: str,
) -> SemanticSegmentSpec:
    return SemanticSegmentSpec(
        segment_id=segment_id,
        parent_subgoal=parent,
        semantic_label=label,
        label_ko=label_ko,
        start_anchor=start,
        end_anchor=end,
        gripper_semantics=gripper,
        manipulated_object=target,
        entry_state=entry,
        exit_state=exit_state,
        evidence=evidence,
        confidence=confidence,
    )


def _t2_spec() -> TaskSpec:
    initial = {
        "gripper": "open",
        "blue_block": "source_region",
        "held_object": "none",
    }
    held = {
        "gripper": "closed",
        "blue_block": "held",
        "held_object": "blue_block",
    }
    placed = {
        "gripper": "open",
        "blue_block": "on_black_table",
        "held_object": "none",
    }
    return TaskSpec(
        task_id="t2",
        dataset_root=Path("/home/rvlab/lerobot_datasets/a0509_blue_block_t2_table"),
        dataset_repo_id="local/a0509_blue_block_t2_table",
        associated_model_path=Path(
            "/home/rvlab/lerobot_models/models/"
            "act_a0509_blue_block_t2_table_bs32_40k_20260821/"
            "040000/pretrained_model"
        ),
        task_description="bring the blue block to black table",
        task_semantic_label="move_blue_block_from_source_region_to_black_table",
        event_sequence=("C1", "O1"),
        events={
            "C1": {
                "semantic_label": "grasp_blue_block_in_source_region",
                "label_ko": "출발 영역의 파란 블록 파지",
                "gripper_change": "open_to_closed",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_held",
                "confidence": "medium",
            },
            "O1": {
                "semantic_label": "release_blue_block_on_black_table",
                "label_ko": "검은 테이블 위에 파란 블록 방출",
                "gripper_change": "closed_to_open",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_on_black_table",
                "confidence": "high",
            },
        },
        segments=(
            _segment(
                "S1",
                "acquire_blue_block_from_source_region",
                "approach_blue_block_in_source_region",
                "출발 영역의 파란 블록으로 접근",
                "START",
                "C1",
                "open; closes at C1",
                "blue_block",
                initial,
                held,
                ("C1", "TCP approach motion", "prior multi-camera review"),
                "medium",
            ),
            _segment(
                "S2",
                "place_blue_block_on_black_table",
                "lift_transport_and_align_blue_block_to_black_table",
                "파란 블록을 들어 검은 테이블로 운반·정렬",
                "C1",
                "O1",
                "closed; opens at O1",
                "blue_block",
                held,
                placed,
                ("TASK_DESCRIPTION", "C1-O1", "TCP lift/transport", "prior multi-camera review"),
                "high",
            ),
            _segment(
                "S3",
                "finish_task",
                "retract_after_placing_blue_block_on_black_table",
                "블록을 놓은 뒤 검은 테이블에서 후퇴",
                "O1",
                "END",
                "open",
                None,
                placed,
                placed,
                ("O1", "TCP post-release motion"),
                "medium",
            ),
        ),
        parent_labels_ko={
            "acquire_blue_block_from_source_region": "출발 영역 블록 파지",
            "place_blue_block_on_black_table": "검은 테이블에 놓기",
            "finish_task": "후퇴",
        },
        state_nodes=(
            StateNode("START", "G: open\nBlock: source region\nHeld: none", initial),
            StateNode("C1", "G: closed\nBlock: held\nHeld: blue block", held),
            StateNode("O1", "G: open\nBlock: black table\nHeld: none", placed),
            StateNode("TAIL", "G: open\nState unchanged", placed),
        ),
        semantic_notes=(
            "TASK_DESCRIPTION does not name the source surface; source_region is intentionally geometry-neutral.",
        ),
    )


def _t3_spec() -> TaskSpec:
    initial = {
        "gripper": "open",
        "blue_block": "on_black_table",
        "held_object": "none",
    }
    held = {
        "gripper": "closed",
        "blue_block": "held",
        "held_object": "blue_block",
    }
    placed = {
        "gripper": "open",
        "blue_block": "off_black_table",
        "held_object": "none",
    }
    return TaskSpec(
        task_id="t3",
        dataset_root=Path("/home/rvlab/lerobot_datasets/a0509_blue_block_t3"),
        dataset_repo_id="local/a0509_blue_block_t3",
        associated_model_path=Path(
            "/home/rvlab/lerobot_models/models/"
            "act_a0509_blue_block_t3_bs32_40k_20260822/"
            "040000/pretrained_model"
        ),
        task_description="Move the blue block off the black table",
        task_semantic_label="move_blue_block_off_black_table",
        event_sequence=("C1", "O1"),
        events={
            "C1": {
                "semantic_label": "grasp_blue_block_on_black_table",
                "label_ko": "검은 테이블의 파란 블록 파지",
                "gripper_change": "open_to_closed",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_held",
                "confidence": "high",
            },
            "O1": {
                "semantic_label": "release_blue_block_off_black_table",
                "label_ko": "검은 테이블 밖에 파란 블록 방출",
                "gripper_change": "closed_to_open",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_off_black_table",
                "confidence": "high",
            },
        },
        segments=(
            _segment(
                "S1",
                "acquire_blue_block_from_black_table",
                "approach_blue_block_on_black_table",
                "검은 테이블의 파란 블록으로 접근",
                "START",
                "C1",
                "open; closes at C1",
                "blue_block",
                initial,
                held,
                ("TASK_DESCRIPTION", "C1", "TCP approach motion", "prior multi-camera review"),
                "high",
            ),
            _segment(
                "S2",
                "place_blue_block_off_black_table",
                "lift_transport_and_align_blue_block_off_black_table",
                "파란 블록을 들어 검은 테이블 밖으로 운반·정렬",
                "C1",
                "O1",
                "closed; opens at O1",
                "blue_block",
                held,
                placed,
                ("TASK_DESCRIPTION", "C1-O1", "TCP lift/transport", "prior multi-camera review"),
                "high",
            ),
            _segment(
                "S3",
                "finish_task",
                "retract_after_releasing_blue_block_off_black_table",
                "블록을 놓은 뒤 방출 영역에서 후퇴",
                "O1",
                "END",
                "open",
                None,
                placed,
                placed,
                ("O1", "TCP post-release motion"),
                "medium",
            ),
        ),
        parent_labels_ko={
            "acquire_blue_block_from_black_table": "검은 테이블 블록 파지",
            "place_blue_block_off_black_table": "테이블 밖에 놓기",
            "finish_task": "후퇴",
        },
        state_nodes=(
            StateNode("START", "G: open\nBlock: black table\nHeld: none", initial),
            StateNode("C1", "G: closed\nBlock: held\nHeld: blue block", held),
            StateNode("O1", "G: open\nBlock: off black table\nHeld: none", placed),
            StateNode("TAIL", "G: open\nState unchanged", placed),
        ),
    )


def _t4_spec() -> TaskSpec:
    initial = {
        "gripper": "open",
        "drawer": "closed",
        "blue_block": "on_black_table",
        "held_object": "none",
    }
    handle_held = {
        "gripper": "closed",
        "drawer": "closed",
        "blue_block": "on_black_table",
        "held_object": "drawer_handle",
    }
    drawer_open = {
        "gripper": "open",
        "drawer": "open",
        "blue_block": "on_black_table",
        "held_object": "none",
    }
    block_held = {
        "gripper": "closed",
        "drawer": "open",
        "blue_block": "held",
        "held_object": "blue_block",
    }
    block_in_drawer = {
        "gripper": "open",
        "drawer": "open",
        "blue_block": "in_drawer",
        "held_object": "none",
    }
    complete = {
        "gripper": "open",
        "drawer": "closed",
        "blue_block": "in_drawer",
        "held_object": "none",
    }
    return TaskSpec(
        task_id="t4",
        dataset_root=Path("/home/rvlab/lerobot_datasets/a0509_blue_block_t4"),
        dataset_repo_id="local/a0509_blue_block_t4",
        associated_model_path=Path(
            "/home/rvlab/lerobot_models/models/"
            "act_a0509_blue_block_t4_bs32_80k_20260823/"
            "080000/pretrained_model"
        ),
        task_description=(
            "Open the drawer, pick up the blue block, place it inside the drawer, "
            "and then close the drawer"
        ),
        task_semantic_label="store_blue_block_in_drawer_and_close_drawer",
        event_sequence=("C1", "O1", "C2", "O2"),
        events={
            "C1": {
                "semantic_label": "grasp_drawer_handle",
                "label_ko": "서랍 손잡이 파지",
                "gripper_change": "open_to_closed",
                "manipulated_object": "drawer_handle",
                "semantic_effect": "drawer_handle_held",
                "confidence": "high",
            },
            "O1": {
                "semantic_label": "release_handle_after_opening_drawer",
                "label_ko": "서랍을 연 뒤 손잡이 해제",
                "gripper_change": "closed_to_open",
                "manipulated_object": "drawer_handle",
                "semantic_effect": "drawer_open",
                "confidence": "high",
            },
            "C2": {
                "semantic_label": "grasp_blue_block_on_black_table",
                "label_ko": "검은 테이블의 파란 블록 파지",
                "gripper_change": "open_to_closed",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_held",
                "confidence": "high",
            },
            "O2": {
                "semantic_label": "release_blue_block_in_drawer",
                "label_ko": "서랍 안에 파란 블록 방출",
                "gripper_change": "closed_to_open",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_in_drawer",
                "confidence": "high",
            },
        },
        segments=(
            _segment(
                "S1",
                "open_drawer",
                "approach_drawer_handle",
                "서랍 손잡이로 접근",
                "START",
                "C1",
                "open; closes at C1",
                "drawer_handle",
                initial,
                handle_held,
                ("TASK_DESCRIPTION", "C1", "TCP approach motion", "prior multi-camera review"),
                "high",
            ),
            _segment(
                "S2",
                "open_drawer",
                "manipulate_drawer_open",
                "손잡이를 조작하여 서랍 열기",
                "C1",
                "O1",
                "closed; opens at O1",
                "drawer_handle",
                handle_held,
                drawer_open,
                ("TASK_DESCRIPTION", "C1-O1", "TCP drawer motion", "prior multi-camera review"),
                "high",
            ),
            _segment(
                "S3",
                "acquire_blue_block",
                "approach_blue_block_on_black_table",
                "검은 테이블의 파란 블록으로 접근",
                "O1",
                "C2",
                "open; closes at C2",
                "blue_block",
                drawer_open,
                block_held,
                ("TASK_DESCRIPTION", "O1-C2", "TCP approach motion", "prior multi-camera review"),
                "high",
            ),
            _segment(
                "S4",
                "place_blue_block_in_drawer",
                "lift_transport_and_align_blue_block_in_drawer",
                "파란 블록을 들어 서랍 안으로 운반·정렬",
                "C2",
                "O2",
                "closed; opens at O2",
                "blue_block",
                block_held,
                block_in_drawer,
                ("TASK_DESCRIPTION", "C2-O2", "TCP lift/transport", "prior multi-camera review"),
                "high",
            ),
            _segment(
                "S5",
                "close_drawer",
                "close_drawer_and_retract_with_open_gripper",
                "열린 그리퍼로 서랍을 닫고 후퇴",
                "O2",
                "END",
                "open throughout",
                "drawer_front",
                block_in_drawer,
                complete,
                ("TASK_DESCRIPTION", "TCP post-O2 motion", "prior multi-camera review"),
                "medium",
            ),
        ),
        parent_labels_ko={
            "open_drawer": "서랍 열기",
            "acquire_blue_block": "블록 파지",
            "place_blue_block_in_drawer": "서랍에 놓기",
            "close_drawer": "서랍 닫기",
        },
        state_nodes=(
            StateNode("START", "G: open\nDrawer: closed\nBlock: black table", initial),
            StateNode("C1", "G: closed\nDrawer: closed\nHeld: handle", handle_held),
            StateNode("O1", "G: open\nDrawer: open\nBlock: black table", drawer_open),
            StateNode("C2", "G: closed\nDrawer: open\nHeld: blue block", block_held),
            StateNode("O2", "G: open\nDrawer: open\nBlock: in drawer", block_in_drawer),
            StateNode("TAIL", "G: open\nDrawer: closed\nBlock: in drawer", complete),
        ),
        semantic_notes=(
            "Drawer closing occurs after O2 while the gripper remains open, so S5 is not anchored by another C/O event.",
            "S5 is kept as one semantic tail; no internal close-drawer boundary is invented.",
        ),
    )


def _t5_spec() -> TaskSpec:
    initial = {
        "gripper": "open",
        "drawer": "closed",
        "blue_block": "in_drawer",
        "held_object": "none",
    }
    handle_held = {
        "gripper": "closed",
        "drawer": "closed",
        "blue_block": "in_drawer",
        "held_object": "drawer_handle",
    }
    drawer_open = {
        "gripper": "open",
        "drawer": "open",
        "blue_block": "in_drawer",
        "held_object": "none",
    }
    block_held = {
        "gripper": "closed",
        "drawer": "open",
        "blue_block": "held",
        "held_object": "blue_block",
    }
    block_on_table = {
        "gripper": "open",
        "drawer": "open",
        "blue_block": "on_black_table",
        "held_object": "none",
    }
    complete = {
        "gripper": "open",
        "drawer": "closed",
        "blue_block": "on_black_table",
        "held_object": "none",
    }
    return TaskSpec(
        task_id="t5",
        dataset_root=Path("/home/rvlab/lerobot_datasets/a0509_blue_block_t5"),
        dataset_repo_id="local/a0509_blue_block_v5",
        associated_model_path=Path(
            "/home/rvlab/lerobot_models/models/"
            "act_a0509_blue_block_t5_bs32_80k_20260824/"
            "080000/pretrained_model"
        ),
        task_description=(
            "Open the drawer, take out the blue block, place it on the black table, "
            "and close the drawer."
        ),
        task_semantic_label="retrieve_blue_block_from_drawer_place_on_black_table_and_close_drawer",
        event_sequence=("C1", "O1", "C2", "O2"),
        events={
            "C1": {
                "semantic_label": "grasp_drawer_handle",
                "label_ko": "서랍 손잡이 파지",
                "gripper_change": "open_to_closed",
                "manipulated_object": "drawer_handle",
                "semantic_effect": "drawer_handle_held",
                "confidence": "high",
            },
            "O1": {
                "semantic_label": "release_handle_after_opening_drawer",
                "label_ko": "서랍을 연 뒤 손잡이 해제",
                "gripper_change": "closed_to_open",
                "manipulated_object": "drawer_handle",
                "semantic_effect": "drawer_open",
                "confidence": "high",
            },
            "C2": {
                "semantic_label": "grasp_blue_block_in_drawer",
                "label_ko": "서랍 안의 파란 블록 파지",
                "gripper_change": "open_to_closed",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_held",
                "confidence": "high",
            },
            "O2": {
                "semantic_label": "release_blue_block_on_black_table",
                "label_ko": "검은 테이블 위에 파란 블록 방출",
                "gripper_change": "closed_to_open",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_on_black_table",
                "confidence": "high",
            },
        },
        segments=(
            _segment(
                "S1",
                "open_drawer",
                "approach_drawer_handle",
                "서랍 손잡이로 접근",
                "START",
                "C1",
                "open; closes at C1",
                "drawer_handle",
                initial,
                handle_held,
                ("TASK_DESCRIPTION", "C1", "TCP approach motion", "T4 inverse-task anchor correspondence"),
                "high",
            ),
            _segment(
                "S2",
                "open_drawer",
                "manipulate_drawer_open",
                "손잡이를 조작하여 서랍 열기",
                "C1",
                "O1",
                "closed; opens at O1",
                "drawer_handle",
                handle_held,
                drawer_open,
                ("TASK_DESCRIPTION", "C1-O1", "TCP drawer motion", "T4 inverse-task anchor correspondence"),
                "high",
            ),
            _segment(
                "S3",
                "acquire_blue_block_from_drawer",
                "approach_blue_block_in_drawer",
                "서랍 안의 파란 블록으로 접근",
                "O1",
                "C2",
                "open; closes at C2",
                "blue_block",
                drawer_open,
                block_held,
                ("TASK_DESCRIPTION", "O1-C2", "TCP approach motion", "T4 in-drawer release anchor correspondence"),
                "high",
            ),
            _segment(
                "S4",
                "place_blue_block_on_black_table",
                "lift_transport_and_align_blue_block_on_black_table",
                "파란 블록을 들어 검은 테이블로 운반·정렬",
                "C2",
                "O2",
                "closed; opens at O2",
                "blue_block",
                block_held,
                block_on_table,
                ("TASK_DESCRIPTION", "C2-O2", "TCP lift/transport", "T4 black-table grasp anchor correspondence"),
                "high",
            ),
            _segment(
                "S5",
                "close_drawer",
                "close_drawer_and_retract_with_open_gripper",
                "열린 그리퍼로 서랍을 닫고 후퇴",
                "O2",
                "END",
                "open throughout",
                "drawer_front",
                block_on_table,
                complete,
                ("TASK_DESCRIPTION", "TCP post-O2 motion", "T4 close-drawer tail correspondence"),
                "medium",
            ),
        ),
        parent_labels_ko={
            "open_drawer": "서랍 열기",
            "acquire_blue_block_from_drawer": "서랍 속 블록 파지",
            "place_blue_block_on_black_table": "검은 테이블에 놓기",
            "close_drawer": "서랍 닫기",
        },
        state_nodes=(
            StateNode("START", "G: open\nDrawer: closed\nBlock: in drawer", initial),
            StateNode("C1", "G: closed\nDrawer: closed\nHeld: handle", handle_held),
            StateNode("O1", "G: open\nDrawer: open\nBlock: in drawer", drawer_open),
            StateNode("C2", "G: closed\nDrawer: open\nHeld: blue block", block_held),
            StateNode("O2", "G: open\nDrawer: open\nBlock: black table", block_on_table),
            StateNode("TAIL", "G: open\nDrawer: closed\nBlock: black table", complete),
        ),
        semantic_notes=(
            "All 30 episodes share the audited C1-O1-C2-O2 gripper-event grammar.",
            "The C2 and O2 median anchors correspond to T4's in-drawer release and black-table grasp anchors, respectively, supporting the inverse-task semantic labels.",
            "Drawer closing occurs after O2 while the gripper remains open, so S5 is not anchored by another C/O event.",
            "S5 is kept as one semantic tail; no internal close-drawer boundary is invented.",
            "Semantic object roles come from TASK_DESCRIPTION, event order, and cross-task Cartesian correspondence; no image/frame averaging is used.",
        ),
    )


def _t6_spec() -> TaskSpec:
    initial = {
        "gripper": "open",
        "moving_blue_block": "source_region",
        "support_blue_block": "on_black_table",
        "held_object": "none",
        "stack": "unassembled",
    }
    held = {
        "gripper": "closed",
        "moving_blue_block": "held",
        "support_blue_block": "on_black_table",
        "held_object": "moving_blue_block",
        "stack": "unassembled",
    }
    stacked = {
        "gripper": "open",
        "moving_blue_block": "on_top_of_support_blue_block",
        "support_blue_block": "on_black_table",
        "held_object": "none",
        "stack": "assembled",
    }
    return TaskSpec(
        task_id="t6",
        dataset_root=Path("/home/rvlab/lerobot_datasets/a0509_blue_block_t6"),
        dataset_repo_id="local/a0509_blue_block_v6",
        associated_model_path=Path(
            "/home/rvlab/lerobot_models/models/"
            "act_a0509_blue_block_t6_bs32_50k_20260824/"
            "050000/pretrained_model"
        ),
        task_description=(
            "Stack the blue block on top of the other blue block on the black table."
        ),
        task_semantic_label=(
            "stack_one_blue_block_on_another_blue_block_on_black_table"
        ),
        event_sequence=("C1", "O1"),
        events={
            "C1": {
                "semantic_label": "grasp_blue_block_for_stacking",
                "label_ko": "적층할 파란 블록 파지",
                "gripper_change": "open_to_closed",
                "manipulated_object": "moving_blue_block",
                "semantic_effect": "moving_blue_block_held",
                "confidence": "high",
            },
            "O1": {
                "semantic_label": "release_blue_block_on_top_of_support_blue_block",
                "label_ko": "지지 파란 블록 위에 이동 블록 방출",
                "gripper_change": "closed_to_open",
                "manipulated_object": "moving_blue_block",
                "semantic_effect": "blue_block_stack_assembled",
                "confidence": "high",
            },
        },
        segments=(
            _segment(
                "S1",
                "acquire_blue_block_for_stacking",
                "approach_moving_blue_block_in_source_region",
                "적층할 파란 블록으로 접근",
                "START",
                "C1",
                "open; closes at C1",
                "moving_blue_block",
                initial,
                held,
                ("TASK_DESCRIPTION", "C1", "TCP approach motion"),
                "high",
            ),
            _segment(
                "S2",
                "stack_blue_blocks_on_black_table",
                "lift_transport_align_and_stack_blue_block_on_support_block",
                "파란 블록을 들어 지지 블록 위로 운반·정렬·적층",
                "C1",
                "O1",
                "closed; opens at O1",
                "moving_blue_block",
                held,
                stacked,
                ("TASK_DESCRIPTION", "C1-O1", "TCP lift/transport/alignment motion"),
                "high",
            ),
            _segment(
                "S3",
                "finish_task",
                "retract_after_stacking_blue_blocks",
                "블록 적층 후 안전하게 후퇴",
                "O1",
                "END",
                "open",
                None,
                stacked,
                stacked,
                ("O1", "TCP post-release motion"),
                "medium",
            ),
        ),
        parent_labels_ko={
            "acquire_blue_block_for_stacking": "적층할 블록 파지",
            "stack_blue_blocks_on_black_table": "블록 운반·정렬·적층",
            "finish_task": "후퇴",
        },
        state_nodes=(
            StateNode(
                "START",
                "G: open\nMoving block: source\nSupport block: black table",
                initial,
            ),
            StateNode(
                "C1",
                "G: closed\nMoving block: held\nStack: unassembled",
                held,
            ),
            StateNode(
                "O1",
                "G: open\nMoving block: on support block\nStack: assembled",
                stacked,
            ),
            StateNode("TAIL", "G: open\nStack: assembled\nState unchanged", stacked),
        ),
        semantic_notes=(
            "All 30 episodes share the audited C1-O1 gripper-event grammar.",
            "TASK_DESCRIPTION identifies the stationary support block as the other blue block on the black table and disambiguates O1 as the stacking release.",
            "The moving block's source surface is not stated, so source_region is intentionally geometry-neutral.",
            "S2 remains one transport-align-stack semantic span; no unobserved contact boundary before O1 is invented.",
            "Semantic object roles come from TASK_DESCRIPTION, event order, and Cartesian motion; no image/frame averaging is used.",
        ),
    )


def _t7_spec() -> TaskSpec:
    initial = {
        "gripper": "open",
        "blue_block": "on_top_of_drawer",
        "held_object": "none",
    }
    held = {
        "gripper": "closed",
        "blue_block": "held",
        "held_object": "blue_block",
    }
    placed = {
        "gripper": "open",
        "blue_block": "on_black_table",
        "held_object": "none",
    }
    return TaskSpec(
        task_id="t7",
        dataset_root=Path("/home/rvlab/lerobot_datasets/a0509_blue_block_t7"),
        dataset_repo_id="rvlab/a0509_blue_block_v7",
        associated_model_path=Path(
            "/home/rvlab/lerobot_models/models/"
            "act_a0509_blue_block_t7_bs32_40k_20260828/"
            "040000/pretrained_model"
        ),
        task_description=(
            "Place the blue block on top of the drawer onto the black table."
        ),
        task_semantic_label=(
            "move_blue_block_from_top_of_drawer_to_black_table"
        ),
        event_sequence=("C1", "O1"),
        events={
            "C1": {
                "semantic_label": "grasp_blue_block_on_top_of_drawer",
                "label_ko": "서랍 위의 파란 블록 파지",
                "gripper_change": "open_to_closed",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_held",
                "confidence": "high",
            },
            "O1": {
                "semantic_label": "release_blue_block_on_black_table",
                "label_ko": "검은 테이블 위에 파란 블록 방출",
                "gripper_change": "closed_to_open",
                "manipulated_object": "blue_block",
                "semantic_effect": "blue_block_on_black_table",
                "confidence": "high",
            },
        },
        segments=(
            _segment(
                "S1",
                "acquire_blue_block_from_top_of_drawer",
                "approach_blue_block_on_top_of_drawer",
                "서랍 위의 파란 블록으로 접근",
                "START",
                "C1",
                "open; closes at C1",
                "blue_block",
                initial,
                held,
                ("TASK_DESCRIPTION", "C1", "TCP approach motion"),
                "high",
            ),
            _segment(
                "S2",
                "place_blue_block_on_black_table",
                "lift_transport_and_align_blue_block_to_black_table",
                "파란 블록을 들어 검은 테이블로 운반·정렬",
                "C1",
                "O1",
                "closed; opens at O1",
                "blue_block",
                held,
                placed,
                ("TASK_DESCRIPTION", "C1-O1", "TCP lift/transport motion"),
                "high",
            ),
            _segment(
                "S3",
                "finish_task",
                "retract_after_placing_blue_block_on_black_table",
                "검은 테이블에 블록을 놓은 뒤 후퇴",
                "O1",
                "END",
                "open",
                None,
                placed,
                placed,
                ("O1", "TCP post-release motion"),
                "medium",
            ),
        ),
        parent_labels_ko={
            "acquire_blue_block_from_top_of_drawer": "서랍 위 블록 파지",
            "place_blue_block_on_black_table": "검은 테이블에 놓기",
            "finish_task": "후퇴",
        },
        state_nodes=(
            StateNode(
                "START",
                "G: open\nBlock: top of drawer\nHeld: none",
                initial,
            ),
            StateNode(
                "C1",
                "G: closed\nBlock: held\nHeld: blue block",
                held,
            ),
            StateNode(
                "O1",
                "G: open\nBlock: black table\nHeld: none",
                placed,
            ),
            StateNode("TAIL", "G: open\nState unchanged", placed),
        ),
        semantic_notes=(
            "All 30 episodes share the audited C1-O1 gripper-event grammar.",
            "TASK_DESCRIPTION identifies the source as the top of the drawer and the destination as the black table.",
            "S2 remains one lift-transport-align-place semantic span; no unobserved contact boundary before O1 is invented.",
            "Semantic object roles come from TASK_DESCRIPTION, event order, and Cartesian motion; no image/frame averaging is used.",
        ),
    )


def _t8_spec() -> TaskSpec:
    initial = {
        "gripper": "open",
        "moving_blue_block": "on_top_of_support_blue_block",
        "support_blue_block": "on_black_table",
        "held_object": "none",
        "stack": "assembled",
    }
    held = {
        "gripper": "closed",
        "moving_blue_block": "held",
        "support_blue_block": "on_black_table",
        "held_object": "moving_blue_block",
        "stack": "disassembling",
    }
    placed = {
        "gripper": "open",
        "moving_blue_block": "on_black_table",
        "support_blue_block": "on_black_table",
        "held_object": "none",
        "stack": "disassembled",
    }
    return TaskSpec(
        task_id="t8",
        dataset_root=Path("/home/rvlab/lerobot_datasets/a0509_blue_block_t8"),
        dataset_repo_id="rvlab/a0509_blue_block_v8",
        associated_model_path=Path(
            "/home/rvlab/lerobot_models/models/"
            "act_a0509_blue_block_t8_bs32_50k_20260828/"
            "050000/pretrained_model"
        ),
        task_description=(
            "Pick up the blue block on top of the other block and place it on the black table."
        ),
        task_semantic_label=(
            "unstack_top_blue_block_and_place_it_on_black_table"
        ),
        event_sequence=("C1", "O1"),
        events={
            "C1": {
                "semantic_label": "grasp_top_blue_block_from_stack",
                "label_ko": "적층 상단의 파란 블록 파지",
                "gripper_change": "open_to_closed",
                "manipulated_object": "moving_blue_block",
                "semantic_effect": "top_blue_block_held",
                "confidence": "high",
            },
            "O1": {
                "semantic_label": "release_unstacked_blue_block_on_black_table",
                "label_ko": "분리한 파란 블록을 검은 테이블에 방출",
                "gripper_change": "closed_to_open",
                "manipulated_object": "moving_blue_block",
                "semantic_effect": "blue_block_stack_disassembled",
                "confidence": "high",
            },
        },
        segments=(
            _segment(
                "S1",
                "acquire_top_blue_block_from_stack",
                "approach_top_blue_block_on_support_block",
                "지지 블록 위 상단 파란 블록으로 접근",
                "START",
                "C1",
                "open; closes at C1",
                "moving_blue_block",
                initial,
                held,
                ("TASK_DESCRIPTION", "C1", "TCP approach motion"),
                "high",
            ),
            _segment(
                "S2",
                "unstack_and_place_blue_block_on_black_table",
                "lift_separate_transport_and_align_top_block_to_black_table",
                "상단 블록을 들어 분리하고 검은 테이블로 운반·정렬",
                "C1",
                "O1",
                "closed; opens at O1",
                "moving_blue_block",
                held,
                placed,
                (
                    "TASK_DESCRIPTION",
                    "C1-O1",
                    "TCP lift/separation/transport motion",
                    "T6 inverse-task semantic correspondence",
                ),
                "high",
            ),
            _segment(
                "S3",
                "finish_task",
                "retract_after_unstacking_and_placing_blue_block",
                "분리한 블록을 놓은 뒤 후퇴",
                "O1",
                "END",
                "open",
                None,
                placed,
                placed,
                ("O1", "TCP post-release motion"),
                "medium",
            ),
        ),
        parent_labels_ko={
            "acquire_top_blue_block_from_stack": "적층 상단 블록 파지",
            "unstack_and_place_blue_block_on_black_table": "분리·운반·배치",
            "finish_task": "후퇴",
        },
        state_nodes=(
            StateNode(
                "START",
                "G: open\nMoving block: on support\nStack: assembled",
                initial,
            ),
            StateNode(
                "C1",
                "G: closed\nMoving block: held\nStack: disassembling",
                held,
            ),
            StateNode(
                "O1",
                "G: open\nBoth blocks: black table\nStack: disassembled",
                placed,
            ),
            StateNode(
                "TAIL",
                "G: open\nStack: disassembled\nState unchanged",
                placed,
            ),
        ),
        semantic_notes=(
            "All 30 episodes share the audited C1-O1 gripper-event grammar.",
            "TASK_DESCRIPTION identifies the manipulated object as the upper block and leaves the lower support block on the black table.",
            "T8 is modeled as the semantic inverse of T6 stacking: the stack changes from assembled to disassembled at the manipulated-block release.",
            "S2 remains one unstack-lift-transport-align-place semantic span; no unobserved contact boundary before O1 is invented.",
            "Semantic object roles come from TASK_DESCRIPTION, event order, T6 inverse-task correspondence, and Cartesian motion; no image/frame averaging is used.",
        ),
    )

TASKS = {
    spec.task_id: spec
    for spec in (
        _t2_spec(),
        _t3_spec(),
        _t4_spec(),
        _t5_spec(),
        _t6_spec(),
        _t7_spec(),
        _t8_spec(),
    )
}


def _configure_plot_font() -> None:
    candidates = (
        Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf"),
        Path("/usr/share/opentype/noto/NotoSansCJK-Regular.ttc"),
    )
    for path in candidates:
        if path.exists():
            from matplotlib import font_manager

            font_manager.fontManager.addfont(str(path))
            plt.rcParams["font.family"] = font_manager.FontProperties(
                fname=str(path)
            ).get_name()
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.dpi"] = 120
    plt.rcParams["savefig.dpi"] = 220


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (SemanticSegmentSpec, StateNode)):
        return {
            field: _jsonable(getattr(value, field))
            for field in value.__dataclass_fields__
        }
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _event_centers(
    trajectories: list[Any],
    episode_events: list[dict[str, int]],
    event_sequence: tuple[str, ...],
) -> dict[str, np.ndarray]:
    return {
        event: np.median(
            np.stack(
                [
                    trajectory.xyz_mm[events[event]]
                    for trajectory, events in zip(trajectories, episode_events)
                ],
                axis=0,
            ),
            axis=0,
        )
        for event in event_sequence
    }


def _plot_projection(
    ax: Any,
    aggregates: list[dict[str, Any]],
    event_centers: dict[str, np.ndarray],
    axes: tuple[int, int],
    xlabel: str,
    ylabel: str,
    title: str,
) -> None:
    for color, aggregate in zip(COLORS, aggregates):
        for episode_xyz in aggregate["episode_xyz_mm"]:
            ax.plot(
                episode_xyz[:, axes[0]],
                episode_xyz[:, axes[1]],
                color=color,
                alpha=0.065,
                linewidth=0.65,
            )
        median = aggregate["median_xyz_mm"]
        ax.plot(
            median[:, axes[0]],
            median[:, axes[1]],
            color=color,
            linewidth=3.2,
        )
    for event, center in event_centers.items():
        ax.scatter(center[axes[0]], center[axes[1]], color="#111111", s=34, zorder=8)
        ax.annotate(event, (center[axes[0]], center[axes[1]]), xytext=(5, 5), textcoords="offset points", fontsize=9)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontweight="bold")
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="datalim")


def _parent_runs(spec: TaskSpec) -> list[tuple[int, int, str]]:
    output: list[tuple[int, int, str]] = []
    start = 0
    current = spec.segments[0].parent_subgoal
    for index, segment in enumerate(spec.segments[1:], start=1):
        if segment.parent_subgoal != current:
            output.append((start, index, current))
            start = index
            current = segment.parent_subgoal
    output.append((start, len(spec.segments), current))
    return output


def _plot_state_graph(ax: Any, spec: TaskSpec, aggregates: list[dict[str, Any]]) -> None:
    segment_count = len(aggregates)
    ax.set_xlim(-0.3, segment_count + 0.3)
    ax.set_ylim(-1.8, 2.25)
    ax.axis("off")
    ax.set_title("D. 계층적 semantic subgoal과 world-state", loc="left", fontweight="bold")
    if len(spec.state_nodes) != segment_count + 1:
        raise ValueError(f"{spec.task_id} state-node count does not match segments")

    node_size = 760 if segment_count <= 3 else 660
    state_font = 8.0 if segment_count <= 3 else 7.2
    for index, node in enumerate(spec.state_nodes):
        ax.scatter(
            [index],
            [0.25],
            s=node_size,
            color="#F2F4F7" if node.node_id in {"START", "TAIL"} else "#FFF3CD",
            edgecolor="#222222",
            linewidth=1.15,
            zorder=5,
        )
        ax.text(index, 0.25, node.node_id, ha="center", va="center", fontweight="bold", fontsize=8.5, zorder=6)
        ax.text(index, -0.48, node.summary, ha="center", va="top", fontsize=state_font)

    for index, (color, aggregate) in enumerate(zip(COLORS, aggregates)):
        semantic = aggregate["spec"]
        ax.annotate(
            "",
            xy=(index + 0.82, 0.25),
            xytext=(index + 0.18, 0.25),
            arrowprops={"arrowstyle": "-|>", "lw": 7, "color": color, "alpha": 0.85},
        )
        ax.text(
            index + 0.5,
            0.86 if index % 2 == 0 else 1.28,
            f"{semantic.segment_id}  {semantic.label_ko}\nG: {semantic.gripper_semantics}",
            ha="center",
            va="center",
            fontsize=8.1 if segment_count <= 3 else 7.4,
            bbox={"boxstyle": "round,pad=0.22", "fc": "white", "ec": color},
        )

    for start, end, parent in _parent_runs(spec):
        color = COLORS[start]
        ax.plot(
            [start + 0.08, end - 0.08],
            [1.86, 1.86],
            color=color,
            alpha=0.32,
            linewidth=9,
            solid_capstyle="round",
        )
        ax.text(
            (start + end) / 2,
            2.08,
            spec.parent_labels_ko[parent],
            ha="center",
            va="center",
            fontsize=8.5,
            fontweight="bold",
        )


def _plot_main_graph(
    output_dir: Path,
    spec: TaskSpec,
    aggregates: list[dict[str, Any]],
    event_centers: dict[str, np.ndarray],
) -> None:
    _configure_plot_font()
    figure = plt.figure(figsize=(18, 12), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, height_ratios=(1.05, 0.95))
    ax3d = figure.add_subplot(grid[0, 0], projection="3d")
    ax_xz = figure.add_subplot(grid[0, 1])
    ax_residual = figure.add_subplot(grid[1, 0])
    ax_state = figure.add_subplot(grid[1, 1])

    for color, aggregate in zip(COLORS, aggregates):
        for episode_xyz in aggregate["episode_xyz_mm"]:
            ax3d.plot(*episode_xyz.T, color=color, alpha=0.05, linewidth=0.55)
        ax3d.plot(*aggregate["median_xyz_mm"].T, color=color, linewidth=3.2)
    for event, center in event_centers.items():
        ax3d.scatter(*center, color="#111111", s=32)
        ax3d.text(*center, f"  {event}", fontsize=9)
    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")
    ax3d.set_title(
        f"A. {spec.task_id.upper()} semantic-phase 궤적과 component-median",
        loc="left",
        fontweight="bold",
    )
    ax3d.view_init(elev=24, azim=-58)

    _plot_projection(
        ax_xz,
        aggregates,
        event_centers,
        (0, 2),
        "X [mm]",
        "Z [mm]",
        "B. X-Z: 굵은 색 선=phase별 median, 흐린 선=전체 분포",
    )

    for index, (color, aggregate) in enumerate(zip(COLORS, aggregates)):
        x = index + aggregate["phase"]
        ax_residual.fill_between(
            x,
            aggregate["residual_p50_mm"],
            aggregate["residual_p90_mm"],
            color=color,
            alpha=0.22,
        )
        ax_residual.plot(
            x,
            aggregate["residual_p90_mm"],
            color=color,
            linewidth=2.0,
            label="p90" if index == 0 else None,
        )
        ax_residual.plot(
            x,
            aggregate["residual_p50_mm"],
            color=color,
            linewidth=1.0,
            alpha=0.75,
            label="p50" if index == 0 else None,
        )
        ax_residual.text(
            index + 0.5,
            float(np.max(aggregate["residual_p90_mm"])) + 1.0,
            aggregate["spec"].segment_id,
            ha="center",
            fontsize=9,
        )
    for boundary in range(1, len(aggregates)):
        ax_residual.axvline(boundary, color="#AAAAAA", linewidth=0.8)
    ax_residual.set_xlim(0.0, float(len(aggregates)))
    ax_residual.set_xlabel("concatenated semantic phase (각 구간 0→1)")
    ax_residual.set_ylabel("대표 궤적으로부터 Cartesian residual [mm]")
    ax_residual.set_title("C. 전체 episode의 phase별 분산(p50/p90)", loc="left", fontweight="bold")
    ax_residual.grid(True, alpha=0.2)
    ax_residual.legend(loc="upper left", fontsize=8)

    _plot_state_graph(ax_state, spec, aggregates)
    figure.suptitle(
        f"{spec.task_id.upper()} semantic-only standardized graph "
        f"({aggregates[0]['episode_xyz_mm'].shape[0]} episodes)\n"
        f'TASK_DESCRIPTION="{spec.task_description}"',
        fontsize=16,
        fontweight="bold",
    )
    figure.savefig(output_dir / f"{spec.task_id}_semantic_only_graph.png", bbox_inches="tight")
    figure.savefig(output_dir / f"{spec.task_id}_semantic_only_graph.svg", bbox_inches="tight")
    plt.close(figure)


def _state_text(state: dict[str, str]) -> str:
    return "\n".join(f"{key}: {value}" for key, value in state.items())


def _plot_hierarchy(output_dir: Path, spec: TaskSpec) -> None:
    _configure_plot_font()
    figure, (ax_tree, ax_state) = plt.subplots(2, 1, figsize=(18, 10), constrained_layout=True)
    ax_tree.axis("off")
    ax_state.axis("off")
    ax_tree.set_xlim(0.0, 10.0)
    ax_tree.set_ylim(0.0, 5.0)
    ax_tree.set_title(f"A. {spec.task_id.upper()} hierarchical semantic ontology", loc="left", fontweight="bold")
    ax_tree.text(5.0, 4.5, spec.task_semantic_label, ha="center", va="center", fontsize=12, fontweight="bold", bbox={"boxstyle": "round,pad=0.35", "fc": "#E8EDF4", "ec": "#34495E"})

    parents = [parent for _, _, parent in _parent_runs(spec)]
    parent_x = np.linspace(1.0, 9.0, len(parents))
    parent_position = dict(zip(parents, parent_x))
    for parent, x in parent_position.items():
        ax_tree.annotate("", xy=(x, 3.55), xytext=(5.0, 4.2), arrowprops={"arrowstyle": "->", "color": "#777777"})
        ax_tree.text(x, 3.35, parent, ha="center", va="center", fontsize=9, fontweight="bold", bbox={"boxstyle": "round,pad=0.22", "fc": "white", "ec": "#777777"})
    counts: dict[str, int] = {}
    totals = {parent: sum(segment.parent_subgoal == parent for segment in spec.segments) for parent in parents}
    for index, (color, segment) in enumerate(zip(COLORS, spec.segments)):
        offset = counts.get(segment.parent_subgoal, 0)
        counts[segment.parent_subgoal] = offset + 1
        total = totals[segment.parent_subgoal]
        x = parent_position[segment.parent_subgoal] + (offset - (total - 1) / 2) * 1.35
        ax_tree.annotate("", xy=(x, 1.85), xytext=(parent_position[segment.parent_subgoal], 3.05), arrowprops={"arrowstyle": "->", "color": color})
        ax_tree.text(x, 1.55, f"{segment.segment_id}\n{segment.semantic_label}\nG: {segment.gripper_semantics}", ha="center", va="center", fontsize=8.1, bbox={"boxstyle": "round,pad=0.28", "fc": "white", "ec": color})

    node_count = len(spec.state_nodes)
    ax_state.set_xlim(-0.2, node_count - 0.8)
    ax_state.set_ylim(-0.5, 3.7)
    ax_state.set_title("B. Semantic world-state progression", loc="left", fontweight="bold")
    x_values = np.linspace(0.0, node_count - 1.0, node_count)
    for index, (x, node) in enumerate(zip(x_values, spec.state_nodes)):
        ax_state.text(x, 1.55, f"{node.node_id}\n\n{_state_text(node.semantic_state)}", ha="center", va="center", fontsize=8.3 if node_count > 4 else 9.5, bbox={"boxstyle": "round,pad=0.38", "fc": "#F7F9FB", "ec": "#536878"})
        if index < node_count - 1:
            next_x = x_values[index + 1]
            label = spec.segments[index].segment_id
            ax_state.annotate("", xy=(next_x - 0.23, 1.55), xytext=(x + 0.23, 1.55), arrowprops={"arrowstyle": "-|>", "lw": 2.0, "color": "#536878"})
            ax_state.text((x + next_x) / 2, 1.83, label, ha="center", va="bottom", fontsize=7.3)
    figure.suptitle(f"{spec.task_id.upper()} semantic hierarchy and state changes", fontsize=16, fontweight="bold")
    figure.savefig(output_dir / f"{spec.task_id}_semantic_hierarchy.png", bbox_inches="tight")
    figure.savefig(output_dir / f"{spec.task_id}_semantic_hierarchy.svg", bbox_inches="tight")
    plt.close(figure)


def _write_tables(
    output_dir: Path,
    spec: TaskSpec,
    aggregates: list[dict[str, Any]],
    event_centers: dict[str, np.ndarray],
) -> None:
    with (output_dir / "semantic_segment_catalog.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = (
            "segment_id",
            "parent_subgoal",
            "semantic_label",
            "label_ko",
            "start_anchor",
            "end_anchor",
            "gripper_semantics",
            "manipulated_object",
            "entry_state",
            "exit_state",
            "path_length_median_mm",
            "phase_residual_p90_mean_mm",
            "confidence",
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for aggregate in aggregates:
            semantic = aggregate["spec"]
            writer.writerow(
                {
                    "segment_id": semantic.segment_id,
                    "parent_subgoal": semantic.parent_subgoal,
                    "semantic_label": semantic.semantic_label,
                    "label_ko": semantic.label_ko,
                    "start_anchor": semantic.start_anchor,
                    "end_anchor": semantic.end_anchor,
                    "gripper_semantics": semantic.gripper_semantics,
                    "manipulated_object": semantic.manipulated_object or "none",
                    "entry_state": json.dumps(semantic.entry_state, ensure_ascii=False, sort_keys=True),
                    "exit_state": json.dumps(semantic.exit_state, ensure_ascii=False, sort_keys=True),
                    "path_length_median_mm": aggregate["path_length_mm"]["median"],
                    "phase_residual_p90_mean_mm": aggregate["phase_residual_mm"]["p90_mean"],
                    "confidence": semantic.confidence,
                }
            )

    with (output_dir / "semantic_event_catalog.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = (
            "event_id",
            "semantic_label",
            "label_ko",
            "gripper_change",
            "manipulated_object",
            "semantic_effect",
            "representative_x_mm",
            "representative_y_mm",
            "representative_z_mm",
            "confidence",
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for event in spec.event_sequence:
            item = spec.events[event]
            center = event_centers[event]
            writer.writerow(
                {
                    "event_id": event,
                    **item,
                    "representative_x_mm": center[0],
                    "representative_y_mm": center[1],
                    "representative_z_mm": center[2],
                }
            )

    with (output_dir / "representative_semantic_phase.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = (
            "segment_id",
            "semantic_label",
            "semantic_phase",
            "median_x_mm",
            "median_y_mm",
            "median_z_mm",
            "mean_x_mm",
            "mean_y_mm",
            "mean_z_mm",
            "residual_p50_mm",
            "residual_p90_mm",
            "residual_p95_mm",
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for aggregate in aggregates:
            semantic = aggregate["spec"]
            for index, phase in enumerate(aggregate["phase"]):
                median = aggregate["median_xyz_mm"][index]
                mean = aggregate["mean_xyz_mm"][index]
                writer.writerow(
                    {
                        "segment_id": semantic.segment_id,
                        "semantic_label": semantic.semantic_label,
                        "semantic_phase": phase,
                        "median_x_mm": median[0],
                        "median_y_mm": median[1],
                        "median_z_mm": median[2],
                        "mean_x_mm": mean[0],
                        "mean_y_mm": mean[1],
                        "mean_z_mm": mean[2],
                        "residual_p50_mm": aggregate["residual_p50_mm"][index],
                        "residual_p90_mm": aggregate["residual_p90_mm"][index],
                        "residual_p95_mm": aggregate["residual_p95_mm"][index],
                    }
                )


def _write_npz(output_dir: Path, aggregates: list[dict[str, Any]]) -> None:
    arrays: dict[str, np.ndarray] = {}
    for aggregate in aggregates:
        prefix = aggregate["spec"].segment_id
        for name in (
            "phase",
            "episode_ids",
            "episode_xyz_mm",
            "median_xyz_mm",
            "mean_xyz_mm",
            "covariance_mm2",
            "mad_mm",
            "residual_p50_mm",
            "residual_p90_mm",
            "residual_p95_mm",
        ):
            arrays[f"{prefix}_{name}"] = np.asarray(aggregate[name])
    np.savez_compressed(output_dir / "standardized_semantic_phase_trajectories.npz", **arrays)


def _report(spec: TaskSpec, episode_count: int, aggregates: list[dict[str, Any]]) -> str:
    rows = []
    for aggregate in aggregates:
        semantic = aggregate["spec"]
        rows.append(
            f"| {semantic.segment_id} | `{semantic.parent_subgoal}` | `{semantic.semantic_label}` | "
            f"{semantic.gripper_semantics} | {semantic.manipulated_object or 'none'} | "
            f"{aggregate['path_length_mm']['median']:.1f} | "
            f"{aggregate['phase_residual_mm']['p90_mean']:.1f} | {semantic.confidence} |"
        )
    events = "\n".join(
        f"- {event}: {spec.events[event]['label_ko']}"
        for event in spec.event_sequence
    )
    states = "\n  → ".join(
        ", ".join(f"{key}={value}" for key, value in node.semantic_state.items())
        for node in spec.state_nodes
    )
    notes = "\n".join(f"- {note}" for note in spec.semantic_notes) or "- 없음"
    return f"""# {spec.task_id.upper()} semantic-only standardized graph V3

TASK_DESCRIPTION: `{spec.task_description}`  
Dataset: `{spec.dataset_root}`  
Associated model: `{spec.associated_model_path}`

## 설계 범위

{episode_count}개 demonstration을 semantic event로 정렬하고 각 구간을 Cartesian
arc-length phase `0..1`로 표준화했다. 특정 episode를 대표로 선택하지 않으며,
frame 번호 평균·구형 runtime 경계·전환 적합도·Bridge 계산을 포함하지 않는다.

![Semantic-only graph]({spec.task_id}_semantic_only_graph.png)

![Semantic hierarchy]({spec.task_id}_semantic_hierarchy.png)

## 계층적 semantic segment

| ID | 상위 subgoal | Semantic label | Gripper 의미 | 대상 | 대표 경로 길이 [mm] | phase 평균 p90 residual [mm] | 신뢰도 |
|---|---|---|---|---|---:|---:|---|
{chr(10).join(rows)}

## Semantic event grammar

`{' → '.join(spec.event_sequence)}`

{events}

각 event는 semantic anchor이며 정책 전환 지점을 의미하지 않는다.

## Semantic world state

```text
{states}
```

## Task별 주의사항

{notes}

## 대표 정보의 정의

- 굵은 색 경로: 전체 궤적의 동일 semantic phase에서 계산한 component median
- 흐린 색 경로: 개별 episode 분포
- 실제 대표 episode, 영상·이미지·frame 번호 평균 없음

## 포함하지 않은 판단

- 전환 적합도와 전환 phase
- Bridge 비용과 구형 경계
- 상위 planner의 task 조합 결정
"""


def _checksums(output_dir: Path) -> None:
    lines = []
    for path in sorted(output_dir.iterdir()):
        if path.is_file() and path.name != "checksums.sha256":
            lines.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}")
    (output_dir / "checksums.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze_task(
    spec: TaskSpec,
    output_dir: Path,
    phase_points: int,
) -> dict[str, Any]:
    if not spec.dataset_root.exists():
        raise FileNotFoundError(spec.dataset_root)
    if not spec.associated_model_path.exists():
        raise FileNotFoundError(spec.associated_model_path)
    source = spec.dataset_root.resolve()
    target = output_dir.resolve()
    if source == target or source in target.parents:
        raise ValueError("output directory must not be inside the source dataset")

    trajectories, info = load_lerobot_trajectories(spec.dataset_root, spec.task_id)
    if info.get("task_descriptions") != [spec.task_description]:
        raise ValueError(
            f"{spec.task_id} TASK_DESCRIPTION differs from semantic spec: "
            f"{info.get('task_descriptions')}"
        )
    episode_events = [
        detect_gripper_event_sequence(trajectory, spec.event_sequence)
        for trajectory in trajectories
    ]
    aggregates = []
    for semantic in spec.segments:
        standardized = []
        for trajectory, events in zip(trajectories, episode_events):
            start = resolve_anchor_index(semantic.start_anchor, trajectory, events)
            end = resolve_anchor_index(semantic.end_anchor, trajectory, events)
            standardized.append(
                resample_cartesian_span(trajectory, start, end, phase_points)
            )
        aggregates.append(aggregate_standardized_segment(semantic, standardized))

    event_centers = _event_centers(
        trajectories,
        episode_events,
        spec.event_sequence,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": "a0509.semantic_only_graph.v3",
        "task_id": spec.task_id,
        "dataset_repo_id": spec.dataset_repo_id,
        "dataset_root": str(spec.dataset_root.resolve()),
        "associated_model_path": str(spec.associated_model_path.resolve()),
        "model_used_for_semantic_extraction": False,
        "task_description": spec.task_description,
        "task_semantic_label": spec.task_semantic_label,
        "episode_count": len(trajectories),
        "event_grammar": list(spec.event_sequence),
        "events": {
            event: {
                **spec.events[event],
                "representative_position_mm": event_centers[event],
            }
            for event in spec.event_sequence
        },
        "hierarchy": {
            "task": spec.task_semantic_label,
            "subgoals": [parent for _, _, parent in _parent_runs(spec)],
        },
        "state_nodes": spec.state_nodes,
        "semantic_notes": list(spec.semantic_notes),
        "segments": [
            {
                "spec": aggregate["spec"],
                "path_length_mm": aggregate["path_length_mm"],
                "phase_residual_mm": aggregate["phase_residual_mm"],
            }
            for aggregate in aggregates
        ],
        "representative": {
            "synthetic": "component median XYZ at each semantic phase",
            "individual_episode_selected": False,
        },
        "standardization": {
            "axis": "per-segment Cartesian arc-length phase 0..1",
            "coordinate_frame": "Doosan base",
            "units": "mm",
            "phase_points": phase_points,
            "frame_number_average_used": False,
            "image_pixel_average_used": False,
        },
        "scope": {
            "semantic_information_only": True,
            "transition_fitness_included": False,
            "spherical_boundaries_included": False,
            "bridge_planning_included": False,
            "robot_executable": False,
            "original_dataset_modified": False,
            "model_modified": False,
        },
    }

    _write_tables(output_dir, spec, aggregates, event_centers)
    _write_npz(output_dir, aggregates)
    (output_dir / "semantic_graph_v3.json").write_text(
        json.dumps(_jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _plot_main_graph(output_dir, spec, aggregates, event_centers)
    _plot_hierarchy(output_dir, spec)
    (output_dir / "README.md").write_text(
        _report(spec, len(trajectories), aggregates),
        encoding="utf-8",
    )
    _checksums(output_dir)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--tasks",
        nargs="+",
        choices=tuple(TASKS),
        default=list(TASKS),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("docs/artifacts"),
    )
    parser.add_argument("--phase-points", type=int, default=101)
    parser.add_argument("--date", default="2026-08-23")
    args = parser.parse_args()
    if args.phase_points < 3:
        parser.error("--phase-points must be at least 3")

    summaries = []
    for task_id in args.tasks:
        spec = TASKS[task_id]
        output_dir = (
            args.output_root
            / f"{task_id}_semantic_only_graph_v3_{args.date}"
        )
        manifest = analyze_task(spec, output_dir, args.phase_points)
        summaries.append(
            {
                "task_id": task_id,
                "output_dir": str(output_dir.resolve()),
                "episodes": manifest["episode_count"],
                "segments": len(manifest["segments"]),
                "event_grammar": manifest["event_grammar"],
                "individual_episode_selected": False,
                "semantic_only": True,
            }
        )
    print(json.dumps(summaries, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
