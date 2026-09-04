"""Episode-level multi-stage orchestration for repeated Task-C V2 handoffs.

This module deliberately owns no ROS publishers, robot commands, cameras, or
CUDA work. It turns a planner-approved sequence such as T4 -> T2 -> T4 into
stage visits and transition edges. A live strategy can then execute each edge
with the existing non-blocking V2 coordinator.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Mapping

from .models import EpisodeHandoffManifest
from .runtime_command_profile import get_runtime_command_profile
from .stage_supervisor import StagePhaseSupervisorConfig


MULTI_STAGE_SCHEMA_VERSION = "a0509.task_c_multi_stage_plan.v1"


def _load_mapping(path: Path) -> Mapping[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        value = json.loads(text)
    else:
        try:
            import yaml
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("PyYAML is required for YAML multi-stage plans") from exc
        value = yaml.safe_load(text)
    if not isinstance(value, Mapping):
        raise ValueError("multi-stage plan must contain a mapping")
    return value


def _resolved_from(base: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


@dataclass(frozen=True)
class MultiPolicySpec:
    policy_id: str
    checkpoint: Path
    reuse_context_policy: bool = False

    @classmethod
    def from_mapping(
        cls,
        policy_id: str,
        value: Mapping[str, Any],
        *,
        base_dir: Path,
    ) -> "MultiPolicySpec":
        if not policy_id:
            raise ValueError("policy_id must not be empty")
        checkpoint = str(value.get("checkpoint", "")).strip()
        if not checkpoint:
            raise ValueError(f"policy {policy_id} requires checkpoint")
        reuse = value.get("reuse_context_policy", False)
        if not isinstance(reuse, bool):
            raise ValueError("reuse_context_policy must be boolean")
        return cls(
            policy_id=policy_id,
            checkpoint=_resolved_from(base_dir, checkpoint),
            reuse_context_policy=reuse,
        )


@dataclass(frozen=True)
class MultiStageSpec:
    stage_id: str
    policy_id: str
    exit_authority: str
    terminal: bool = False
    phase_supervisor: StagePhaseSupervisorConfig | None = None
    inference_end_steps: int | None = None

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
        *,
        base_dir: Path,
    ) -> "MultiStageSpec":
        stage_id = str(value.get("stage_id", "")).strip()
        policy_id = str(value.get("policy_id", "")).strip()
        terminal = value.get("terminal", False)
        exit_authority = str(
            value.get(
                "exit_authority",
                "external_complete" if terminal else "external_planner",
            )
        ).strip()
        if not stage_id or not policy_id:
            raise ValueError("each stage requires stage_id and policy_id")
        if exit_authority not in {
            "external_planner",
            "phase_supervisor",
            "external_complete",
            "policy_inference_end",
        }:
            raise ValueError(
                "stage exit_authority must be external_planner, phase_supervisor, "
                "external_complete, or policy_inference_end"
            )
        if not isinstance(terminal, bool):
            raise ValueError("stage terminal must be boolean")
        raw_supervisor = value.get("phase_supervisor")
        supervisor = (
            None
            if raw_supervisor is None
            else StagePhaseSupervisorConfig.from_mapping(
                dict(raw_supervisor),
                base_dir=base_dir,
            )
        )
        raw_end_steps = value.get("inference_end_steps")
        inference_end_steps = (
            None if raw_end_steps is None else int(raw_end_steps)
        )
        if inference_end_steps is not None and (
            isinstance(raw_end_steps, bool) or inference_end_steps < 1
        ):
            raise ValueError("inference_end_steps must be a positive integer")
        if exit_authority == "phase_supervisor" and supervisor is None:
            raise ValueError(
                f"stage {stage_id} phase_supervisor authority requires "
                "phase_supervisor config"
            )
        if exit_authority != "phase_supervisor" and supervisor is not None:
            raise ValueError(
                f"stage {stage_id} phase_supervisor config requires "
                "phase_supervisor authority"
            )
        if (
            exit_authority == "policy_inference_end"
            and inference_end_steps is None
        ):
            raise ValueError(
                f"stage {stage_id} policy_inference_end requires "
                "inference_end_steps"
            )
        if (
            exit_authority != "policy_inference_end"
            and inference_end_steps is not None
        ):
            raise ValueError(
                f"stage {stage_id} inference_end_steps requires "
                "policy_inference_end authority"
            )
        return cls(
            stage_id,
            policy_id,
            exit_authority,
            terminal,
            supervisor,
            inference_end_steps,
        )


@dataclass(frozen=True)
class MultiTransitionSpec:
    transition_id: str
    source_stage: str
    successor_stage: str
    handoff_manifest_path: Path
    handoff_manifest: EpisodeHandoffManifest

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
        *,
        base_dir: Path,
    ) -> "MultiTransitionSpec":
        transition_id = str(value.get("transition_id", "")).strip()
        source_stage = str(value.get("source_stage", "")).strip()
        successor_stage = str(value.get("successor_stage", "")).strip()
        manifest_value = str(value.get("handoff_manifest", "")).strip()
        if not transition_id or not source_stage or not successor_stage:
            raise ValueError(
                "each transition requires transition_id/source_stage/successor_stage"
            )
        if not manifest_value:
            raise ValueError(f"transition {transition_id} requires handoff_manifest")
        manifest_path = _resolved_from(base_dir, manifest_value)
        return cls(
            transition_id=transition_id,
            source_stage=source_stage,
            successor_stage=successor_stage,
            handoff_manifest_path=manifest_path,
            handoff_manifest=EpisodeHandoffManifest.load(manifest_path),
        )


@dataclass(frozen=True)
class MultiStagePlan:
    composition_id: str
    policies: tuple[MultiPolicySpec, ...]
    stages: tuple[MultiStageSpec, ...]
    transitions: tuple[MultiTransitionSpec, ...]
    source_path: Path
    runtime_command_profile_id: str | None = None
    schema_version: str = MULTI_STAGE_SCHEMA_VERSION

    @classmethod
    def load(cls, path: str | Path) -> "MultiStagePlan":
        source_path = Path(path).expanduser().resolve()
        if not source_path.is_file():
            raise FileNotFoundError(f"multi-stage plan not found: {source_path}")
        value = _load_mapping(source_path)
        schema = str(value.get("schema_version", MULTI_STAGE_SCHEMA_VERSION))
        if schema != MULTI_STAGE_SCHEMA_VERSION:
            raise ValueError(f"unsupported multi-stage schema: {schema}")
        composition_id = str(value.get("composition_id", "")).strip()
        if not composition_id:
            raise ValueError("multi-stage plan requires composition_id")
        raw_profile_id = value.get("runtime_command_profile_id")
        runtime_command_profile_id = (
            None if raw_profile_id is None else str(raw_profile_id).strip()
        )
        if runtime_command_profile_id == "":
            raise ValueError("runtime_command_profile_id must not be empty")
        if runtime_command_profile_id is not None:
            get_runtime_command_profile(runtime_command_profile_id)
        raw_policies = value.get("policies")
        raw_stages = value.get("stages")
        raw_transitions = value.get("transitions")
        if not isinstance(raw_policies, Mapping) or not raw_policies:
            raise ValueError("multi-stage plan requires a non-empty policies mapping")
        if not isinstance(raw_stages, list) or len(raw_stages) < 2:
            raise ValueError("multi-stage plan requires at least two stages")
        if not isinstance(raw_transitions, list):
            raise ValueError("multi-stage plan transitions must be a list")
        base_dir = source_path.parent
        policies = tuple(
            MultiPolicySpec.from_mapping(
                str(policy_id), dict(policy_value), base_dir=base_dir
            )
            for policy_id, policy_value in raw_policies.items()
        )
        stages = tuple(
            MultiStageSpec.from_mapping(dict(item), base_dir=base_dir)
            for item in raw_stages
        )
        transitions = tuple(
            MultiTransitionSpec.from_mapping(dict(item), base_dir=base_dir)
            for item in raw_transitions
        )
        plan = cls(
            composition_id=composition_id,
            policies=policies,
            stages=stages,
            transitions=transitions,
            source_path=source_path,
            runtime_command_profile_id=runtime_command_profile_id,
            schema_version=schema,
        )
        plan._validate()
        return plan

    def _validate(self) -> None:
        policy_ids = [item.policy_id for item in self.policies]
        stage_ids = [item.stage_id for item in self.stages]
        transition_ids = [item.transition_id for item in self.transitions]
        for name, values in (
            ("policy", policy_ids),
            ("stage", stage_ids),
            ("transition", transition_ids),
        ):
            if len(values) != len(set(values)):
                raise ValueError(f"multi-stage {name} identifiers must be unique")
        policy_set = set(policy_ids)
        if any(stage.policy_id not in policy_set for stage in self.stages):
            raise ValueError("every stage policy_id must exist in policies")
        context_policies = [
            item.policy_id for item in self.policies if item.reuse_context_policy
        ]
        if context_policies != [self.stages[0].policy_id]:
            raise ValueError(
                "exactly the first-stage policy must set reuse_context_policy=true"
            )
        if len(self.transitions) != len(self.stages) - 1:
            raise ValueError("multi-stage plan requires exactly one edge between stages")
        if any(stage.terminal for stage in self.stages[:-1]):
            raise ValueError("only the final stage may be terminal")
        if not self.stages[-1].terminal:
            raise ValueError("the final stage must be terminal")
        if self.stages[-1].exit_authority not in {
            "external_complete",
            "policy_inference_end",
        }:
            raise ValueError(
                "the final stage requires external_complete or "
                "policy_inference_end authority"
            )
        if any(
            stage.exit_authority not in {"external_planner", "phase_supervisor"}
            for stage in self.stages[:-1]
        ):
            raise ValueError(
                "every non-final stage requires external_planner or "
                "phase_supervisor authority"
            )
        policy_by_stage = {stage.stage_id: stage.policy_id for stage in self.stages}
        for index, transition in enumerate(self.transitions):
            source = self.stages[index]
            successor = self.stages[index + 1]
            if (
                transition.source_stage != source.stage_id
                or transition.successor_stage != successor.stage_id
            ):
                raise ValueError("transitions must connect consecutive ordered stages")
            manifest = transition.handoff_manifest
            if manifest.source.task != policy_by_stage[transition.source_stage]:
                raise ValueError(
                    f"transition {transition.transition_id} source task/policy mismatch"
                )
            if manifest.successor.task != policy_by_stage[transition.successor_stage]:
                raise ValueError(
                    f"transition {transition.transition_id} successor task/policy mismatch"
                )

    @property
    def initial_policy(self) -> MultiPolicySpec:
        return self.policy(self.stages[0].policy_id)

    def policy(self, policy_id: str) -> MultiPolicySpec:
        for item in self.policies:
            if item.policy_id == policy_id:
                return item
        raise KeyError(policy_id)

    def transition_after(self, stage_index: int) -> MultiTransitionSpec | None:
        if not 0 <= stage_index < len(self.stages):
            raise IndexError(stage_index)
        if stage_index >= len(self.transitions):
            return None
        return self.transitions[stage_index]


class MultiStageRuntimeState(str, Enum):
    WAITING = "WAITING"
    RUN_STAGE = "RUN_STAGE"
    TRANSITION_REQUESTED = "TRANSITION_REQUESTED"
    TRANSITION_ACTIVE = "TRANSITION_ACTIVE"
    COMPLETE = "COMPLETE"
    FAILED_HOLD = "FAILED_HOLD"


@dataclass(frozen=True)
class MultiStageEvent:
    timestamp_s: float
    event: str
    state: str
    stage_index: int
    stage_id: str
    policy_id: str
    visit_id: int
    transition_id: str | None = None
    details: Mapping[str, Any] | None = None


class MultiStageCoordinator:
    """Thread-safe planner mailbox and stage/edge lifecycle state."""

    def __init__(
        self,
        plan: MultiStagePlan,
        *,
        event_callback: Callable[[MultiStageEvent], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.plan = plan
        self.event_callback = event_callback
        self.clock = clock
        self._lock = threading.RLock()
        self.state = MultiStageRuntimeState.WAITING
        self.stage_index = 0
        self.visit_id = 0
        self._requested_transition: MultiTransitionSpec | None = None
        self._active_transition: MultiTransitionSpec | None = None
        self.events: list[MultiStageEvent] = []

    @property
    def current_stage(self) -> MultiStageSpec:
        with self._lock:
            return self.plan.stages[self.stage_index]

    @property
    def requested_transition(self) -> MultiTransitionSpec | None:
        with self._lock:
            return self._requested_transition

    @property
    def active_transition(self) -> MultiTransitionSpec | None:
        with self._lock:
            return self._active_transition

    def _emit(
        self,
        event: str,
        *,
        transition_id: str | None = None,
        **details: Any,
    ) -> None:
        stage = self.plan.stages[self.stage_index]
        value = MultiStageEvent(
            timestamp_s=float(self.clock()),
            event=event,
            state=self.state.value,
            stage_index=self.stage_index,
            stage_id=stage.stage_id,
            policy_id=stage.policy_id,
            visit_id=self.visit_id,
            transition_id=transition_id,
            details=details or None,
        )
        self.events.append(value)
        if self.event_callback is not None:
            self.event_callback(value)

    def start(self) -> None:
        with self._lock:
            if self.state is not MultiStageRuntimeState.WAITING:
                raise RuntimeError("multi-stage coordinator already started")
            self.state = MultiStageRuntimeState.RUN_STAGE
            self._emit("stage_started")

    def try_request_next(
        self,
        *,
        expected_stage_id: str | None = None,
    ) -> tuple[bool, str]:
        with self._lock:
            if self.state is not MultiStageRuntimeState.RUN_STAGE:
                return False, f"cannot request transition from {self.state.value}"
            stage = self.plan.stages[self.stage_index]
            if expected_stage_id is not None and expected_stage_id != stage.stage_id:
                return False, f"active stage is {stage.stage_id}"
            transition = self.plan.transition_after(self.stage_index)
            if transition is None:
                return False, "final stage requires complete request"
            self._requested_transition = transition
            self.state = MultiStageRuntimeState.TRANSITION_REQUESTED
            self._emit(
                "transition_requested",
                transition_id=transition.transition_id,
                successor_stage=transition.successor_stage,
            )
            return True, transition.transition_id

    def begin_requested_transition(self) -> MultiTransitionSpec:
        with self._lock:
            if self.state is not MultiStageRuntimeState.TRANSITION_REQUESTED:
                raise RuntimeError("no requested multi-stage transition")
            transition = self._requested_transition
            if transition is None:
                raise RuntimeError("requested transition metadata is missing")
            self._requested_transition = None
            self._active_transition = transition
            self.state = MultiStageRuntimeState.TRANSITION_ACTIVE
            self._emit(
                "transition_started",
                transition_id=transition.transition_id,
                successor_stage=transition.successor_stage,
            )
            return transition

    def complete_takeover(
        self,
        *,
        transition_id: str,
        policy_generation: int,
    ) -> MultiStageSpec:
        with self._lock:
            transition = self._active_transition
            if (
                self.state is not MultiStageRuntimeState.TRANSITION_ACTIVE
                or transition is None
            ):
                raise RuntimeError("multi-stage takeover has no active transition")
            if transition.transition_id != transition_id:
                raise RuntimeError("multi-stage takeover transition mismatch")
            self.stage_index += 1
            self.visit_id += 1
            self._active_transition = None
            self.state = MultiStageRuntimeState.RUN_STAGE
            self._emit(
                "stage_takeover_complete",
                transition_id=transition_id,
                policy_generation=int(policy_generation),
            )
            return self.plan.stages[self.stage_index]

    def try_complete(
        self,
        *,
        expected_stage_id: str | None = None,
    ) -> tuple[bool, str]:
        with self._lock:
            if self.state is not MultiStageRuntimeState.RUN_STAGE:
                return False, f"cannot complete from {self.state.value}"
            stage = self.plan.stages[self.stage_index]
            if expected_stage_id is not None and expected_stage_id != stage.stage_id:
                return False, f"active stage is {stage.stage_id}"
            if not stage.terminal:
                return False, "only the final stage may complete the episode"
            self.state = MultiStageRuntimeState.COMPLETE
            self._emit("episode_complete")
            return True, stage.stage_id

    def fail_closed(self, reason: str) -> None:
        with self._lock:
            if self.state is MultiStageRuntimeState.FAILED_HOLD:
                return
            self.state = MultiStageRuntimeState.FAILED_HOLD
            self._emit("multi_stage_fail_closed", reason=str(reason))

    def record(self) -> dict[str, Any]:
        with self._lock:
            stage = self.plan.stages[self.stage_index]
            return {
                "composition_id": self.plan.composition_id,
                "state": self.state.value,
                "stage_index": self.stage_index,
                "stage_id": stage.stage_id,
                "policy_id": stage.policy_id,
                "visit_id": self.visit_id,
                "requested_transition_id": (
                    None
                    if self._requested_transition is None
                    else self._requested_transition.transition_id
                ),
                "active_transition_id": (
                    None
                    if self._active_transition is None
                    else self._active_transition.transition_id
                ),
            }
