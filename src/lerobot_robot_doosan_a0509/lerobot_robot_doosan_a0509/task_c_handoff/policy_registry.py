"""Unique-policy residency and repeated stage-visit accounting."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Mapping


@dataclass
class RegisteredPolicy:
    policy_id: str
    checkpoint: str
    session: Any
    reuse_context_policy: bool
    visit_count: int = 0
    active_visit_id: int | None = None


class PolicyRegistry:
    """Own one session per unique policy while queues remain visit-scoped."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._policies: dict[str, RegisteredPolicy] = {}
        self._active_policy_id: str | None = None
        self._closed = False

    def register(
        self,
        *,
        policy_id: str,
        checkpoint: str,
        session: Any,
        reuse_context_policy: bool,
    ) -> None:
        with self._lock:
            if self._closed:
                raise RuntimeError("policy registry is closed")
            if not policy_id:
                raise ValueError("policy_id must not be empty")
            if policy_id in self._policies:
                raise ValueError(f"policy already registered: {policy_id}")
            if any(item.session is session for item in self._policies.values()):
                raise ValueError("one policy session cannot be registered twice")
            self._policies[policy_id] = RegisteredPolicy(
                policy_id=policy_id,
                checkpoint=str(checkpoint),
                session=session,
                reuse_context_policy=bool(reuse_context_policy),
            )

    def policy(self, policy_id: str) -> RegisteredPolicy:
        with self._lock:
            try:
                return self._policies[policy_id]
            except KeyError as exc:
                raise KeyError(f"unregistered policy: {policy_id}") from exc

    def session(self, policy_id: str) -> Any:
        return self.policy(policy_id).session

    @property
    def active_policy_id(self) -> str | None:
        with self._lock:
            return self._active_policy_id

    def begin_visit(self, policy_id: str, *, invalidate_queue: bool) -> int:
        with self._lock:
            item = self.policy(policy_id)
            if invalidate_queue:
                item.session.deactivate_and_clear()
            item.visit_count += 1
            item.active_visit_id = item.visit_count
            if self._active_policy_id is not None:
                previous = self._policies[self._active_policy_id]
                if previous is not item:
                    previous.active_visit_id = None
            self._active_policy_id = policy_id
            return item.visit_count

    def deactivate_active(self, *, invalidate_queue: bool = True) -> int | None:
        with self._lock:
            policy_id = self._active_policy_id
            if policy_id is None:
                return None
            item = self._policies[policy_id]
            generation = None
            if invalidate_queue:
                generation = int(item.session.deactivate_and_clear())
            item.active_visit_id = None
            self._active_policy_id = None
            return generation

    def record(self) -> Mapping[str, Any]:
        with self._lock:
            return {
                "active_policy_id": self._active_policy_id,
                "policies": {
                    policy_id: {
                        "checkpoint": item.checkpoint,
                        "reuse_context_policy": item.reuse_context_policy,
                        "visit_count": item.visit_count,
                        "active_visit_id": item.active_visit_id,
                        "session_generation": getattr(item.session, "generation", None),
                        "active_generation": getattr(
                            item.session, "active_generation", None
                        ),
                        "queue_size": getattr(item.session, "queue_size", None),
                    }
                    for policy_id, item in self._policies.items()
                },
            }

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            sessions = [item.session for item in self._policies.values()]
        errors: list[BaseException] = []
        for session in sessions:
            try:
                session.close()
            except BaseException as exc:  # pragma: no cover - defensive cleanup
                errors.append(exc)
        if errors:
            detail = "; ".join(
                f"{type(error).__name__}: {error}" for error in errors
            )
            raise RuntimeError(f"policy registry close failed: {detail}") from errors[0]
