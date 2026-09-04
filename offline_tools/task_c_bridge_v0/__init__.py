"""Offline Task-C bridge design tools.

This package is deliberately independent of ROS and policy rollout code.  It
does not publish commands and it does not import the LeRobot runtime.
"""

from .bezier_bridge import (
    CUBIC_BEZIER_FIXED,
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
    CubicBezierBridge,
    TangentRegularizationDiagnostics,
    build_tangent_regularized_bezier,
    build_velocity_matched_bezier,
)
from .bridge_optimizer import BridgeOptimizer, NoFeasibleBridgeError
from .trajectory_states import CandidatePoint, SemanticState, Trajectory

from .runtime_bridge import RuntimeBEntry, RuntimeBridgePlanner
from .runtime_orchestrator import (
    RuntimeHandoffConfig,
    RuntimeObservation,
    RuntimePhase,
    TaskCRealtimeCoordinator,
)

from .runtime_policy import AsyncPolicySession, PolicyChunk, assess_policy_chunk
__all__ = [
    "BridgeOptimizer",
    "CandidatePoint",
    "AsyncPolicySession",
    "PolicyChunk",
    "RuntimeBEntry",
    "RuntimeBridgePlanner",
    "RuntimeHandoffConfig",
    "RuntimeObservation",
    "RuntimePhase",
    "TaskCRealtimeCoordinator",
    "assess_policy_chunk",
    "CubicBezierBridge",
    "CUBIC_BEZIER_FIXED",
    "CUBIC_BEZIER_TANGENT_REGULARIZED_V1",
    "NoFeasibleBridgeError",
    "SemanticState",
    "Trajectory",
    "TangentRegularizationDiagnostics",
    "build_tangent_regularized_bezier",
    "build_velocity_matched_bezier",
]
