"""Opt-in Task-C asynchronous handoff V2 primitives.

The package is deliberately ROS-independent.  It owns episode manifest
validation, one-shot Bridge instantiation, non-blocking ACT-B result routing,
prefix admission, soft handoff math, and machine-readable tracing.  The
existing V0/V1 live strategy remains the safety fallback and owns all ROS
command publication.
"""

from offline_tools.cross_task_handoff.authority import SemanticAuthority

from .bridge_runtime import (
    BridgeGenerationError,
    BridgeRuntimeLimits,
    BridgeRuntimeSnapshot,
    PrecomputedBridgeQueue,
    instantiate_bridge_queue,
)
from .compatibility import (
    HandoffExecutionDynamics,
    HandoffCompatibilityEvaluator,
    HandoffSpliceSelector,
    PrefixAdmission,
    PrefixDynamics,
)
from .coordinator import (
    HandoffCommand,
    TaskCHandoffV2Coordinator,
    V2CoordinatorEvent,
)
from .models import (
    BoundarySemanticState,
    EpisodeHandoffManifest,
    HandoffBoundary,
    HandoffCompatibilityConfig,
    HandoffMode,
    HandoffV2Config,
    HandoffV2State,
)
from .soft_handoff import (
    SoftHandoffPlan,
    build_soft_handoff,
    quintic_smoothstep,
)
from .source_phase import (
    SourcePhaseStatus,
    SourcePhaseSupportBank,
    SourcePhaseSupportConfig,
    SourcePhaseSupportTracker,
    SourceTriggerMode,
    clipped_phase_window,
)

__all__ = [
    "BoundarySemanticState",
    "BridgeGenerationError",
    "BridgeRuntimeLimits",
    "BridgeRuntimeSnapshot",
    "EpisodeHandoffManifest",
    "HandoffBoundary",
    "HandoffCommand",
    "HandoffCompatibilityConfig",
    "HandoffCompatibilityEvaluator",
    "HandoffExecutionDynamics",
    "HandoffMode",
    "HandoffSpliceSelector",
    "HandoffV2Config",
    "HandoffV2State",
    "PrefixAdmission",
    "SemanticAuthority",
    "PrefixDynamics",
    "PrecomputedBridgeQueue",
    "SoftHandoffPlan",
    "SourcePhaseStatus",
    "SourcePhaseSupportBank",
    "SourcePhaseSupportConfig",
    "SourcePhaseSupportTracker",
    "SourceTriggerMode",
    "TaskCHandoffV2Coordinator",
    "V2CoordinatorEvent",
    "build_soft_handoff",
    "clipped_phase_window",
    "instantiate_bridge_queue",
    "quintic_smoothstep",
]
