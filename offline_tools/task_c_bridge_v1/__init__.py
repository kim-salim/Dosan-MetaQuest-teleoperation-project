"""Representative-trajectory Task-C bridge planner (V1).

V1 is intentionally additive.  The audited exhaustive V0 planner remains
available under :mod:`offline_tools.task_c_bridge_v0` and existing manifests
continue to use it unchanged.
"""

from .runtime_boundary import (
    RepresentativeBoundaryContract,
    RepresentativeBoundaryStatus,
    RepresentativeBoundaryTracker,
)

__all__ = [
    "RepresentativeBoundaryContract",
    "RepresentativeBoundaryStatus",
    "RepresentativeBoundaryTracker",
]
