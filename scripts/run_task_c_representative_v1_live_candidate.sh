#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${RUNTIME_MANIFEST:-}" ]]; then
    echo "ERROR: set RUNTIME_MANIFEST to a reviewed V1 runtime_transition_manifest.json" >&2
    exit 1
fi
if [[ ! -f "${RUNTIME_MANIFEST}" ]]; then
    echo "ERROR: V1 manifest not found: ${RUNTIME_MANIFEST}" >&2
    exit 1
fi

export REPRESENTATIVE_BOUNDARY_ENABLED=true
export MOVING_OVERLAP_PRIMARY_ENABLED=false
export STOPPED_ENDPOINT_DIRECT_HANDOFF_ENABLED=true
export STOPPED_ENDPOINT_POSITION_BRIDGE_ENABLED=true
exec "${project_root}/scripts/run_task_c_live_candidate.sh"
