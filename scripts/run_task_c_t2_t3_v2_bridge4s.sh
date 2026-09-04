#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "${HOME}/venvs/lerobot/bin/activate"
source "${project_root}/install/setup.bash"
set -u

export CHECKPOINT_A="${CHECKPOINT_A:-${HOME}/lerobot_models/models/act_a0509_blue_block_t2_table_bs32_40k_20260821/040000/pretrained_model}"
export CHECKPOINT_B="${CHECKPOINT_B:-${HOME}/lerobot_models/models/act_a0509_blue_block_t3_bs32_40k_20260822/040000/pretrained_model}"
export RUNTIME_MANIFEST="${RUNTIME_MANIFEST:-${project_root}/docs/artifacts/task_c_t2_t3_floor_to_floor_v2_2026-08-25/v2_fail_closed_support_runtime_manifest_t2_t3.json}"
export HANDOFF_EPISODE_MANIFEST="${HANDOFF_EPISODE_MANIFEST:-${project_root}/docs/artifacts/task_c_t2_t3_floor_to_floor_v2_2026-08-25/episode_manifests/h_d4ebd8858ac1_bridge4s.json}"
export SOURCE_TRIGGER_MODE="${SOURCE_TRIGGER_MODE:-phase_support}"
export SOURCE_PHASE_ARTIFACT="${SOURCE_PHASE_ARTIFACT:-${project_root}/docs/artifacts/t2_semantic_only_graph_v3_2026-08-23/standardized_semantic_phase_trajectories.npz}"
export B_RELEASE_POSITION_GATE_ENABLED="${B_RELEASE_POSITION_GATE_ENABLED:-false}"
export B_COMPLETION_MODE="${B_COMPLETION_MODE:-successor_owned}"
export B_COMPLETION_POSITION_GATE_ENABLED="${B_COMPLETION_POSITION_GATE_ENABLED:-false}"
export MAX_CROSSFADE_COMMAND_ACCELERATION_MM_S2="${MAX_CROSSFADE_COMMAND_ACCELERATION_MM_S2:-4000}"
export ENABLE_ENDPOINT_FALLBACK="${ENABLE_ENDPOINT_FALLBACK:-true}"
export TRANSITION_TIMEOUT_S="${TRANSITION_TIMEOUT_S:-12}"
export BRIDGE_ACK_MODE="${BRIDGE_ACK_MODE:-bounded_pipeline}"
export BRIDGE_MAX_ACK_LAG_STEPS="${BRIDGE_MAX_ACK_LAG_STEPS:-1}"

exec "${project_root}/scripts/run_task_c_async_handoff_v2_candidate.sh" "$@"
