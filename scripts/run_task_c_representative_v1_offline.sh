#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dataset_a="${DATASET_A:-/home/rvlab/lerobot_datasets/a0509_blue_block_v1_20260810_170226}"
dataset_b="${DATASET_B:-/home/rvlab/lerobot_datasets/a0509_blue_block_v2_20260810_191044}"
config="${CONFIG:-${project_root}/offline_tools/task_c_bridge_v1/config_representative_closed_holding.json}"
run_stamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${OUTPUT_DIR:-/home/rvlab/lerobot_datasets/task_c_bridge_v1_representative_${run_stamp}}"
python_bin="${PYTHON_BIN:-/home/rvlab/venvs/lerobot/bin/python}"

for required_path in "${dataset_a}" "${dataset_b}" "${config}" "${python_bin}"; do
    if [[ ! -e "${required_path}" ]]; then
        echo "ERROR: required path not found: ${required_path}" >&2
        exit 1
    fi
done

cd "${project_root}"
exec "${python_bin}" -m offline_tools.task_c_bridge_v1.run_representative_dry_run \
    --dataset-a "${dataset_a}" \
    --dataset-b "${dataset_b}" \
    --config "${config}" \
    --output "${output_dir}"
