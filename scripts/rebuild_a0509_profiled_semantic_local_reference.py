#!/usr/bin/env python3
"""Build a semantic-local reference with a shared, versioned Bridge profile.

This is the canonical entry point for new execution-tail policies.  It wraps
the existing command-free semantic-local builder, selects a profile solely by
source-reference class, and records the resolved profile and analytical source
speed coverage beside every artifact.  It performs no ROS or robot I/O.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
for value in (str(REPOSITORY_ROOT), str(PACKAGE_ROOT)):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.task_c_handoff.bridge_profile import (  # noqa: E402
    apply_bridge_generation_profile,
    attach_bridge_profile_provenance,
    evaluate_source_bank_speed_coverage,
    load_bridge_generation_profile,
    validate_profile_against_validation_config,
)
from scripts.rebuild_a0509_t7_t3_execution_tail_reference import (  # noqa: E402
    DEFAULT_BASELINE,
    DEFAULT_OUTPUT,
    DEFAULT_VALIDATION,
    main as build_semantic_local_reference,
)


DEFAULT_PROFILE_CONFIG = (
    REPOSITORY_ROOT
    / "config/realtime/a0509_flexible_bridge_profiles_v1.json"
)


def _option(
    argv: list[str],
    name: str,
    default: str | Path | None,
) -> str | None:
    prefix = f"{name}="
    for index, item in enumerate(argv):
        if item.startswith(prefix):
            return item[len(prefix) :]
        if item == name:
            if index + 1 >= len(argv):
                raise ValueError(f"{name} requires a value")
            return argv[index + 1]
    return None if default is None else str(default)


def _without_wrapper_options(argv: list[str]) -> list[str]:
    result: list[str] = []
    skip = False
    for item in argv:
        if skip:
            skip = False
            continue
        if item in {"--bridge-profile-config", "--bridge-profile-id"}:
            skip = True
            continue
        if item.startswith("--bridge-profile-config=") or item.startswith(
            "--bridge-profile-id="
        ):
            continue
        result.append(item)
    return result


def _replace_option(argv: list[str], name: str, value: str) -> list[str]:
    result: list[str] = []
    skip = False
    found = False
    prefix = f"{name}="
    for item in argv:
        if skip:
            result.append(value)
            skip = False
            found = True
            continue
        if item == name:
            result.append(item)
            skip = True
            continue
        if item.startswith(prefix):
            result.append(f"{name}={value}")
            found = True
            continue
        result.append(item)
    if skip:
        raise ValueError(f"{name} requires a value")
    if not found:
        result.extend((name, value))
    return result


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _enrich_generated_artifacts(output_root: Path, profile: Any) -> None:
    manifest_path = output_root / "flexible_reference_manifest.json"
    manifest = _load(manifest_path)
    generation = dict(manifest["generation"])
    source_reference = dict(generation["runtime_source_reference"])
    successor_reference = dict(generation["runtime_successor_reference"])
    source_bank_path = output_root / str(source_reference["support_bank"])
    successor_bank_path = output_root / str(successor_reference["support_bank"])
    source_bank = _load(source_bank_path)
    successor_bank = _load(successor_bank_path)
    profile_record = profile.provenance_record()
    coverage = evaluate_source_bank_speed_coverage(source_bank, profile)

    _write(
        manifest_path,
        attach_bridge_profile_provenance(manifest, profile),
    )
    for path, bank in (
        (source_bank_path, source_bank),
        (successor_bank_path, successor_bank),
    ):
        selection = dict(bank.get("semantic_local_selection", {}))
        selection.update(
            {
                "bridge_generation_profile_id": profile.profile_id,
                "bridge_generation_profile_sha256": profile.profile_sha256,
                "edge_specific_bridge_tuning": profile.edge_specific_tuning,
            }
        )
        bank["semantic_local_selection"] = selection
        _write(path, bank)

    selection_path = output_root / "semantic_local_reference_selection.json"
    selection = _load(selection_path)
    selection["bridge_generation_profile"] = profile_record
    _write(selection_path, selection)

    geometry_path = output_root / "geometry_evaluation.json"
    geometry = _load(geometry_path)
    geometry["bridge_generation_profile"] = profile_record
    geometry["source_speed_coverage"] = str(
        output_root / "bridge_profile_source_speed_coverage.json"
    )
    _write(geometry_path, geometry)

    overlay_path = output_root / "level2_registry_overlay.json"
    overlay = _load(overlay_path)
    for edge in overlay["edges"]:
        evidence = dict(edge.get("evidence", {}))
        evidence.update(
            {
                "bridge_generation_profile_id": profile.profile_id,
                "bridge_generation_profile_sha256": profile.profile_sha256,
                "edge_specific_bridge_tuning": profile.edge_specific_tuning,
                "source_speed_coverage": (
                    "bridge_profile_source_speed_coverage.json"
                ),
            }
        )
        edge["evidence"] = evidence
    _write(overlay_path, overlay)
    _write(output_root / "bridge_generation_profile.json", profile_record)
    _write(
        output_root / "bridge_profile_source_speed_coverage.json",
        coverage,
    )


def main(argv: list[str] | None = None) -> None:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    if "--help" in raw_argv or "-h" in raw_argv:
        print(
            "Shared-profile options:\n"
            f"  --bridge-profile-config PATH  default={DEFAULT_PROFILE_CONFIG}\n"
            "  --bridge-profile-id ID          default=selected by "
            "--source-reference-mode\n"
            "All other options are forwarded to "
            "rebuild_a0509_t7_t3_execution_tail_reference.py.\n"
        )
        build_semantic_local_reference(["--help"])
        return

    source_reference_mode = str(
        _option(raw_argv, "--source-reference-mode", "execution_tail")
    )
    baseline_path = Path(
        str(_option(raw_argv, "--baseline-manifest", DEFAULT_BASELINE))
    ).expanduser().resolve()
    validation_path = Path(
        str(_option(raw_argv, "--validation-config", DEFAULT_VALIDATION))
    ).expanduser().resolve()
    output_root = Path(
        str(_option(raw_argv, "--output-root", DEFAULT_OUTPUT))
    ).expanduser().resolve()
    profile_config = Path(
        str(
            _option(
                raw_argv,
                "--bridge-profile-config",
                DEFAULT_PROFILE_CONFIG,
            )
        )
    ).expanduser().resolve()
    profile_id = _option(raw_argv, "--bridge-profile-id", None)
    profile = load_bridge_generation_profile(
        profile_config,
        source_reference_mode=source_reference_mode,
        profile_id=profile_id,
    )
    validation = _load(validation_path)
    validate_profile_against_validation_config(profile, validation)

    baseline = _load(baseline_path)
    profiled_baseline = apply_bridge_generation_profile(
        baseline,
        profile,
        source_reference_mode=source_reference_mode,
    )
    forwarded = _without_wrapper_options(raw_argv)
    with tempfile.TemporaryDirectory(
        prefix="a0509_profiled_bridge_"
    ) as temporary:
        temporary_manifest = Path(temporary) / "profiled_baseline.json"
        _write(temporary_manifest, profiled_baseline)
        forwarded = _replace_option(
            forwarded,
            "--baseline-manifest",
            str(temporary_manifest),
        )
        build_semantic_local_reference(forwarded)

    _enrich_generated_artifacts(output_root, profile)
    print(
        "PROFILED_SEMANTIC_LOCAL_REFERENCE_OK "
        f"profile={profile.profile_id} "
        f"profile_sha256={profile.profile_sha256} "
        f"output={output_root} robot_commands_published=0"
    )


if __name__ == "__main__":
    main()
