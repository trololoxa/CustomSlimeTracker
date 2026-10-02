#!/usr/bin/env python3
"""Guard the 0021a storage stack budget and non-recursive migration path."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, function_body, parse_stack_usage, require_limit, run_contract_command,
)


from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    store = (ROOT / "src/config/tracker_config_store.cpp").read_text(encoding="utf-8")
    resolve = function_body(store, "bool TrackerConfigStore::resolveActive")
    load = function_body(store, "bool TrackerConfigStore::load")
    if "migrateLegacy" in resolve:
        raise SystemExit("resolveActive must not invoke legacy migration recursively")
    if "if (!migrateLegacy())" not in load:
        raise SystemExit("load must perform legacy migration after resolveActive unwinds")
    if "makeScratch<ResolveActiveScratch>" not in resolve:
        raise SystemExit("resolveActive must keep dual-slot scratch off the task stack")
    if "uint8_t scratch[sizeof(TrackerCalibrationCandidateRecord)]" in (
        ROOT / "src/config/tracker_config_storage.cpp"
    ).read_text(encoding="utf-8"):
        raise SystemExit("CRC calculation must not allocate a candidate-sized stack buffer")

    prepare = function_body(store, "bool TrackerConfigStore::prepareCandidatePromotion")
    if "trackerComposeCalibrationCandidate" in prepare:
        raise SystemExit(
            "prepareCandidatePromotion must compose in-place; returning TrackerConfig by value "
            "creates an ABI-dependent candidate-sized stack temporary"
        )
    if "trackerApplyCalibrationCandidateToConfig(composed, candidateSnapshot)" not in prepare:
        raise SystemExit("prepareCandidatePromotion must use heap-backed in-place composition")

    with project_temp_directory(ROOT, "tracker-stack-policy-") as tmp:
        tmp_path = Path(tmp)
        cxx = compiler()
        common = [
            cxx,
            "-std=c++17",
            "-O2",
            "-fstack-usage",
            "-I",
            str(ROOT / "src"),
            "-I",
            str(ROOT / "tests/native"),
            "-c",
        ]
        for source in (
            ROOT / "src/config/tracker_config_storage.cpp",
            ROOT / "src/config/tracker_config_store.cpp",
            ROOT / "src/serial/tracker_calibration_commands.cpp",
            ROOT / "src/serial/tracker_fifo_config_control.cpp",
        ):
            output = tmp_path / (source.stem + ".o")
            run_contract_command(common + [str(source), "-o", str(output)], check=True)
        usage = parse_stack_usage(tmp_path)

    for function in (
        "TrackerConfigStore::resolveActive",
        "TrackerConfigStore::saveInternal",
        "TrackerConfigStore::inspectStorage",
        "TrackerConfigStore::stageCandidate",
        "TrackerConfigStore::flushCandidate",
        "TrackerConfigStore::load(",
        "TrackerConfigStore::migrateLegacy",
        "TrackerCalibrationCommandDispatcher::cmdCalCandidate",
        "commitImuFifoCandidate",
        "trackerSerialCommitFullHardwareConfig",
    ):
        require_limit(usage, function, 1024)
    # Keep promotion comfortably below the cross-ABI 1 KiB ceiling. 0022b
    # measured 1008 bytes on Linux/GCC but 1040 bytes on Windows/MSYS2 GCC.
    require_limit(usage, "TrackerConfigStore::prepareCandidatePromotion", 768)

    print("# calibration_storage_stack: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
