#!/usr/bin/env python3
"""Guard guided magnetometer coverage-reservoir and setup-flow hardening."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def forbid(text: str, needle: str, description: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {description}: {needle}")


def compiler() -> str:
    explicit = os.environ.get("CXX")
    if explicit and shutil.which(explicit):
        return explicit
    for name in ("g++", "clang++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    raise SystemExit("no C++ compiler available for 0023g stack policy")


def parse_stack_usage(directory: Path) -> dict[str, int]:
    usage: dict[str, int] = {}
    for path in directory.rglob("*.su"):
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            parts = line.split("\t")
            if len(parts) < 2:
                continue
            try:
                size = int(parts[1])
            except ValueError:
                continue
            name = parts[0].split(":", 3)[-1]
            usage[name] = max(size, usage.get(name, 0))
    return usage


def require_limit(usage: dict[str, int], needle: str, limit: int) -> None:
    matches = [(name, size) for name, size in usage.items() if needle in name]
    if not matches:
        raise SystemExit(f"0023g stack policy did not find function: {needle}")
    worst_name, worst_size = max(matches, key=lambda item: item[1])
    if worst_size > limit:
        raise SystemExit(
            f"0023g stack budget exceeded: {worst_name} uses {worst_size} bytes, limit {limit}"
        )


def main() -> int:
    helper = (ROOT / "src/core/deterministic_reservoir.hpp").read_text(encoding="utf-8")
    mag_h = (ROOT / "src/sensor/mag_calibration.hpp").read_text(encoding="utf-8")
    mag = (ROOT / "src/sensor/mag_calibration.cpp").read_text(encoding="utf-8")
    axis_h = (ROOT / "src/sensor/mag_axis_alignment.hpp").read_text(encoding="utf-8")
    setup = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    status = (ROOT / "src/runtime/mag_status_reporter.cpp").read_text(encoding="utf-8")
    controller = (ROOT / "src/runtime/mag_runtime_controller.cpp").read_text(encoding="utf-8")
    hooks = (ROOT / "src/app/hooks/tracker_app_mag_hooks.hpp").read_text(encoding="utf-8")
    mag_test = (ROOT / "tests/native/test_mag_calibration.cpp").read_text(encoding="utf-8")
    heading_test = (ROOT / "tests/native/test_mag_heading_reliability.cpp").read_text(encoding="utf-8")
    cli = (ROOT / "docs/cli_reference.md").read_text(encoding="utf-8")
    project = (ROOT / "docs/project_status.md").read_text(encoding="utf-8")
    testing = (ROOT / "docs/testing.md").read_text(encoding="utf-8")
    report = (ROOT / "docs/0023g_magnetometer_coverage_reservoir_hardening_report.md").read_text(encoding="utf-8")

    require(helper, "deterministicReservoirCandidate", "shared deterministic reservoir helper")
    require(mag, "deterministicReservoirCandidate(", "hard/soft Algorithm-R admission")
    require(mag, "reservoirReplacements_", "hard/soft replacement diagnostics")
    require(mag, "const MagCalibrationFitSetDiagnostics fitSet = fitSetDiagnostics();", "fit-set coverage gates")
    forbid(mag, "storedSequence_ * 2654435761", "recent-sample permutation ring")
    require(mag_h, "MagCalibrationFitSetDiagnostics", "fit-set diagnostics API")

    require(axis_h, "class MagAxisIntervalReservoir", "bounded guided-axis reservoir")
    require(axis_h, "axis * 2u + (interval.windowId & 1u)", "axis and train/validation stratification")
    require(axis_h, "axisExcitationRad_[3]", "incremental axis-coverage accounting")
    require(axis_h, "uint32_t independentWindows_", "incremental independent-window accounting")
    require(axis_h, "partitionConfirmedAxes", "per-partition axis-coverage gate")
    forbid(axis_h, "for (uint16_t j = 0; j < i; ++j)", "quadratic setup-loop window counting")
    forbid(axis_h, "*this = MagAxisIntervalReservoir{}", "large by-value reservoir reset")

    require(setup, "MagAxisIntervalReservoir<kMaxIntervals> intervalReservoir", "guided collector reservoir wiring")
    require(setup, "kMinAcceptedSpacingUs = 75000u", "guided interval cadence bound")
    require(setup, "dynamic_axis_candidates_seen=", "candidate interval diagnostics")
    require(setup, "dynamic_axis_reservoir_replacements=", "axis reservoir replacement diagnostics")
    require(setup, "dynamic_axis_partition_confirmed_axes=", "per-partition axis diagnostics")
    require(setup, "dynamic_axis_bucket_counts=", "axis/partition bucket diagnostics")
    require(setup, "tempReadyAfterRest", "post-rest temperature epoch recheck")
    require(setup, "if (!axisDynamic.readyForSolve() && !(axisX && axisY && axisZ))", "dynamic fallback despite inconclusive face samples")
    require(setup, "collectHardSoftDuringAccel", "hard/soft collector ownership split")
    require(setup, "collectAxisFacesDuringAccel", "axis-face observation ownership split")
    require(setup, "manualAxisRequested", "manual-axis observation suppression")
    require(setup, "stopped temporary face-sample hard/soft collector", "defensive temporary collector cleanup")
    require(setup, "nomag/6dof cannot be combined with an axis mapping", "contradictory option rejection")
    require(setup, "char* tokens[4]", "manual axis trailing-token rejection")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupRunAxisAlignment", "cross-ABI axis alignment noinline boundary")
    require(setup, "setupTryApplyDynamicAxisAlignment", "split dynamic axis phase")
    require(setup, "setupTryApplyStaticAxisAlignment", "split static axis phase")
    require(setup, "setupPromptManualAxisMapping", "split manual prompt phase")
    forbid(setup, "dynamic_axis_dropped=", "misleading frozen-buffer diagnostic")
    forbid(setup, "dynamic_axis_capacity_full=", "capacity-only pseudo-fix diagnostic")
    forbid(setup, "*this = SetupMagAxisDynamicCollector{}", "large by-value guided collector reset")

    require(status, "fit_span_xyz=", "detailed fit-set span diagnostics")
    require(status, "out.println(fitSet.samples);", "fit-set inlier denominator")
    require(controller, "# mag_cal_fit_span_xyz=", "failure-path fit-set diagnostics")
    require(hooks, "fit_span_xyz=", "compact fit-set diagnostics")

    require(mag_test, "a long final sweep around one axis", "hard/soft long-tail regression")
    require(mag_test, "fitSetDiagnostics", "fit-set gate regression")
    require(heading_test, "testGuidedAxisReservoirPreservesLateAxesAndPartitions", "guided-axis late-coverage regression")
    require(heading_test, "bucketSeen(bucket) > reservoir.bucketCount(bucket)", "active reservoir assertion")

    require(cli, "stratified reservoir", "CLI guided reservoir documentation")
    require(project, "0023g", "project patch identity documentation")
    require(testing, "0023g", "0023g test documentation")
    require(report, "Why the supplied 0023h did not apply", "0023g defect report")
    require(report, "dynamic_axis_partition_confirmed_axes", "hardware acceptance diagnostics")

    with project_temp_directory(ROOT, "tracker-0023g-stack-") as tmp:
        tmp_path = Path(tmp)
        common = [
            compiler(), "-std=c++20", "-O2", "-fstack-usage",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
        ]
        subprocess.run(
            [*common, "-c", str(ROOT / "src/serial/tracker_setup_commands.cpp"),
             "-o", str(tmp_path / "setup.o")],
            check=True,
        )
        subprocess.run(
            [*common, "-c", str(ROOT / "src/sensor/mag_calibration.cpp"),
             "-o", str(tmp_path / "mag.o")],
            check=True,
        )
        usage = parse_stack_usage(tmp_path)
    require_limit(usage, "setupRunMagMotionAndApply", 256)
    require_limit(usage, "setupRunAxisAlignment", 512)
    require_limit(usage, "setupTryApplyDynamicAxisAlignment", 768)
    require_limit(usage, "setupTryApplyStaticAxisAlignment", 512)
    require_limit(usage, "setupPrintStaticAxisCrossCheck", 512)
    require_limit(usage, "setupPromptManualAxisMapping", 512)
    require_limit(usage, "setupAutoSolveMagAxisDynamic", 768)
    require_limit(usage, "setupAutoSolveMagAxis", 768)
    require_limit(usage, "MagCalibrationCollector::compute", 2048)

    schema = (ROOT / "src/config/tracker_config_detail.hpp").read_text(encoding="utf-8")
    storage = (ROOT / "src/config/tracker_config_storage.hpp").read_text(encoding="utf-8")
    require(schema, "CONFIG_VERSION = 2", "unchanged config schema")
    require(storage, "CANDIDATE_VERSION = 3", "unchanged candidate format")

    print("# calibration_0023g_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
