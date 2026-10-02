#!/usr/bin/env python3
"""Guard guided magnetometer coverage-reservoir and setup-flow hardening."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, parse_stack_usage, read_native_test, require, require_limit, run_contract_command,
)


from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]


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
    heading_test = read_native_test(ROOT / "tests/native/test_mag_heading_reliability.cpp")

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


    with project_temp_directory(ROOT, "tracker-magnetic_coverage_reservoir-stack-") as tmp:
        tmp_path = Path(tmp)
        common = [
            compiler(), "-std=c++20", "-O2", "-fstack-usage",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
        ]
        run_contract_command(
            [*common, "-c", str(ROOT / "src/serial/tracker_setup_commands.cpp"),
             "-o", str(tmp_path / "setup.o")],
            check=True,
        )
        run_contract_command(
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

    print("# magnetic_coverage_reservoir: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
