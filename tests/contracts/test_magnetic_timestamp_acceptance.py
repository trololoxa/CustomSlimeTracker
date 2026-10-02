#!/usr/bin/env python3
"""Guard magnetic_timestamp_acceptance magnetic timestamp, axis-consensus and setup-verification contracts."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, parse_stack_usage, read_native_test, require, require_limit, run_contract_command,
)

import shutil

from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]


def stack_compiler(default_cxx: str) -> str:
    # Clang may inline tiny functions and omit their standalone .su records.
    # Use GCC stack-usage output when available; functional tests still honor
    # CXX so both compiler families can exercise the code.
    for name in ("g++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    return default_cxx


def main() -> int:
    fifo_h = (ROOT / "src/connection/lsm6dsv_fifo.hpp").read_text(encoding="utf-8")
    fifo = (ROOT / "src/connection/lsm6dsv_fifo.cpp").read_text(encoding="utf-8")
    axis_h = (ROOT / "src/sensor/mag_axis_alignment.hpp").read_text(encoding="utf-8")
    axis = (ROOT / "src/sensor/mag_axis_alignment.cpp").read_text(encoding="utf-8")
    setup = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    verifier_h = (ROOT / "src/runtime/setup_output_verifier.hpp").read_text(encoding="utf-8")
    verifier = (ROOT / "src/runtime/setup_output_verifier.cpp").read_text(encoding="utf-8")
    fifo_test = (ROOT / "tests/native/test_fifo_pair_coherency.cpp").read_text(encoding="utf-8")
    axis_test = read_native_test(ROOT / "tests/native/test_mag_heading_reliability.cpp")
    verifier_test = (ROOT / "tests/native/test_setup_output_verifier.cpp").read_text(encoding="utf-8")
    imu_cli = (ROOT / "src/serial/tracker_imu_fifo_commands.cpp").read_text(encoding="utf-8")
    mag_cli = (ROOT / "src/serial/tracker_mag_commands.cpp").read_text(encoding="utf-8")
    runtime_status = (ROOT / "src/runtime/runtime_status_reporter.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")

    # Every sensor-hub frame must be tied back to the current IMU/FIFO domain.
    require(fifo_h, "MAG_FLAG_TIMESTAMP_IMU_ANCHORED", "per-frame IMU anchor flag")
    require(fifo_h, "magTimestampImuAnchors", "IMU anchor counter")
    require(fifo_h, "magTimestampNominalFallbacks", "nominal fallback counter")
    require(fifo_h, "magTimestampMonotonicAdjustments", "monotonic adjustment counter")
    require(fifo, "const uint64_t imuAnchorUs = std::max", "current FIFO/IMU anchor selection")
    require(fifo, "if (imuAnchorUs != 0)", "IMU anchor primary path")
    require(fifo, "m.flags |= MAG_FLAG_TIMESTAMP_IMU_ANCHORED", "IMU anchored sample tagging")
    require(fifo, "else if (nominalNextUs != 0)", "nominal fallback only without IMU anchor")
    require(fifo, "m.t_us = stats_.lastMagTimestampUs + 1u", "fail-honest monotonic marker")
    forbid(
        fifo,
        "if (stats_.lastMagTimestampUs != 0 && periodUs > 0) {\n            m.t_us = stats_.lastMagTimestampUs + periodUs;",
        "free-running nominal magnetic clock as primary path",
    )
    require(fifo_test, "testSensorHubTimestampsReanchorToImuTimeline", "ODR drift regression")
    require(fifo_test, "testSensorHubTimestampsUseHardwareImuAnchor", "hardware anchor regression")
    require(fifo_test, "testSensorHubRepeatedAnchorUsesFailHonestMonotonicMarker", "repeated-anchor fail-honest regression")

    for text, label in ((imu_cli, "FIFO CLI"), (mag_cli, "mag CLI"), (runtime_status, "runtime status")):
        require(text, "mag_timestamp_imu_anchors=", f"{label} anchor diagnostic")
        require(text, "mag_timestamp_nominal_fallbacks=", f"{label} fallback diagnostic")
        require(text, "mag_timestamp_monotonic_adjustments=", f"{label} monotonic diagnostic")
        require(text, "mag_timestamp_max_anchor_correction_us=", f"{label} correction diagnostic")

    # Same independently recovered discrete mounting may conservatively drop
    # an inconsistent sub-degree mechanical refinement, but a different coarse
    # axis/sign winner must still fail closed.
    require(axis_h, "coarseWinnerMatchesTraining", "coarse partition agreement diagnostic")
    require(axis_h, "continuousRefinementAgreement", "continuous refinement diagnostic")
    require(axis_h, "coarseConsensusFallbackUsed", "coarse fallback diagnostic")
    require(axis, "coarseWinnerMatches && !continuousWinnerMatches", "same-coarse fallback predicate")
    require(axis, "trainingBest.coarse", "shared proper coarse fallback matrix")
    require(axis, "out.validationWinnerMatchesTraining = coarseWinnerMatches", "coarse consensus acceptance contract")
    require(axis, "if (!out.validationWinnerMatchesTraining)", "coarse mismatch fail-closed gate")
    require(axis_test, "testSameCoarseWinnerFallsBackWhenRefinementsDisagree", "coarse fallback regression")
    require(axis_test, "coarseConsensusFallbackUsed", "coarse fallback assertion")
    require(setup, "gyro_mag_axis_continuous_refinement_agreement=", "guided continuous diagnostic")
    require(setup, "gyro_mag_axis_coarse_consensus_fallback_used=", "guided fallback diagnostic")

    # Final verification must retain the original evidence threshold and adapt
    # capture duration instead of rejecting a healthy 207-sample four-second run.
    require(verifier_h, "stationaryInputSampleCountPassed", "stationary sample-count sub-gate")
    require(verifier_h, "stationaryGyroMeanPassed", "gyro mean sub-gate")
    require(verifier_h, "stationaryGyroPrecisionPassed", "gyro precision sub-gate")
    require(verifier_h, "stationaryAccelMeanPassed", "accel mean sub-gate")
    require(verifier_h, "stationaryAccelStdPassed", "accel variance sub-gate")
    require(verifier_h, "uint32_t inputSampleCount() const", "adaptive input evidence getter")
    require(verifier, "result.stationaryInputSampleCountPassed", "sample-count evaluation")
    require(setup, "constexpr uint32_t kMinimumCaptureMs = 4000UL", "minimum verification dwell")
    require(setup, "constexpr uint32_t kMaximumCaptureMs = 8000UL", "bounded verification extension")
    require(setup, "verifier.inputSampleCount() >= verifyConfig.minimumInputSamples", "adaptive evidence stop")
    require(setup, "capture_duration_ms=", "capture duration diagnostic")
    require(setup, "stationary_input_sample_count_passed=", "stationary sample diagnostic")
    require(setup + (ROOT / "src/serial/tracker_calibration_transaction.hpp").read_text(encoding="utf-8"), "earlier committed checkpoints, if any, remain authoritative", "checkpoint-honest rollback message")
    forbid(setup, "while (millis() - captureStartMs < 4000UL)", "fixed verification capture")
    require(verifier_test, "pushStableInput(verifier, 207)", "hardware sample-count regression")


    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-magnetic_timestamp_acceptance-") as tmp_name:
        tmp = Path(tmp_name)
        stack_cxx = stack_compiler(cxx)
        common = [
            stack_cxx, "-std=c++20", "-O2", "-fstack-usage",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
        ]
        for source, output in (
            ("src/connection/lsm6dsv_fifo.cpp", "fifo_reader.o"),
            ("src/sensor/mag_axis_alignment.cpp", "axis.o"),
            ("src/runtime/setup_output_verifier.cpp", "verifier.o"),
            ("src/serial/tracker_setup_commands.cpp", "setup.o"),
        ):
            run_contract_command([*common, "-c", str(ROOT / source), "-o", str(tmp / output)], check=True)
        usage = parse_stack_usage(tmp)
        require_limit(usage, "parseSensorHubSlave0Word", 256)
        require_limit(usage, "solveMagAxisAlignmentDataset", 1536)
        require_limit(usage, "SetupOutputVerificationAccumulator::finish", 512)
        require_limit(usage, "setupVerifyOutputRuntime", 512)


    # check_all.py runs every predecessor policy explicitly. Do not recursively
    # replay the full suffix chain here: older policies already recurse and a
    # further nested invocation makes aggregate validation quadratic without
    # increasing firmware coverage. Direct users should run check_all for the
    # complete chain.
    print("# magnetic_timestamp_acceptance: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
