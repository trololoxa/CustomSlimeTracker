#!/usr/bin/env python3
"""Guard 0023gg magnetic timestamp, axis-consensus and setup-verification contracts."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
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
    raise SystemExit("no C++ compiler available for 0023gg policy")




def stack_compiler(default_cxx: str) -> str:
    # Clang may inline tiny functions and omit their standalone .su records.
    # Use GCC stack-usage output when available; functional tests still honor
    # CXX so both compiler families can exercise the code.
    for name in ("g++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    return default_cxx

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
        raise SystemExit(f"0023gg stack policy did not find function: {needle}")
    worst_name, worst_size = max(matches, key=lambda item: item[1])
    if worst_size > limit:
        raise SystemExit(
            f"0023gg stack budget exceeded: {worst_name} uses {worst_size} bytes, limit {limit}"
        )


def compile_and_run(cxx: str, tmp: Path, name: str, sources: list[str]) -> None:
    exe = tmp / name
    subprocess.run(
        [
            cxx, "-std=c++20", "-O2",
            "-Wall", "-Wextra", "-Wshadow", "-Wdouble-promotion", "-Wformat=2",
            "-Wno-unused-parameter",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
            *(str(ROOT / source) for source in sources),
            "-o", str(exe),
        ],
        check=True,
    )
    subprocess.run([str(exe)], check=True)


def main() -> int:
    fifo_h = (ROOT / "src/connection/lsm6dsv_fifo.hpp").read_text(encoding="utf-8")
    fifo = (ROOT / "src/connection/lsm6dsv_fifo.cpp").read_text(encoding="utf-8")
    axis_h = (ROOT / "src/sensor/mag_axis_alignment.hpp").read_text(encoding="utf-8")
    axis = (ROOT / "src/sensor/mag_axis_alignment.cpp").read_text(encoding="utf-8")
    setup = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    verifier_h = (ROOT / "src/runtime/setup_output_verifier.hpp").read_text(encoding="utf-8")
    verifier = (ROOT / "src/runtime/setup_output_verifier.cpp").read_text(encoding="utf-8")
    fifo_test = (ROOT / "tests/native/test_fifo_pair_coherency.cpp").read_text(encoding="utf-8")
    axis_test = (ROOT / "tests/native/test_mag_heading_reliability.cpp").read_text(encoding="utf-8")
    verifier_test = (ROOT / "tests/native/test_setup_output_verifier.cpp").read_text(encoding="utf-8")
    imu_cli = (ROOT / "src/serial/tracker_imu_fifo_commands.cpp").read_text(encoding="utf-8")
    mag_cli = (ROOT / "src/serial/tracker_mag_commands.cpp").read_text(encoding="utf-8")
    runtime_status = (ROOT / "src/runtime/runtime_status_reporter.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")
    report = (ROOT / "docs/0023gg_magnetic_timestamp_and_setup_acceptance_hardening_report.md").read_text(
        encoding="utf-8"
    )

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
    require(setup, "earlier committed checkpoints, if any, remain authoritative", "checkpoint-honest rollback message")
    forbid(setup, "while (millis() - captureStartMs < 4000UL)", "fixed verification capture")
    require(verifier_test, "pushStableInput(verifier, 207)", "hardware sample-count regression")

    require(check_all,
            '("tools/test_calibration_0023gg_policy.py", "0023gg magnetic timestamp/setup acceptance policy")',
            "aggregate runner entry")
    require(report, "4255", "recorded hardware gyro-skew symptom")
    require(report, "207", "recorded setup verification sample-count symptom")

    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-0023gg-") as tmp_name:
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
            subprocess.run([*common, "-c", str(ROOT / source), "-o", str(tmp / output)], check=True)
        usage = parse_stack_usage(tmp)
        require_limit(usage, "parseSensorHubSlave0Word", 256)
        require_limit(usage, "solveMagAxisAlignmentDataset", 1536)
        require_limit(usage, "SetupOutputVerificationAccumulator::finish", 512)
        require_limit(usage, "setupVerifyOutputRuntime", 512)

        compile_and_run(cxx, tmp, "test_fifo_pair_coherency", [
            "tests/native/test_fifo_pair_coherency.cpp",
            "src/connection/lsm6dsv_fifo.cpp",
            "src/connection/lsm6dsv_driver.cpp",
            "src/sensor/imu_quality.cpp",
        ])
        compile_and_run(cxx, tmp, "test_mag_heading_reliability", [
            "tests/native/test_mag_heading_reliability.cpp",
            "src/sensor/mag_runtime.cpp",
            "src/sensor/mag_heading.cpp",
            "src/sensor/mag_field_reliability.cpp",
            "src/sensor/mag_axis_alignment.cpp",
            "src/sensor/mag_yaw_correction.cpp",
        ])
        compile_and_run(cxx, tmp, "test_setup_output_verifier", [
            "tests/native/test_setup_output_verifier.cpp",
            "src/runtime/setup_output_verifier.cpp",
        ])

    # check_all.py runs every predecessor policy explicitly. Do not recursively
    # replay the full suffix chain here: older policies already recurse and a
    # further nested invocation makes aggregate validation quadratic without
    # increasing firmware coverage. Direct users should run check_all for the
    # complete chain.
    require(check_all,
            '("tools/test_calibration_0023gf_policy.py", "0023gf mag callback cross-ABI policy")',
            "predecessor policy aggregate entry")
    print("# calibration_0023gg_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
