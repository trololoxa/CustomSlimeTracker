#!/usr/bin/env python3
"""Guard 0023gd magnetometer fit/alignment mathematical and timing contracts."""

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
    raise SystemExit("no C++ compiler available for 0023gd policy")


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
        raise SystemExit(f"0023gd stack policy did not find function: {needle}")
    worst_name, worst_size = max(matches, key=lambda item: item[1])
    if worst_size > limit:
        raise SystemExit(
            f"0023gd stack budget exceeded: {worst_name} uses {worst_size} bytes, limit {limit}"
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
    mag_h = (ROOT / "src/sensor/mag_calibration.hpp").read_text(encoding="utf-8")
    mag = (ROOT / "src/sensor/mag_calibration.cpp").read_text(encoding="utf-8")
    axis_h = (ROOT / "src/sensor/mag_axis_alignment.hpp").read_text(encoding="utf-8")
    axis = (ROOT / "src/sensor/mag_axis_alignment.cpp").read_text(encoding="utf-8")
    mag_runtime_h = (ROOT / "src/sensor/mag_runtime.hpp").read_text(encoding="utf-8")
    mag_runtime = (ROOT / "src/sensor/mag_runtime.cpp").read_text(encoding="utf-8")
    controller = (ROOT / "src/runtime/mag_runtime_controller.cpp").read_text(encoding="utf-8")
    fifo_h = (ROOT / "src/runtime/fifo_runtime_processor.hpp").read_text(encoding="utf-8")
    fifo = (ROOT / "src/runtime/fifo_runtime_processor.cpp").read_text(encoding="utf-8")
    fifo_reader_h = (ROOT / "src/connection/lsm6dsv_fifo.hpp").read_text(encoding="utf-8")
    fifo_reader = (ROOT / "src/connection/lsm6dsv_fifo.cpp").read_text(encoding="utf-8")
    qmc_h = (ROOT / "src/sensor/qmc6309.hpp").read_text(encoding="utf-8")
    qmc = (ROOT / "src/sensor/qmc6309.cpp").read_text(encoding="utf-8")
    setup = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    tuning = (ROOT / "src/build_config/tracking_tuning.hpp").read_text(encoding="utf-8")
    status = (ROOT / "src/runtime/mag_status_reporter.cpp").read_text(encoding="utf-8")
    mag_test = (ROOT / "tests/native/test_mag_calibration.cpp").read_text(encoding="utf-8")
    heading_test = (ROOT / "tests/native/test_mag_heading_reliability.cpp").read_text(encoding="utf-8")
    fifo_test = (ROOT / "tests/native/test_fifo_runtime_processor.cpp").read_text(encoding="utf-8")
    pair_test = (ROOT / "tests/native/test_fifo_pair_coherency.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")
    project = (ROOT / "docs/project_status.md").read_text(encoding="utf-8")
    testing = (ROOT / "docs/testing.md").read_text(encoding="utf-8")
    cli = (ROOT / "docs/cli_reference.md").read_text(encoding="utf-8")
    frames = (ROOT / "docs/coordinate_frames.md").read_text(encoding="utf-8")
    report = (ROOT / "docs/0023gd_magnetometer_math_and_alignment_hardening_report.md").read_text(encoding="utf-8")

    # Hard/soft-iron fit: affine normalization, conditioned solve and fail-honest stages.
    require(mag_h, "FitNormalizationFailed", "normalization failure reason")
    require(mag_h, "LinearSolveFailed", "linear solve failure reason")
    require(mag_h, "NonPositiveDefiniteShape", "SPD failure reason")
    require(mag_h, "fitNormalizationCenter", "normalization center diagnostics")
    require(mag_h, "solverPivotRatio", "solver conditioning diagnostics")
    require(mag_h, "magCalibrationEffectiveMinBoxCoverage", "anisotropy-compatible pre-fit coverage")
    require(mag, "struct FitNormalization", "affine normalization workspace")
    require(mag, "finiteCount", "finite-only normalization denominator")
    require(mag, "const double dx = static_cast<double>(v.x) - mean[0]", "translation-independent outlier filter")
    require(mag, "boxCoverageScoreOut = maxRadius > 0.0", "post-filter coverage gate")
    forbid(mag, "const float spanX = fitSet.max.x - fitSet.min.x", "pre-filter raw-extrema coverage gate")
    forbid(mag, "rawNorm >= lo", "origin-dependent raw-norm filter")
    require(mag, "columnScale[i] = std::sqrt(diagonal)", "normal-equation equilibration")
    require(mag, "pivotRatio <= 1.0e-10", "conditioning rejection")
    require(mag, "shapeRaw[r][c] = quad[r][c] /", "double raw-shape mapping")
    require(mag, "normalization.center[0] + normalization.scale[0] * centerNormalized[0]", "raw center mapping")
    require(mag, "softIron.determinant() <= 0.0f", "proper SPD correction validation")
    require(mag, "std::sqrt(residualVar) / std::fabs(k)", "translation-independent algebraic residual")

    # Alignment observability and frame contracts.
    require(axis_h, "buildMagAxisAlignmentInterval", "shared endpoint interval builder")
    require(axis_h, "MAG_AXIS_MAX_GYRO_MAG_SKEW_US", "shared gyro/mag skew ceiling")
    require(axis_h, "class MagAxisIntervalReservoir", "bounded runtime/guided reservoir")
    require(axis, "countSubsetWindows", "exact subset window counting")
    require(axis, "countPartitionConfirmedAxes", "dataset-derived partition coverage")
    forbid(axis, "uint8_t excitedAxes,", "caller-supplied fake axis coverage")
    forbid(axis, "uint32_t independentWindows,", "caller-supplied fake window coverage")
    require(axis, "mag.calibratedMagFrame", "calibrated-direction admission")
    require(axis, "previousGyroSensorRadS_", "timestamp endpoint trapezoid")
    require(axis_h, "kMinAcceptedSpacingUs", "sensor-time runtime cadence")
    require(axis_h, "lastWindowUs_", "sensor-time runtime windows")
    require(axis, "mag.t_us - lastWindowUs_", "sensor-time window partitioning")
    forbid(axis, "mag.receivedMs - lastWindow", "processing-time window partitioning")
    require(axis, "MAG_AXIS_MAX_GYRO_MAG_SKEW_US", "runtime shared skew gate")
    require(axis, "isProperRotationMatrix", "proper SO(3) persistence gate")
    require(mag_runtime, "Raw zero is not intrinsically invalid", "calibrated raw-origin admission")
    require(mag_runtime_h, "gyroEndpointSkewUs", "saved endpoint skew diagnostic")
    require(controller, "processed.gyroEndpointSkewUs", "endpoint skew capture")
    require(controller, "processed.gyroEndpointValid = true", "exact callback endpoint retention")
    require(controller, "MagProcessedSample& processed = *deps_.lastProcessed", "no duplicate hot-path endpoint snapshot")
    forbid(controller, "MagProcessedSample processed;", "stack-local duplicate endpoint snapshot")
    require(setup, "prevGyroSensorRadS, mag.gyroSensorRadS", "guided trapezoidal endpoints")
    forbid(setup, "gyroNormDps >= 2.0f", "selected-only gyro averaging bias")
    require(setup, "gyroEndpointsSeen", "accurate guided endpoint accounting")
    require(setup, "gyro_endpoints_seen=", "accurate guided endpoint diagnostic")
    forbid(setup, "imuSamplesSeen", "misleading endpoint-as-IMU-sample counter")
    forbid(setup, "dynamic result not applied", "weak static veto of validated dynamic solve")
    require(setup, "retaining validated dynamic result", "warning-only static cross-check disagreement")
    require(setup, "TRACKER_SETUP_NOINLINE void setupPrintStaticAxisCrossCheck", "non-vetoing static cross-check contract")
    forbid(setup, "if (!setupPrintStaticAxisCrossCheck", "conditional static veto call")
    require(setup, "if (!setupApplyAxisMatrix(ctx, dynamicAxis.magToImu))", "validated dynamic result application")
    require(setup, "dynamic_axis_gyro_skew_rejected=", "guided skew diagnostics")
    require(status, "gyro_endpoint_skew_us=", "runtime endpoint skew diagnostics")

    # FIFO chronology and QMC input validity.
    require(tuning, "FIFO_RUNTIME_MAX_MAG_CALLBACKS_PER_SLICE = 8", "bounded mag callback headroom")
    require(fifo_h, "magChronologicalDeferrals", "chronological deferral counter")
    require(fifo, "dispatchDueMagCallbacks", "chronological magnetic dispatch")
    require(fifo, "magTimestampUs <= rawTimestampUs", "timestamp ordering predicate")
    require(fifo, "before advancing the raw timeline", "bounded chronology rationale")
    require(fifo_reader_h, "sensorHubSlave0SaturationAbs", "sensor-hub near-rail threshold")
    require(fifo_reader, "MAG_FLAG_RAW_SATURATED", "sensor-hub saturation flag")
    require(qmc_h, "RAW_SATURATION_ABS_COUNTS = 31900", "QMC near-rail threshold")
    require(qmc_h, "RAW_REGISTER_AXES_RIGHT_HANDED = true", "driver handedness contract")
    require(qmc, "static_assert(Qmc6309::RAW_REGISTER_AXES_RIGHT_HANDED", "compile-time handedness guard")

    # Regression fixtures.
    require(mag_test, "hard-iron is about one", "hardware-shaped ellipsoid fixture")
    require(mag_test, "Deterministic convergence sweep", "fit convergence sweep")
    require(mag_test, "fully covered 4:1 ellipsoid", "anisotropy/coverage compatibility fixture")
    require(mag_test, "finite one-axis magnetic spike", "post-filter coverage fixture")
    require(heading_test, "testAllProperSignedPermutationMountingsConverge", "all 24 proper mappings")
    require(heading_test, "testReflectionAmbiguityRequiresRightHandedDriverContract", "reflection ambiguity contract")
    require(heading_test, "testIntervalBuilderUsesTimestampCoherentGyroEndpoints", "endpoint integration fixture")
    require(heading_test, "testSolverCountsUniqueReservoirWindows", "exact window fixture")
    require(heading_test, "testCalibratedRawOriginRemainsUsable", "raw-origin runtime fixture")
    require(heading_test, "testRuntimeCollectorUsesSensorTimeAcrossProcessingBacklog", "sensor-time backlog fixture")
    require(fifo_test, "testMagCallbacksFollowNearestRawTimestamp", "chronological callback fixture")
    require(pair_test, "31950", "near-rail FIFO saturation fixture")

    require(check_all, '("tools/test_calibration_0023gd_policy.py", "0023gd magnetometer math/alignment policy")', "aggregate runner entry")
    require(project, "0023gd_magnetometer_math_and_alignment_hardening", "project status entry")
    require(testing, "0023gd full magnetometer mathematics regression", "testing entry")
    require(cli, "last_fit_solver_stage", "CLI solver diagnostics")
    require(frames, "S_raw = D^-T", "coordinate-frame fit equations")
    require(report, "Uncentered raw-origin fit", "recorded root cause")
    require(report, "Raw-first FIFO scheduling", "recorded timing defect")

    schema = (ROOT / "src/config/tracker_config_detail.hpp").read_text(encoding="utf-8")
    storage = (ROOT / "src/config/tracker_config_storage.hpp").read_text(encoding="utf-8")
    require(schema, "CONFIG_VERSION = 2", "unchanged config schema")
    require(storage, "CANDIDATE_VERSION = 3", "unchanged candidate format")

    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-0023gd-") as tmp_name:
        tmp = Path(tmp_name)
        common = [
            cxx, "-std=c++20", "-O2", "-fstack-usage",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
        ]
        for source, output in (
            ("src/sensor/mag_calibration.cpp", "mag.o"),
            ("src/sensor/mag_axis_alignment.cpp", "axis.o"),
            ("src/serial/tracker_setup_commands.cpp", "setup.o"),
            ("src/runtime/fifo_runtime_processor.cpp", "fifo.o"),
            ("src/runtime/mag_runtime_controller.cpp", "controller.o"),
        ):
            subprocess.run([*common, "-c", str(ROOT / source), "-o", str(tmp / output)], check=True)
        usage = parse_stack_usage(tmp)
        require_limit(usage, "solveLinear9", 1024)
        require_limit(usage, "fitFromAccumulator", 1024)
        require_limit(usage, "MagCalibrationCollector::compute", 1792)
        require_limit(usage, "buildRefinedCandidates", 1024)
        require_limit(usage, "solveMagAxisAlignmentDataset", 1536)
        require_limit(usage, "SetupMagAxisDynamicCollector::update", 512)
        require_limit(usage, "setupAutoSolveMagAxisDynamic", 512)
        require_limit(usage, "setupRunAxisAlignment", 512)
        require_limit(usage, "FifoRuntimeProcessor::process", 512)
        require_limit(usage, "MagRuntimeController::processRawSample", 1024)

        compile_and_run(cxx, tmp, "test_mag_calibration", [
            "tests/native/test_mag_calibration.cpp",
            "src/sensor/mag_calibration.cpp",
        ])
        compile_and_run(cxx, tmp, "test_mag_heading_reliability", [
            "tests/native/test_mag_heading_reliability.cpp",
            "src/sensor/mag_runtime.cpp",
            "src/sensor/mag_heading.cpp",
            "src/sensor/mag_field_reliability.cpp",
            "src/sensor/mag_axis_alignment.cpp",
            "src/sensor/mag_yaw_correction.cpp",
        ])
        compile_and_run(cxx, tmp, "test_fifo_runtime_processor", [
            "tests/native/test_fifo_runtime_processor.cpp",
            "src/runtime/fifo_runtime_processor.cpp",
            "src/connection/lsm6dsv_fifo.cpp",
            "src/connection/lsm6dsv_driver.cpp",
            "src/sensor/imu_quality.cpp",
        ])
        compile_and_run(cxx, tmp, "test_fifo_pair_coherency", [
            "tests/native/test_fifo_pair_coherency.cpp",
            "src/connection/lsm6dsv_fifo.cpp",
            "src/connection/lsm6dsv_driver.cpp",
            "src/sensor/imu_quality.cpp",
        ])

    print("# calibration_0023gd_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
