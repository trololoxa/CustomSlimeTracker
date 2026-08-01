#!/usr/bin/env python3
"""Guard pre-0024ac IMU hotpath reuse and optional-service admission."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(haystack: str, needle: str, label: str) -> None:
    if needle not in haystack:
        raise SystemExit(f"missing {label}: {needle}")


def forbid(haystack: str, needle: str, label: str) -> None:
    if needle in haystack:
        raise SystemExit(f"forbidden {label}: {needle}")


def isolate(haystack: str, begin: str, end: str) -> str:
    start = haystack.find(begin)
    if start < 0:
        raise SystemExit(f"section start not found: {begin}")
    finish = haystack.find(end, start)
    if finish < 0:
        raise SystemExit(f"section end not found: {end}")
    return haystack[start:finish]


def compiler() -> str:
    for candidate in (os.environ.get("CXX"), "g++", "clang++", "c++"):
        if candidate and shutil.which(candidate):
            return candidate
    raise SystemExit("no C++ compiler available for pre-0024ac policy")


def run(cmd: list[str]) -> None:
    subprocess.run(cmd, cwd=ROOT, check=True)


def compile_run(cxx: str, exe: Path, sources: list[str], flags: list[str]) -> None:
    run([
        cxx, "-std=c++20", *flags,
        "-I", str(ROOT / "src"),
        "-I", str(ROOT / "tests/native"),
        *[str(ROOT / source) for source in sources],
        "-o", str(exe),
    ])
    run([str(exe)])


def stack_usage(tmp: Path, symbol: str) -> int:
    found: list[int] = []
    for path in tmp.glob("*.su"):
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if symbol not in line:
                continue
            fields = line.rsplit("\t", 2)
            if len(fields) >= 2:
                match = re.search(r"(\d+)", fields[-2])
                if match:
                    found.append(int(match.group(1)))
    if not found:
        raise SystemExit(f"stack usage not found for {symbol}")
    return max(found)


def main() -> int:
    pipeline = text("src/runtime/imu_sample_pipeline.cpp")
    pipeline_h = text("src/runtime/imu_sample_pipeline.hpp")
    temp_h = text("src/sensor/gyro_temperature_compensation.hpp")
    temp = text("src/sensor/gyro_temperature_compensation.cpp")
    bias = text("src/runtime/runtime_gyro_bias_controller.cpp")
    app = text("src/app/tracker_app.cpp")
    admission = text("src/runtime/tracking_slack_admission.hpp")
    profiler_h = text("src/runtime/runtime_profiler.hpp")
    profiler = text("src/runtime/runtime_profiler.cpp")
    tuning = text("src/build_config/runtime_tuning.hpp")
    tracking_tuning = text("src/build_config/tracking_tuning.hpp")
    check_all = text("tools/check_all.py")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/pre_0024ac_imu_hotpath_slack_hardening_report.md")

    # Tracking-quality invariants: this wave may reuse immutable/per-sample
    # calculations and defer optional bookkeeping, but cannot alter sensor
    # history, fusion equations or output cadence.
    require(tracking_tuning, "FIFO_RUNTIME_MAX_RAW_CALLBACKS_PER_SLICE = 64", "unchanged FIFO sample contract")
    for forbidden in ("dropOldest", "discardRaw", "decimateGyro", "skipRaw"):
        forbid(pipeline, forbidden, f"sensor-history loss: {forbidden}")
    require(report, "No IMU sample is dropped", "quality-preservation statement")

    # One lightweight temperature evaluation is created by the main 960 Hz
    # pipeline and reused for correction, quality and bias evidence.
    main_pipeline = isolate(
        pipeline,
        "FifoRuntimeSampleResult imuSamplePipelineProcessRaw",
        "\n} // namespace tracker",
    )
    if main_pipeline.count("evaluateRuntime(") != 1:
        raise SystemExit("main IMU pipeline must evaluate temperature compensation exactly once per sample")
    require(main_pipeline, "const Vec3 currentGyroBiasRadS", "single current-bias evaluation")
    require(main_pipeline, "runtimeBiasApplyGyroTempQualityFlags(tempEval", "shared temp quality evaluation")
    require(main_pipeline, "tempEval, currentGyroBiasRadS", "shared runtime-bias inputs")
    forbid(main_pipeline, ".snapshot(", "full human-readable temp snapshot in sample path")
    require(temp_h, "struct GyroTempCompRuntimeEval", "lightweight runtime temperature state")
    require(temp, "const GyroTempCompRuntimeEval eval = evaluateRuntime(currentTempC);", "snapshot/runtime shared semantics")
    require(bias, "const GyroTempCompRuntimeEval& tempEval", "bias overload reuses temp evaluation")
    require(bias, "scaled.gyro_rad_s - currentGyroBiasRadS", "single residual-bias value")

    # Disabled serial streaming must not read micros() on every IMU sample.
    output = isolate(
        pipeline,
        "bool imuPipelineEmitPerSampleOutputs(ImuSamplePipelineDeps& deps,",
        "FifoRuntimeSampleResult imuSamplePipelineProcessRaw",
    )
    mode_pos = output.find("deps.streamState->mode != TrackerStreamMode::Off")
    micros_pos = output.find("micros()")
    if mode_pos < 0 or micros_pos < 0 or mode_pos > micros_pos:
        raise SystemExit("serial stream mode must be checked before the hotpath micros() read")

    # Optional services require empty/non-urgent FIFO and deadline slack. One
    # completed background worker consumes the loop's background slot.
    require(admission, "input.fifoUrgent || input.softwareQueuePending", "FIFO admission veto")
    require(admission, "trackingBackgroundRuntimeAdmitted", "single background-slot helper")
    require(admission, "!backgroundSlotConsumed", "background phase-collision prevention")
    loop = isolate(app, "void TrackerApp::loop()", "bool TrackerApp::ready() const")
    if loop.count("trackingSlackAdmissionInput()") != 3:
        raise SystemExit("ordinary app loop must take exactly three shared slack snapshots")
    require(loop, "trackingBackgroundRuntimeAdmitted(\n        postCriticalSlack, magDeferredWorked)", "autonomy waits after actual mag work")
    require(loop, "updateNetworkRuntime", "critical outer network service remains ungated")
    require(loop, "updateTapRuntime", "tap cadence remains ungated")
    require(tuning, "TRACKER_OPTIONAL_SHORT_SERVICE_MIN_SLACK_US 1500UL", "short-service slack")
    require(tuning, "TRACKER_OPTIONAL_CONSOLE_SERVICE_MIN_SLACK_US 2500UL", "console slack")
    require(tuning, "TRACKER_OPTIONAL_BACKGROUND_SERVICE_MIN_SLACK_US 3500UL", "background slack")

    # Profiler detail is sampled and fixed-memory, not another per-sample tax.
    require(tuning, "TRACKER_IMU_STAGE_PROFILER_SAMPLE_DIVISOR 64UL", "sampled stage profiler")
    require(pipeline_h, "RuntimeProfiler* runtimeProfiler", "optional profiler dependency")
    require(profiler_h, "enum class ImuStage", "IMU stage categories")
    require(profiler_h, "SampledStageStats imuStages_", "fixed stage storage")
    forbid(profiler_h, "std::vector", "heap stage telemetry")
    require(profiler, "perf_optional_service_admission_skips_", "admission-skip telemetry")
    require(profiler, "perf_imu_stage_sample_divisor", "stage divisor telemetry")
    require(profiler, "perf_imu_stage=", "stage timing telemetry")

    require(check_all, '("tools/test_pre_0024ac_imu_hotpath_slack_policy.py", "pre-0024ac IMU hotpath/slack policy")', "aggregate policy entry")
    require(testing, "pre-0024ac IMU hotpath/slack", "testing documentation")
    require(project, "pre-0024ac_imu_hotpath_and_slack_admission_hardening", "project status")

    cxx = compiler()
    base_warnings = ["-O2", "-Wall", "-Wextra", "-Wshadow", "-Werror"]
    with tempfile.TemporaryDirectory(prefix="tracker-pre0024ac-") as temp_name:
        tmp = Path(temp_name)
        compile_run(cxx, tmp / "temp", [
            "tests/native/test_gyro_temp_compensation.cpp",
            "src/sensor/gyro_temperature_compensation.cpp",
        ], base_warnings)
        compile_run(cxx, tmp / "bias", [
            "tests/native/test_runtime_bias_controller.cpp",
            "src/runtime/runtime_gyro_bias_controller.cpp",
            "src/sensor/ahrs_6dof.cpp",
            "src/sensor/calibration.cpp",
            "src/sensor/gyro_temperature_compensation.cpp",
            "src/sensor/imu_quality.cpp",
        ], ["-O1", "-g", "-Wall", "-Wextra", "-Werror",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer"])
        compile_run(cxx, tmp / "profiler", [
            "tests/native/test_runtime_profiler.cpp",
            "src/runtime/runtime_profiler.cpp",
        ], base_warnings)
        compile_run(cxx, tmp / "admission", [
            "tests/native/test_tracking_slack_admission.cpp",
        ], base_warnings)

        run([
            cxx, "-std=c++20", "-O2", "-Wall", "-Wextra", "-Wshadow", "-Werror",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
            "-c", str(ROOT / "src/runtime/imu_sample_pipeline.cpp"),
            "-o", str(tmp / "imu_sample_pipeline.o"),
        ])
        run([
            cxx, "-std=c++20", "-O2", "-Wall", "-Wextra", "-Wshadow", "-Werror",
            "-DARDUINO", "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
            "-c", str(ROOT / "src/app/tracker_app.cpp"),
            "-o", str(tmp / "tracker_app.o"),
        ])

        for source, define in (
            ("src/runtime/imu_sample_pipeline.cpp", None),
            ("src/app/tracker_app.cpp", "-DARDUINO"),
        ):
            cmd = [
                cxx, "-std=c++20", "-O2", "-fstack-usage",
                "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
            ]
            if define:
                cmd.append(define)
            cmd.extend([
                "-c", str(ROOT / source),
                "-o", str(tmp / (Path(source).stem + "_stack.o")),
            ])
            run(cmd)

        for symbol, ceiling in (
            ("imuSamplePipelineProcessRaw", 448),
            ("TrackerApp::loop", 256),
        ):
            measured = stack_usage(tmp, symbol)
            if measured > ceiling:
                raise SystemExit(f"{symbol} stack {measured} exceeds {ceiling}")

    print("# pre_0024ac_imu_hotpath_slack_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
