#!/usr/bin/env python3
"""Guard pre-0024ac IMU hotpath reuse and optional-service admission."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

from quality_gate_runtime import (
    asan_ubsan_environment,
    project_temp_directory,
    strongest_supported_sanitizer_flags,
)

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
    subprocess.run(cmd, cwd=ROOT, check=True, env=asan_ubsan_environment(ROOT))


def compile_run(cxx: str, exe: Path, sources: list[str], flags: list[str]) -> None:
    run([
        cxx, "-std=c++20", *flags,
        "-I", str(ROOT / "src"),
        "-I", str(ROOT / "tests/native"),
        *[str(ROOT / source) for source in sources],
        str(ROOT / "tests/native/sanitizer_runtime_options.cpp"),
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
    serial_h = text("src/serial/tracker_serial_commands.hpp")
    temp_h = text("src/sensor/gyro_temperature_compensation.hpp")
    temp = text("src/sensor/gyro_temperature_compensation.cpp")
    bias = text("src/runtime/runtime_gyro_bias_controller.cpp")
    command_hooks = text("src/app/hooks/tracker_app_command_hooks.hpp")
    runtime_hooks = text("src/app/hooks/tracker_app_runtime_hooks.hpp")
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
    require(runtime_hooks, "tempEval, currentGyroBiasRadS", "shared logger hook inputs")
    require(command_hooks, "const GyroTempCompRuntimeEval& tempEval", "logger temp-eval input")
    require(command_hooks, "const Vec3& currentGyroBiasRadS", "logger current-bias input")
    forbid(
        command_hooks,
        "currentGyroBiasRadS(calibrated.temp_c)",
        "duplicate logger current-bias evaluation",
    )
    forbid(
        command_hooks,
        "gyroBiasRuntimeFlags(calibrated.temp_c)",
        "duplicate logger temp evaluation",
    )

    # Disabled serial streaming must not read micros() on every IMU sample.
    output = isolate(
        pipeline,
        "bool imuPipelineEmitPerSampleOutputs(ImuSamplePipelineDeps& deps,",
        "FifoRuntimeSampleResult imuSamplePipelineProcessRaw",
    )
    require(
        output,
        "deps.logState == nullptr || deps.logState->accepting()",
        "logger acceptance gate before callback",
    )
    mode_pos = output.find("deps.streamState->mode != TrackerStreamMode::Off")
    micros_pos = output.find("micros()")
    if mode_pos < 0 or micros_pos < 0 or mode_pos > micros_pos:
        raise SystemExit("serial stream mode must be checked before the hotpath micros() read")

    # Optional services require empty/non-urgent FIFO and deadline slack. One
    # completed background worker consumes the loop's background slot. The
    # deferred machine-log serializer takes a fresh snapshot after background
    # work in a separate bounded stack frame instead of inflating loop().
    require(admission, "input.fifoUrgent || input.softwareQueuePending", "FIFO admission veto")
    require(admission, "trackingBackgroundRuntimeAdmitted", "single background-slot helper")
    require(admission, "!backgroundSlotConsumed", "background phase-collision prevention")
    loop = isolate(app, "void TrackerApp::loop()", "bool TrackerApp::ready() const")
    if loop.count("trackingSlackAdmissionInput()") != 2:
        raise SystemExit("ordinary app loop must take exactly two shared tracking-phase slack snapshots")
    for snapshot in ("preNetworkSlack", "postCriticalSlack"):
        require(loop, f"const TrackingSlackAdmissionInput {snapshot}", f"{snapshot} snapshot")
    require(loop, "trackingBackgroundRuntimeAdmitted(\n        postCriticalSlack, magDeferredWorked)", "autonomy waits after actual mag work")
    require(loop, "serviceMachineLogRuntimeWithAdmission()", "out-of-line machine-log service")
    require(loop, "serviceConsoleRuntime(", "out-of-line bounded console phase")
    require(loop, "updateDiagnosticTimingActivation()", "out-of-line diagnostic activation")
    console = isolate(
        app,
        "bool TrackerApp::serviceConsoleRuntime(uint32_t* loopTimingUs)",
        "bool TrackerApp::serviceMachineLogRuntimeWithAdmission()",
    )
    require(console, "trackingSlackAdmissionInput()", "fresh console slack snapshot")
    require(console, "TrackingOptionalServiceClass::Console", "remote console admission")
    require(console, "updateRemoteConsoleRuntime", "bounded TCP console service")
    require(console, "updateSerialConsoleRuntime", "bounded USB console service")
    machine_log = isolate(
        app,
        "bool TrackerApp::serviceMachineLogRuntimeWithAdmission()",
        "TrackingSlackAdmissionInput TrackerApp::trackingSlackAdmissionInput() const",
    )
    require(machine_log, "trackingSlackAdmissionInput()", "fresh machine-log slack snapshot")
    require(machine_log, "TrackingOptionalServiceClass::Console", "machine-log console-class admission")
    require(machine_log, "callBool(deps_.callbacks.updateMachineLogRuntime)", "bounded deferred machine-log service")
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
    require(serial_h, "TRACKER_SERIAL_NOINLINE size_t poll", "out-of-line bounded CLI parser")

    require(check_all, '("tools/test_pre_0024ac_imu_hotpath_slack_policy.py", "pre-0024ac IMU hotpath/slack policy")', "aggregate policy entry")
    require(testing, "pre-0024ac IMU hotpath/slack", "testing documentation")
    require(project, "pre-0024ac_imu_hotpath_and_slack_admission_hardening", "project status")

    cxx = compiler()
    base_warnings = ["-O2", "-Wall", "-Wextra", "-Wshadow", "-Werror"]
    with project_temp_directory(ROOT, "tracker-pre0024ac-") as temp_name:
        tmp = Path(temp_name)
        compile_run(cxx, tmp / "temp", [
            "tests/native/test_gyro_temp_compensation.cpp",
            "src/sensor/gyro_temperature_compensation.cpp",
        ], base_warnings)
        sanitizer_name, sanitizer_flags = strongest_supported_sanitizer_flags(cxx, ROOT)
        bias_flags = ["-O1", "-g", "-Wall", "-Wextra", "-Werror"]
        if sanitizer_flags:
            print(f"# pre-0024ac sanitizer={sanitizer_name}")
            bias_flags.extend(sanitizer_flags)
        else:
            print("# pre-0024ac sanitizer: SKIP (toolchain cannot link ASan/UBSan)")
        compile_run(cxx, tmp / "bias", [
            "tests/native/test_runtime_bias_controller.cpp",
            "src/runtime/runtime_gyro_bias_controller.cpp",
            "src/sensor/ahrs_6dof.cpp",
            "src/sensor/calibration.cpp",
            "src/sensor/gyro_temperature_compensation.cpp",
            "src/sensor/imu_quality.cpp",
        ], bias_flags)
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
            ("TrackerApp::serviceConsoleRuntime", 160),
            ("TrackerApp::serviceMachineLogRuntimeWithAdmission", 96),
            ("TrackerApp::updateDiagnosticTimingActivation", 64),
        ):
            measured = stack_usage(tmp, symbol)
            if measured > ceiling:
                raise SystemExit(f"{symbol} stack {measured} exceeds {ceiling}")

    print("# pre_0024ac_imu_hotpath_slack_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
