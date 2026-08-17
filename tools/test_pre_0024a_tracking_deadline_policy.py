#!/usr/bin/env python3
"""Guard pre-0024a deadline, FIFO-slice and deferred-mag semantics."""

from __future__ import annotations

import os
import re
import runpy
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
    raise SystemExit("no C++ compiler available for pre-0024a policy")


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
    app = text("src/app/tracker_app.cpp")
    app_h = text("src/app/tracker_app.hpp")
    hooks = text("src/app/hooks/tracker_app_runtime_hooks.hpp")
    slime_h = text("src/runtime/slimevr_output_runtime.hpp")
    slime = text("src/runtime/slimevr_output_runtime_impl.inc")
    fifo = text("src/runtime/fifo_runtime_processor.cpp")
    fifo_h = text("src/runtime/fifo_runtime_processor.hpp")
    tuning = text("src/build_config/tracking_tuning.hpp")
    mag = text("src/runtime/mag_runtime_controller.cpp")
    mag_h = text("src/runtime/mag_runtime_controller.hpp")
    axis_h = text("src/sensor/mag_axis_alignment.hpp")
    perf = text("src/serial/tracker_perf_commands.cpp")
    status = text("src/runtime/mag_status_reporter.cpp")
    check_all = text("tools/check_all.py")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/pre_0024a_tracking_deadline_hardening_report.md")

    # Tracking-quality invariants: no sample-history deletion, ODR reduction,
    # AHRS equation change or packet-rate reduction is permitted in this wave.
    require(tuning, "FIFO_RUNTIME_MAX_RAW_CALLBACKS_PER_SLICE = 64", "unchanged max raw slice")
    require(tuning, "FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE = 4", "bounded raw micro-batch")
    require(tuning, "FIFO_RUNTIME_URGENT_SPAN_US = 40000u", "age-based urgency threshold")
    for forbidden in ("dropOldest", "skipRaw", "discardRaw", "decimateGyro"):
        forbid(fifo, forbidden, f"sensor-history loss: {forbidden}")
    scheduler = isolate(app, "bool TrackerApp::processFifoRuntime()", "void TrackerApp::startMagFromConfig")
    forbid(scheduler, "resetWork();", "implicit FIFO reset")

    # O(1) deadline admission must prevent high-frequency no-op nested calls.
    require(app_h, "criticalNetworkRuntimeDue", "side-effect-free app gate")
    require(hooks, "criticalRotationServiceDue(millis())", "SlimeVR due hook")
    require(scheduler, "if (criticalNetworkDue)", "nested admission gate")
    due = isolate(slime, "bool SlimeVROutputRuntime::criticalRotationServiceDue", "bool SlimeVROutputRuntime::updateCritical")
    require(due, "nextRotationDeadlineMs_", "absolute rotation deadline")
    critical = isolate(slime, "bool SlimeVROutputRuntime::updateCritical", "bool SlimeVROutputRuntime::update(")
    require(critical, "if (!criticalRotationServiceDue(nowMs)) return false;", "critical fast rejection")
    require(critical, "const uint32_t dueBefore = rotationSendDue_;", "constant-time work detection")
    forbid(critical, "activitySignature()", "large status reduction in nested path")
    for forbidden in ("pollIncoming", "sendHeartbeat", "sendTelemetry", "sendSensorInfo", "wifi_->update"):
        forbid(critical, forbidden, f"noncritical nested work: {forbidden}")

    # Budget is checked before dequeuing another sample after four coherent
    # callbacks. Locate the loop relative to its stable scheduling/statistics
    # anchors rather than requiring the old unconditional profiler epilogue.
    loop_start = fifo.find("while (chronologicalReady)")
    loop_end = fifo.find("const uint32_t callbackElapsedUs", loop_start)
    if loop_start < 0 or loop_end < 0:
        raise SystemExit("FIFO chronological loop/sampled timing epilogue not found")
    loop = fifo[loop_start:loop_end]
    budget_pos = loop.find("rawCallbacks >= cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE")
    dequeue_pos = loop.find("dequeueRaw(raw, checkStats)")
    if budget_pos < 0 or dequeue_pos < 0 or budget_pos > dequeue_pos:
        raise SystemExit("FIFO budget must be checked before dequeuing another raw sample")
    require(fifo_h, "sliceBudgetOvershootEvents", "overshoot count")
    require(fifo_h, "sliceBudgetOvershootMaxUs", "overshoot maximum")
    require(fifo_h, "rawCallbackTimedSamples", "sampled raw callback timing")
    require(fifo_h, "magCallbackTimeCalls", "mag callback timing")
    require(fifo, "urgentByDepth() || urgentByAge()", "combined urgency")
    require(fifo, "diagnosticsTimingSampled_ = diagnosticsTimingEnabled_ &&", "dormant profiler timing")
    require(fifo, "const bool timeRawCallback = diagnosticsTimingSampled_ &&", "sampled callback timing")

    # Axis-alignment evidence leaves the chronological 60 Hz callback intact,
    # remains bounded, preserves FIFO order and rejects stale calibration epochs.
    process_mag = isolate(mag, "void MagRuntimeController::processRawSample", "Stream& MagRuntimeController::stream")
    require(process_mag, "enqueueAxisAlignmentEvidence(nowMs)", "cheap evidence enqueue")
    forbid(process_mag, "axisAlignmentCollector->observe", "axis reservoir work in mag callback")
    require(mag_h, "kAxisEvidenceCapacity = 8u", "bounded evidence queue")
    require(mag_h, "uint32_t configCrc = 0", "evidence calibration epoch")
    require(mag, "deps_.config->data.crc32 != evidence.configCrc", "stale evidence rejection")
    deferred = isolate(mag, "bool MagRuntimeController::serviceDeferred", "#undef TRACKER_MAG_RUNTIME_NOINLINE")
    require(deferred, "const bool evidencePending = axisEvidenceCount_ != 0u", "queued evidence admission")
    gate_pos = deferred.find("if (!deferredServiceAllowed(serviceGate))")
    evidence_service_pos = deferred.find("if (evidencePending) return serviceOneAxisAlignmentEvidence()")
    if gate_pos < 0 or evidence_service_pos < 0 or gate_pos > evidence_service_pos:
        raise SystemExit("axis evidence must obey the deferred FIFO/output admission gate")
    require(axis_h, "evidenceServiceDeferrals", "evidence admission deferral counter")
    require(axis_h, "evidenceQueued", "queued counter")
    require(axis_h, "evidenceProcessed", "processed counter")
    require(axis_h, "evidenceDropped", "overflow counter")
    require(axis_h, "evidenceStaleDropped", "stale epoch counter")
    require(perf, "runtime_mag_axis_evidence_stale_dropped_delta", "perf stale evidence telemetry")
    require(perf, "runtime_mag_axis_evidence_service_deferrals_delta", "perf evidence admission telemetry")
    require(status, "axis_evidence_stale_dropped", "mag status stale evidence telemetry")

    require(check_all, '("tools/test_pre_0024a_tracking_deadline_policy.py", "pre-0024a tracking deadline policy")', "aggregate entry")
    require(testing, "pre-0024a tracking deadline", "testing documentation")
    require(project, "pre-0024a_tracking_deadline_hardening", "project status")
    require(report, "No IMU sample is dropped", "quality-preservation report")

    cxx = compiler()
    runner = runpy.run_path(str(ROOT / "tools/run_standalone_tests.py"))
    project_sources = [str(path) for path in runner["PROJECT_SOURCES"]]
    base_flags = list(runner["BASE_FLAGS"])

    with project_temp_directory(ROOT, "tracker-pre0024a-") as temp_name:
        tmp = Path(temp_name)
        sanitizer_name, sanitizer_flags = strongest_supported_sanitizer_flags(cxx, ROOT)
        fifo_flags = ["-O1", "-g", "-Wall", "-Wextra", "-Werror"]
        if sanitizer_flags:
            print(f"# pre-0024a sanitizer={sanitizer_name}")
            fifo_flags.extend(sanitizer_flags)
        else:
            print("# pre-0024a sanitizer: SKIP (toolchain cannot link ASan/UBSan)")
        compile_run(cxx, tmp / "fifo", [
            "tests/native/test_fifo_runtime_processor.cpp",
            "src/runtime/fifo_runtime_processor.cpp",
            "src/connection/lsm6dsv_fifo.cpp",
            "src/connection/lsm6dsv_driver.cpp",
            "src/sensor/imu_quality.cpp",
        ], fifo_flags)

        # Full runtime linkage proves the actual deferred-controller boundary,
        # including config-store types and magnetic processing components.
        mag_sources = [
            *project_sources,
            "src/runtime/mag_runtime_controller.cpp",
            "src/connection/lsm6dsv_sensorhub.cpp",
            "src/sensor/qmc6309.cpp",
            "tests/native/policy_pre_0024a_mag_deferred.cpp",
        ]
        run([
            cxx, *base_flags,
            *[str(ROOT / source) for source in mag_sources],
            "-o", str(tmp / "mag_deferred"),
        ])
        run([str(tmp / "mag_deferred")])

        for source in (
            "src/runtime/fifo_runtime_processor.cpp",
            "src/runtime/mag_runtime_controller.cpp",
            "src/runtime/slimevr_output_runtime.cpp",
        ):
            run([
                cxx, "-std=c++20", "-O2", "-fstack-usage",
                "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
                "-c", str(ROOT / source), "-o", str(tmp / (Path(source).stem + ".o")),
            ])
        for symbol, ceiling in (
            ("FifoRuntimeProcessor::process", 384),
            ("MagRuntimeController::processRawSample", 1024),
            ("MagRuntimeController::serviceOneAxisAlignmentEvidence", 384),
            ("SlimeVROutputRuntime::updateCritical", 128),
        ):
            measured = stack_usage(tmp, symbol)
            if measured > ceiling:
                raise SystemExit(f"{symbol} stack {measured} exceeds {ceiling}")

    print("# pre_0024a_tracking_deadline_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
