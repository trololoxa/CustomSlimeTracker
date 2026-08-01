#!/usr/bin/env python3
"""Guard pre-0024 hotpath headroom/freshness foundation semantics."""

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
    raise SystemExit("no C++ compiler available for pre-0024 policy")


def compile_run(cxx: str, tmp: Path, name: str, sources: list[str], extra: list[str]) -> None:
    exe = tmp / name
    subprocess.run(
        [
            cxx, "-std=c++20", *extra,
            "-I", str(ROOT / "src"),
            "-I", str(ROOT / "tests/native"),
            *[str(ROOT / source) for source in sources],
            "-o", str(exe),
        ],
        check=True,
    )
    subprocess.run([str(exe)], check=True)


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
    hooks = text("src/app/hooks/tracker_app_runtime_hooks.hpp")
    fifo_h = text("src/runtime/fifo_runtime_processor.hpp")
    fifo = text("src/runtime/fifo_runtime_processor.cpp")
    slime_h = text("src/runtime/slimevr_output_runtime.hpp")
    slime = text("src/runtime/slimevr_output_runtime_impl.inc")
    battery = text("src/runtime/battery_adc_batch_sampler.cpp")
    motion = text("src/runtime/runtime_motion_diagnostics.cpp")
    tuning = text("src/build_config/runtime_tuning.hpp")
    tracking_tuning = text("src/build_config/tracking_tuning.hpp")
    bias = text("src/runtime/runtime_gyro_bias_controller.cpp")
    autonomy = text("src/runtime/calibration_autonomy_controller.cpp")
    profiler = text("src/runtime/runtime_profiler.cpp")
    profiler_h = text("src/runtime/runtime_profiler.hpp")
    remote = text("src/network/wifi_remote_console.cpp")
    wifi = text("src/network/esp32_wifi_station.cpp")
    check_all = text("tools/check_all.py")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/pre_0024_hotpath_headroom_foundation_report.md")

    # Tracking/product invariants: this foundation may schedule work, but it
    # must not reduce IMU/AHRS/output rates or discard sensor history.
    require(tracking_tuning, "FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE", "bounded minimum raw progress")
    require(tracking_tuning, "FIFO_RUNTIME_MAX_RAW_CALLBACKS_PER_SLICE = 64", "unchanged maximum raw slice")
    forbid(fifo, "dropOldest", "raw-sample dropping")
    forbid(fifo, "skipRaw", "raw-sample skipping")
    fifo_start = app.find("bool TrackerApp::processFifoRuntime()")
    if fifo_start < 0:
        raise SystemExit("FIFO scheduler not found")
    fifo_scheduler = app[fifo_start:]
    forbid(fifo_scheduler, "resetWork();", "implicit FIFO reset in scheduler")
    require(fifo_h, "YieldRequested", "typed deferred-work yield")

    # Hardware drain and callbacks share a real absolute budget.
    require(fifo, "const uint32_t fifoProcessStartUs = micros();", "budget before hardware drain")
    require(fifo, "micros() - fifoProcessStartUs", "absolute FIFO elapsed time")
    require(fifo, "nearDeadlineReducedDrains", "near-deadline drain bounding")
    require(fifo, "sliceBudgetStops", "slice budget stop telemetry")
    require(app, "rotationDeadlineSlackUs", "pose-deadline reservation")
    require(app, "kEmergencyFifoSliceUs = 750u", "bounded emergency FIFO progress")

    # Nested catch-up is rotation-only and cannot run full network lifecycle.
    nested = isolate(fifo_scheduler, "const bool networkWorked =", "} while (deps_.runtime.fifoRuntime->hasPendingWork());")
    require(nested, "updateCriticalNetworkRuntime", "narrow nested transport hook")
    forbid(nested, "updateNetworkRuntime", "full network update in FIFO catch-up")
    critical = isolate(slime, "bool SlimeVROutputRuntime::updateCritical", "\nbool SlimeVROutputRuntime::update(")
    require(critical, "maybeSendRotation(nowMs);", "critical rotation delivery")
    for forbidden in ("pollIncoming", "sendHeartbeat", "sendSensorInfo", "sendTelemetry", "wifi_->update"):
        forbid(critical, forbidden, f"noncritical work in updateCritical: {forbidden}")

    # Battery estimator preserves the old sorted trimmed mean while spreading
    # ADC conversions and insertion work over bounded services.
    require(tuning, "TRACKER_BATTERY_ADC_READS_PER_SERVICE 2u", "bounded ADC service")
    require(battery, "insertSorted(millivolts);", "incremental deterministic sorting")
    require(battery, "performed < maxReads", "bounded ADC loop")
    forbid(hooks, "uint16_t reads[kMaxReads]", "old stack ADC burst")

    # Diagnostics remain exact for event counters while expensive metrics are sampled.
    require(tuning, "TRACKER_MOTION_DIAGNOSTICS_SAMPLE_DIVISOR 16UL", "sampled motion diagnostics")
    require(motion, "++stats_.samples;", "exact total sample counter")
    require(motion, "++stats_.metricSamples;", "sampled metric counter")
    exact_prefix = isolate(motion, "++stats_.samples;", "const uint32_t divisor")
    require(exact_prefix, "estimatedDroppedSamples", "exact drop accounting before sampling")
    require(exact_prefix, "fifoRecoveryRequests", "exact recovery accounting before sampling")

    # Runtime-bias and autonomy math are deferred without changing evidence.
    require(bias, "bias.completedCalibratedGyroRadS = bias.calibratedGyroRadS", "exact bias accumulator copy")
    require(bias, "runtimeBiasFinalizePendingWindow", "deferred bias finalizer")
    require(fifo, "if (yieldRequested) break;", "yield before next raw sample")
    require(app, "updateHotpathDeferredRuntime", "bias finalization between FIFO slices")
    require(autonomy, "completedWindow_ = window_;", "exact autonomy accumulator copy")
    require(autonomy, "finalizeWindow(completed, completedNowMs);", "deferred autonomy finalization")

    # Telemetry is bounded and honest about software-only age.
    require(profiler_h, "RuntimeLatencyHistogram", "fixed profiler histogram")
    require(profiler_h, "Hardware-FIFO residence", "software-age limitation documentation")
    require(profiler, "perf_profiler_overhead", "profiler self-overhead telemetry")
    require(profiler, "perf_frame_headroom_p05_us", "frame headroom telemetry")
    require(profiler, "perf_rotation_software_age", "rotation software-age telemetry")
    require(fifo_h, "#if TRACKER_HAS_RUNTIME_PROFILER", "DIAG-only queue timestamp storage")
    forbid(profiler_h, "std::vector", "heap histogram")
    forbid(profiler_h, "new ", "heap profiler allocation")

    # Idle taxes are reduced without changing tap cadence.
    require(remote, "TRACKER_REMOTE_CONSOLE_ACCEPT_POLL_INTERVAL_MS", "bounded remote accept polling")
    forbid(remote, "candidate.flush()", "blocking remote busy flush")
    require(wifi, "TRACKER_WIFI_DIAGNOSTIC_INFO_REFRESH_MS", "cached slow Wi-Fi diagnostics")
    require(tuning, "#define TRACKER_TAP_POLL_INTERVAL_MS 5", "unchanged tap polling contract")

    require(check_all, '("tools/test_pre_0024_hotpath_headroom_policy.py", "pre-0024 hotpath headroom policy")', "aggregate policy entry")
    require(testing, "pre-0024 hotpath headroom/freshness regression", "testing documentation")
    require(project, "pre-0024_hotpath_headroom_foundation", "project status entry")
    require(report, "tracking equations", "quality-preservation report")

    cxx = compiler()
    with tempfile.TemporaryDirectory(prefix="tracker-pre0024-") as temp_name:
        tmp = Path(temp_name)
        compile_run(cxx, tmp, "profiler", [
            "tests/native/test_runtime_profiler.cpp",
            "src/runtime/runtime_profiler.cpp",
        ], ["-O2", "-Wall", "-Wextra", "-Werror"])
        compile_run(cxx, tmp, "motion", [
            "tests/native/test_runtime_motion_diagnostics.cpp",
            "src/runtime/runtime_motion_diagnostics.cpp",
            "src/sensor/imu_quality.cpp",
        ], ["-O2", "-Wall", "-Wextra", "-Werror"])
        compile_run(cxx, tmp, "battery", [
            "tests/native/test_battery_runtime.cpp",
            "src/runtime/battery_runtime.cpp",
            "src/runtime/battery_adc_batch_sampler.cpp",
        ], ["-O2", "-Wall", "-Wextra", "-Werror"])
        compile_run(cxx, tmp, "bias", [
            "tests/native/test_runtime_bias_controller.cpp",
            "src/runtime/runtime_gyro_bias_controller.cpp",
            "src/sensor/ahrs_6dof.cpp",
            "src/sensor/calibration.cpp",
            "src/sensor/gyro_temperature_compensation.cpp",
            "src/sensor/imu_quality.cpp",
        ], ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer"])

        for source in (
            "src/runtime/fifo_runtime_processor.cpp",
            "src/runtime/runtime_gyro_bias_controller.cpp",
            "src/runtime/calibration_autonomy_controller.cpp",
        ):
            subprocess.run([
                cxx, "-std=c++20", "-O2", "-fstack-usage",
                "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
                "-c", str(ROOT / source), "-o", str(tmp / (Path(source).stem + ".o")),
            ], check=True)
        for symbol, ceiling in (
            ("FifoRuntimeProcessor::process", 256),
            ("runtimeBiasFinalizePendingWindow", 256),
            ("CalibrationAutonomyController::service", 512),
        ):
            measured = stack_usage(tmp, symbol)
            if measured > ceiling:
                raise SystemExit(f"{symbol} stack {measured} exceeds {ceiling}")

    print("# pre_0024_hotpath_headroom_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
