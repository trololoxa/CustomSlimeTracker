#!/usr/bin/env python3
"""Guard 0023ge post-audit realtime and diagnostic hardening contracts."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

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
    raise SystemExit("no C++ compiler available for 0023ge policy")


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
        raise SystemExit(f"0023ge stack policy did not find function: {needle}")
    worst_name, worst_size = max(matches, key=lambda item: item[1])
    if worst_size > limit:
        raise SystemExit(
            f"0023ge stack budget exceeded: {worst_name} uses {worst_size} bytes, limit {limit}"
        )


def main() -> int:
    fifo_h = (ROOT / "src/runtime/fifo_runtime_processor.hpp").read_text(encoding="utf-8")
    fifo = (ROOT / "src/runtime/fifo_runtime_processor.cpp").read_text(encoding="utf-8")
    mag_h = (ROOT / "src/sensor/mag_runtime.hpp").read_text(encoding="utf-8")
    mag = (ROOT / "src/sensor/mag_runtime.cpp").read_text(encoding="utf-8")
    frame = (ROOT / "src/sensor/frame_transform.hpp").read_text(encoding="utf-8")
    controller = (ROOT / "src/runtime/mag_runtime_controller.cpp").read_text(encoding="utf-8")
    cal = (ROOT / "src/sensor/mag_calibration.cpp").read_text(encoding="utf-8")
    fifo_test = (ROOT / "tests/native/test_fifo_runtime_processor.cpp").read_text(encoding="utf-8")
    mag_test = (ROOT / "tests/native/test_mag_calibration.cpp").read_text(encoding="utf-8")
    heading_test = (ROOT / "tests/native/test_mag_heading_reliability.cpp").read_text(encoding="utf-8")
    status = (ROOT / "src/runtime/runtime_status_reporter.cpp").read_text(encoding="utf-8")
    perf = (ROOT / "src/serial/tracker_perf_commands.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")

    # Chronological dispatch must obey both count and cooperative time budgets.
    require(fifo_h, "magCallbackCountDeferrals", "mag callback count deferral diagnostics")
    require(fifo_h, "magCallbackBudgetDeferrals", "mag callback time-budget diagnostics")
    require(fifo_h, "peekMagTimestamp", "timestamp-only queue peek")
    forbid(fifo_h, "bool peekMag(Lsm6dsvFifoReader::MagRawSample&", "full magnetic sample copy on every raw callback")
    require(fifo, "bool chronologicalReady = true", "fail-closed backlog gate")
    require(fifo, "chronologicalReady = false", "raw timeline stop on due magnetic backlog")
    require(fifo, "micros() - callbackSliceStartUs", "mag callback cooperative time budget")
    require(fifo, "queueStats_.magCallbackBudgetDeferrals++", "time-budget counter")
    require(fifo, "while (chronologicalReady)", "raw advancement loop gated by magnetic chronology")
    require(fifo, "if (!dispatchDueMagCallbacks(lastDispatchedRawTimestampUs_", "per-sample magnetic chronology gate")
    require(fifo, "queueStats_.magChronologicalDeferrals++", "chronology deferral accounting")
    require(fifo_test, "testMagCallbacksRespectCountBudget", "mag callback count-budget regression")
    require(fifo_test, "testMagCallbacksRespectCooperativeTimeBudget", "expensive magnetic burst regression")
    require(fifo_test, "rawAfterFirstPass", "backlog raw-timeline freeze assertion")

    # One sensor-to-device SO(3) validation per magnetic sample, reused for gyro.
    require(mag_h, "sensorToDeviceApplied", "retained frame-validation decision")
    forbid(mag_h, "const MagProcessedSample& last() const", "stale duplicate last-sample API")
    forbid(mag_h, "MagProcessedSample last_", "stale duplicate last-sample storage")
    require(mag, "out.sensorToDeviceApplied = frame.enabled", "runtime frame decision capture")
    forbid(frame, "inverseApplyValidatedSensorToDevice", "public unchecked inverse helper")
    require(controller, "inverseApplyAcceptedSensorToDevice", "controller-local inverse mapping")
    require(controller, "processed.sensorToDeviceApplied", "controller reuse of processor decision")
    forbid(controller, "const SensorToDeviceFrame frame = makeSensorToDeviceFrame(", "duplicate controller SO(3) validation")
    require(heading_test, "testMagRuntimeRetainsValidatedDeviceFrameDecision", "frame reuse regression")

    # Every post-normalization physical rejection must retain useful diagnostics.
    require(cal, "Normalization and retained-sample diagnostics are already authoritative", "pre-solve diagnostic preservation")
    require(cal, "out.solverStage = MagCalibrationSolverStage::Normalized", "normalized stage before physical gates")
    require(cal, "replaceFitFromAccumulator", "isolated refit candidate stack phase")
    forbid((ROOT / "src/sensor/mag_calibration.hpp").read_text(encoding="utf-8"), "MagCalibrationResult compute();", "unused by-value fit result API")
    require(mag_test, "physical pre-solve coverage rejection", "early-failure diagnostics regression")

    require(status, "fifo_runtime_mag_budget_deferrals=", "runtime budget diagnostics")
    require(perf, "runtime_mag_budget_deferrals_delta=", "perf budget diagnostics")
    require(check_all, '("tools/test_calibration_0023ge_policy.py", "0023ge magnetometer audit hardening policy")', "aggregate runner entry")

    cxx = compiler()
    with tempfile.TemporaryDirectory(prefix="tracker-0023ge-") as tmp_name:
        tmp = Path(tmp_name)
        common = [
            cxx, "-std=c++20", "-O2", "-fstack-usage",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
        ]
        for source, output in (
            ("src/runtime/fifo_runtime_processor.cpp", "fifo.o"),
            ("src/runtime/mag_runtime_controller.cpp", "controller.o"),
            ("src/sensor/mag_runtime.cpp", "mag_runtime.o"),
            ("src/sensor/mag_calibration.cpp", "mag_calibration.o"),
        ):
            subprocess.run([*common, "-c", str(ROOT / source), "-o", str(tmp / output)], check=True)
        usage = parse_stack_usage(tmp)
        require_limit(usage, "FifoRuntimeProcessor::process", 512)
        require_limit(usage, "dispatchDueMagCallbacks", 256)
        require_limit(usage, "MagRuntimeController::processRawSample", 1024)
        require_limit(usage, "MagRuntimeProcessor::process", 256)
        require_limit(usage, "MagCalibrationCollector::compute", 1536)
        require_limit(usage, "replaceFitFromAccumulator", 512)

    subprocess.run([sys.executable, str(ROOT / "tools/test_calibration_0023gd_policy.py")], check=True)
    print("# calibration_0023ge_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
