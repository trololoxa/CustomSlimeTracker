#!/usr/bin/env python3
"""Guard additive 0028c bounded capture and tolerant verification contracts."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {label}: {needle}")


def main() -> int:
    fifo_h = read("src/sensor/fifo_calibrations.hpp")
    fifo = read("src/sensor/fifo_calibrations.cpp")
    cancel = read("src/serial/tracker_calibration_capture_cancel.hpp")
    setup = read("src/serial/tracker_setup_commands.cpp")
    verifier_h = read("src/runtime/setup_output_verifier.hpp")
    verifier = read("src/runtime/setup_output_verifier.cpp")
    output_h = read("src/runtime/output_runtime.hpp")
    output = read("src/runtime/output_runtime.cpp")
    autonomy = read("src/runtime/calibration_autonomy_controller.cpp")
    mag_heading = read("src/sensor/mag_heading.cpp")
    accel = read("src/sensor/accel_6pos_calibration.cpp")
    config = read("src/config/tracker_config_runtime.cpp")
    bootstrap = read("src/app/tracker_bootstrap.cpp")
    native_cal = read("tests/native/test_sensor_calibration.cpp")
    native_verify = read("tests/native/test_setup_output_verifier.cpp")
    check_all = read("tools/check_all.py")
    report = read("docs/0028c_bounded_calibration_recovery_report.md")

    for needle, label in (
        ("maximumCaptureMs", "absolute capture deadline"),
        ("maximumConsecutiveWaitTimeouts", "bounded timeout streak"),
        ("FifoCalibrationCaptureStatus", "typed capture terminal state"),
        ("cancelRequested", "cooperative cancellation hook"),
    ):
        require(fifo_h, needle, label)
    for needle, label in (
        ("now() - startedMs_ >= maximumMs_", "wrap-safe capture deadline check"),
        ("FifoCalibrationCaptureStatus::SensorUnavailable", "timeout terminal state"),
        ("FifoCalibrationCaptureStatus::DrainFailed", "drain terminal state"),
        ("FifoCalibrationCaptureStatus::Cancelled", "cancel terminal state"),
    ):
        require(fifo, needle, label)
    require(cancel, "TrackerCalibrationCaptureCancelScope", "non-recursive command capture cancel owner")
    require(cancel, "value == 'q' || value == 'Q'", "explicit q cancellation")

    require(output_h, "copyCoherent", "coherent invalid-snapshot inspection API")
    require(output, "return copyCoherent(out) && out.valid", "ordinary consumer validity contract")
    for needle, label in (
        ("maximumInvalidSnapshots", "bounded invalid publication budget"),
        ("maximumStaleSnapshots", "bounded stale publication budget"),
        ("maximumCoherentReadFailures", "bounded seqlock read budget"),
        ("maximumSnapshotAgeUs", "explicit snapshot freshness limit"),
    ):
        require(verifier_h, needle, label)
    require(verifier, "staleSnapshots_++", "stale evidence exclusion")
    require(verifier, "qNormSq <= MATH_EPSILON", "near-zero quaternion hard rejection")
    require(setup, "recoverableStreamEvents <= recoverableStreamBudget", "bounded stream-event policy")
    require(setup, "fatalStreamFailure", "separate fatal stream policy")
    require(setup, "retrying once with fresh evidence", "single automatic clean-window retry")

    require(autonomy, "rejectionRetryDelayMs", "reason-aware rejection retry deadline")
    require(autonomy, "rejectionWrittenThisBoot_", "boot-local uptime ownership")
    require(autonomy, "nowMs - rejection_.rejectedUptimeMs", "wrap-safe rejection age")
    require(mag_heading, "qWorldFromBody.tryNormalized", "fail-closed heading quaternion")
    require(accel, "INVALID_VALIDATION_PARAMS", "calibration divisor/input validation")
    require(accel, "s[2] != '\\0'", "exact face token validation")
    require(config, "data.hardware.serialBaud != cfg::SERIAL_BAUD", "non-dead serial baud contract")
    require(config, "nominalDtUs * 0.25f", "expected-dt lower compatibility bound")
    require(config, "nominalDtUs * 4.0f", "expected-dt upper compatibility bound")
    require(bootstrap, "Local debug streaming is intentionally session-scoped",
            "non-autostart debug performance contract")

    require(native_cal, "testFifoCaptureWaitsAreBoundedAndCancellable", "bounded capture regression")
    require(native_verify, "maximumStaleSnapshots", "recoverable verification regression")
    ast.parse(check_all, filename="tools/check_all.py")
    entry = '("tools/test_0028c_bounded_calibration_recovery_policy.py", "0028c bounded calibration/recovery")'
    require(check_all, entry, "aggregate 0028c gate")
    require(report, "Not verified", "honest target verification section")
    print("# 0028c_bounded_calibration_recovery_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
