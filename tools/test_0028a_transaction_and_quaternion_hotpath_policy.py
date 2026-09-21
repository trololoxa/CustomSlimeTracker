#!/usr/bin/env python3
"""Guard additive 0028a transaction recovery and quaternion hot-path contracts."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {label}: {needle}")


def forbid(text: str, needle: str, label: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {label}: {needle}")


def between(text: str, start: str, end: str, label: str) -> str:
    first = text.find(start)
    if first < 0:
        raise SystemExit(f"missing {label} start: {start}")
    last = text.find(end, first + len(start))
    if last < 0:
        raise SystemExit(f"missing {label} end: {end}")
    return text[first:last]


def main() -> int:
    ahrs = read("src/sensor/ahrs_6dof.cpp")
    config = read("src/config/tracker_config_runtime.cpp")
    storage = read("src/config/tracker_config_storage.cpp")
    store = read("src/config/tracker_config_store.cpp")
    fifo_control = read("src/serial/tracker_fifo_config_control.cpp")
    mag = read("src/runtime/mag_runtime_controller.cpp")
    reset_h = read("src/config/factory_reset_coordinator.hpp")
    reset = read("src/config/factory_reset_coordinator.cpp")
    app = read("src/app/tracker_app.cpp")
    commands = read("src/serial/tracker_serial_commands.cpp")
    autonomy = read("src/runtime/calibration_autonomy_controller.cpp")
    quality = read("src/sensor/imu_quality.cpp")
    check_all = read("tools/check_all.py")
    report = read("docs/0028a_transaction_recovery_and_quaternion_hotpath_report.md")

    update = between(ahrs, "bool Ahrs6Dof::update(const Vec3& gyroRadS,", "Ahrs6DofConfig Ahrs6Dof::sanitizeConfig", "AHRS update")
    require(update, "if (!predicted.isFinite())", "per-sample non-finite quaternion rejection")
    if update.count("predicted.normSq()") != 1:
        raise SystemExit("production path regained redundant per-sample predicted norm")
    require(update, "const float correctedNormSq = q_.normSq()", "decimated correction norm validation")
    require(update, "accelCorrectionSamples_ == 0u", "decimated quaternion norm boundary")
    require(update, "#if TRACKER_BUILD_IS_SLIM", "size-profile quaternion path")
    require(update, "return true;", "valid gyro propagation survives invalid accel correction")
    require(update, "normalizeInPlace()", "periodic quaternion norm proof")
    require(ahrs, "recoverPredictionAfterInvalidQuaternion", "out-of-line invalid-prediction recovery")
    require(ahrs, "initialized_ = false", "degenerate quaternion fail-closed state")
    for external in ("Ahrs6Dof::reset(", "Ahrs6Dof::setQuaternion("):
        block = between(ahrs, external, "\n}", external)
        require(block, "tryNormalized", f"strict external quaternion admission {external}")

    for limit in (
        "HARDWARE_FIFO_WORD_CAPACITY",
        "MAX_TIMESTAMP_WAITING_SAMPLES",
        "calibration_limits::ACCEL_MAX_ABS_BIAS_G",
        "calibration_limits::GYRO_STARTUP_MAX_NORM_RAD_S",
        "calibration_limits::MAG_MAX_AXIS_RATIO",
    ):
        require(config, limit, f"derived finite bound {limit}")
    require(storage, "record.version == tracker_config_storage_detail::SLOT_VERSION", "current slot version branch")
    require(storage, "!config.validateSemanticConfig()", "semantic current-slot validation")
    require(store, "record->version != tracker_config_storage_detail::SLOT_VERSION", "explicit non-current-slot migration")

    for text, owner in ((fifo_control, "IMU/FIFO"), (mag, "mag")):
        require(text, "prepareAuthoritativeCommit", f"{owner} inactive-slot prepare")
        require(text, "commitPreparedAuthoritative", f"{owner} selector commit")
        require(text, "abortPreparedAuthoritative", f"{owner} prepared-slot abort")
    forbid(fifo_control, "restorePersistedPrevious", "rollback rewrite of old config")
    forbid(mag, "previous mag config restore failed", "rollback rewrite of old mag config")

    for error in ("MarkerInvalid", "PendingScopeMismatch"):
        require(reset_h, error, f"factory reset error {error}")
        require(reset, f"FactoryResetError::{error}", f"factory reset handling {error}")
    require(app, "factory_reset_recovery_mode=yes", "fail-closed boot recovery mode")
    recovery = between(app, "if (factoryResetRecoveryActive_) {", "// Safe mode must inhibit", "factory reset setup gate")
    forbid(recovery, "setupNetworkRuntime", "network startup during partial reset recovery")
    require(app, "FACTORY_RESET_RECOVERY_MAX_ATTEMPTS", "bounded reset retries")
    require(app, "setFactoryResetRecoveryOnly(true)", "restricted reset recovery console")
    require(commands, "if (ctx.factoryResetRecoveryOnly)", "recovery-only dispatcher gate")
    require(commands, "command unavailable during factory reset recovery", "unsafe recovery command rejection")

    sensor_fault = between(autonomy, "if (sensorFailed) {", "if (transportFailed) {", "sensor probation fault")
    require(sensor_fault, "window_.reset()", "sensor-only evidence reset")
    transport_fault = between(autonomy, "if (transportFailed) {", "if (!sensorFailed && probationCanAccept", "transport probation fault")
    forbid(transport_fault, "window_.reset()", "transport erasing sensor evidence")
    require(transport_fault, "probationTransportVerified_ = false", "separate transport verdict")
    require(autonomy, "probationTransportUnverifiedAccepts", "unverified transport acceptance diagnostic")

    require(quality, "TIMESTAMP_SMALL_GAP", "small-gap diagnostic")
    require(quality, "counters_.smallGapSamples++", "small-gap counter")

    ast.parse(check_all, filename="tools/check_all.py")
    entry = '("tools/test_0028a_transaction_and_quaternion_hotpath_policy.py", "0028a transaction/quaternion hot-path hardening")'
    from check_all import TOOL_CHECKS
    if ast.literal_eval(entry) not in TOOL_CHECKS:
        raise SystemExit("required policy missing from aggregate TOOL_CHECKS registry")
    require(report, "Not verified", "honest unverified report section")

    print("# 0028a_transaction_and_quaternion_hotpath_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
