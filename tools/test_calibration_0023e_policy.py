#!/usr/bin/env python3
"""Guard setup prompt, gyro quality and disabled-autonomy cold-path hardening."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def main() -> int:
    controller_h = (ROOT / "src/runtime/calibration_autonomy_controller.hpp").read_text(encoding="utf-8")
    controller = (ROOT / "src/runtime/calibration_autonomy_controller.cpp").read_text(encoding="utf-8")
    pipeline = (ROOT / "src/runtime/imu_sample_pipeline.cpp").read_text(encoding="utf-8")
    hooks = (ROOT / "src/app/hooks/tracker_app_runtime_hooks.hpp").read_text(encoding="utf-8")
    setup = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    config_cmd = (ROOT / "src/serial/tracker_config_commands.cpp").read_text(encoding="utf-8")
    serial_ctx = (ROOT / "src/serial/tracker_serial_context.hpp").read_text(encoding="utf-8")
    remote = (ROOT / "src/network/wifi_remote_console.cpp").read_text(encoding="utf-8")
    fifo_h = (ROOT / "src/sensor/fifo_calibrations.hpp").read_text(encoding="utf-8")
    fifo_cpp = (ROOT / "src/sensor/fifo_calibrations.cpp").read_text(encoding="utf-8")
    cal_cmd = (ROOT / "src/serial/tracker_calibration_commands.cpp").read_text(encoding="utf-8")
    autonomy_tests = (ROOT / "tests/native/test_calibration_autonomy.cpp").read_text(encoding="utf-8")
    cal_tests = (ROOT / "tests/native/test_sensor_calibration.cpp").read_text(encoding="utf-8")
    docs = (ROOT / "docs/calibration_autonomy.md").read_text(encoding="utf-8")
    testing = (ROOT / "docs/testing.md").read_text(encoding="utf-8")

    require(controller_h, "imuObservationRequired() const", "IMU learner admission API")
    require(controller_h, "deferredServiceRequired() const", "deferred learner admission API")
    require(pipeline, "deps.calibrationAutonomy->imuObservationRequired()", "sample-path cold gate")
    require(hooks, "if (!g_calibrationAutonomy.deferredServiceRequired()) return false;", "loop cold gate")
    gate_pos = hooks.index("if (!g_calibrationAutonomy.deferredServiceRequired()) return false;")
    millis_pos = hooks.index("g_calibrationAutonomy.service(millis())", gate_pos)
    if millis_pos < gate_pos:
        raise SystemExit("autonomy loop reads millis before disabled-service gate")
    if "calibrationContractChanged()" in controller or "trackerCalibrationPayloadRevision(*deps_.config)" in controller.split("void CalibrationAutonomyController::observeImuSample", 1)[1].split("bool CalibrationAutonomyController::windowLooksStationary", 1)[0]:
        raise SystemExit("calibration revision hashing remains in the IMU sample path")
    require(controller, "notifyCalibrationContractChanged", "explicit calibration epoch invalidation")
    require(config_cmd, "ConfigCalibrationOwnershipScope", "config mutation autonomy ownership")
    require(config_cmd, "notifyCalibrationContractChanged", "config apply epoch invalidation")
    require(controller, "autonomy_imu_hotpath_enabled=", "hot-path status diagnostic")
    require(controller, "autonomy_deferred_service_required=", "service status diagnostic")
    require(autonomy_tests, "testDisabledAutonomyHasColdHotPath", "disabled-autonomy native regression")
    require(autonomy_tests, "controller.stats().samplesObserved == 0u", "disabled learner no-work assertion")
    require(autonomy_tests, "controller.stats().serviceCalls == 0u", "disabled service no-work assertion")

    require(serial_ctx, "commandOutputNeedsExplicitFlush", "buffered command output flag")
    require(remote, "clientContext_.commandOutputNeedsExplicitFlush = true;", "remote setup flush wiring")
    require(setup, "s.flush();", "setup prompt flush")
    require(setup, "ctx.commandOutputNeedsExplicitFlush", "periodic blocking-command flush")

    require(fifo_h, "float maxGyroStdDps = 0.80f;", "hard raw gyro noise ceiling")
    require(fifo_h, "maxGyroMeanStdErrorDps", "fit mean precision gate")
    require(fifo_h, "maxValidationGyroMeanStdErrorDps", "held-out mean precision gate")
    require(fifo_h, "fifoGyroStartupCalibrationEvaluateQuality", "shared gyro quality evaluator")
    require(fifo_cpp, "standardError(result.gyroStdDps", "fit standard-error calculation")
    require(fifo_cpp, "result.validationGyroMeanStdErrorDps", "held-out standard-error calculation")
    require(cal_cmd, "train_mean_precision_gate=", "gyro gate diagnostics")
    require(cal_tests, "0.581890f", "reported hardware-noise regression fixture")
    require(cal_tests, "0.002848f", "reported held-out residual regression fixture")

    require(docs, "0023e setup prompt, gyro quality and cold hot path", "0023e design documentation")
    require(testing, "0023e setup and disabled-autonomy regression", "0023e acceptance documentation")

    print("# calibration_0023e_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
