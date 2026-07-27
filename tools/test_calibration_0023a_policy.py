#!/usr/bin/env python3
"""Guard the 0023a autonomy/setup calibration hardening contract."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def forbid(text: str, needle: str, description: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {description}: {needle}")


def main() -> int:
    controller = (ROOT / "src/runtime/calibration_autonomy_controller.cpp").read_text(encoding="utf-8")
    controller_h = (ROOT / "src/runtime/calibration_autonomy_controller.hpp").read_text(encoding="utf-8")
    autonomy_store = (ROOT / "src/runtime/calibration_autonomy_store.cpp").read_text(encoding="utf-8")
    autonomy_store_h = (ROOT / "src/runtime/calibration_autonomy_store.hpp").read_text(encoding="utf-8")
    config_store = (ROOT / "src/config/tracker_config_store.cpp").read_text(encoding="utf-8")
    calibration_cli = (ROOT / "src/serial/tracker_calibration_commands.cpp").read_text(encoding="utf-8")
    setup_cli = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    system_cli = (ROOT / "src/serial/tracker_system_commands.cpp").read_text(encoding="utf-8")
    mag_cli = (ROOT / "src/serial/tracker_mag_commands.cpp").read_text(encoding="utf-8")
    fifo_cal = (ROOT / "src/sensor/fifo_calibrations.cpp").read_text(encoding="utf-8")
    fifo_cal_h = (ROOT / "src/sensor/fifo_calibrations.hpp").read_text(encoding="utf-8")
    accel_h = (ROOT / "src/sensor/accel_6pos_calibration.hpp").read_text(encoding="utf-8")
    temp_fit = (ROOT / "src/runtime/gyro_temp_static_fit.cpp").read_text(encoding="utf-8")
    app = (ROOT / "src/app/tracker_app.cpp").read_text(encoding="utf-8")
    hooks = (ROOT / "src/app/hooks/tracker_app_runtime_hooks.hpp").read_text(encoding="utf-8")
    command_hooks = (ROOT / "src/app/hooks/tracker_app_command_hooks.hpp").read_text(encoding="utf-8")
    serial_context = (ROOT / "src/serial/tracker_serial_context.hpp").read_text(encoding="utf-8")
    setup_verifier = (ROOT / "src/runtime/setup_output_verifier.cpp").read_text(encoding="utf-8")
    feature = (ROOT / "src/build_config/firmware_feature_version.hpp").read_text(encoding="utf-8")

    require(feature, '"c3-6dsv-safe-calibration-autonomy-hardened"', "0023a feature identity")

    # Exact predecessor generation rollback, never a derived replacement save.
    require(config_store, "restoreAuthoritativeGeneration", "exact rollback API")
    require(controller, "journal_.previousSlot", "rollback slot journal")
    require(controller, "journal_.previousGeneration", "rollback generation journal")
    require(controller, "restoreAuthoritativeGeneration(", "exact rollback use")

    # Journal writes must update the verified in-RAM copy so multi-tick promotion progresses.
    require(autonomy_store_h, "writeJournal(CalibrationAutonomyJournalRecord& record)",
            "mutable verified journal API")
    require(autonomy_store, "if (ok) input = journalVerify_", "verified journal returned to controller")
    require(controller, "PromotionCommitPending", "split selector/probation transition")
    require(controller, "Candidate is now durable", "durable candidate before prepare state")
    require(controller, "completeRollbackSynchronously", "manual/setup synchronous rollback completion")
    require(controller, "journal_.state != CalibrationAutonomyJournalState::AcceptPending",
            "split accept cleanup journal stage")
    require(controller, "journal_.state != CalibrationAutonomyJournalState::RollbackPending",
            "split rollback write-ahead stage")
    require(controller, "confirmAuthoritativeConfigApplied", "promotion runtime-apply acknowledgement")
    require(controller, "Ownership must survive", "post-promotion autonomy candidate ownership")

    # Manual/setup owns calibration synchronously and invalidates old evidence.
    require(controller_h, "beginManualCalibration", "manual ownership API")
    require(controller_h, "endManualCalibration", "manual ownership release")
    require(calibration_cli, "CalibrationManualOwnershipScope", "all mutating cal command ownership")
    require(setup_cli, "setup calibration blocked: autonomous calibration transaction could not be resolved",
            "setup autonomy preflight")
    require(setup_cli, "beginManualCalibration(millis())", "setup ownership acquisition")
    require(mag_cli, "MagCalibrationOwnershipScope", "manual mag calibration ownership")
    require(mag_cli, "magCommandMutatesCalibration", "mag calibration mutation classification")

    # Complete persistent calibration erase is explicit and preserves ordinary policy.
    require(calibration_cli, 'is(argv[1], "erase_all")', "full erase command")
    require(calibration_cli, 'is(argv[2], "confirm")', "destructive erase confirmation")
    require(calibration_cli, "clearAllCalibrationPreservingPolicy", "policy-preserving calibration erase")
    require(calibration_cli, "configStore->erase()", "slot/candidate erase")
    require(calibration_cli, "forceClearPersistentCalibrationStateForErase", "journal/rejection erase")
    require(system_cli, "cal erase_all confirm", "full erase help")

    # Initial gyro capture: continuous evidence + held-out validation, no extra pose.
    require(fifo_cal_h, "validationSamples", "gyro/accel held-out sample configuration")
    require(fifo_cal_h, "maxValidationBiasErrorDps", "gyro held-out bias gate")
    require(fifo_cal, "resetContinuousWindow", "continuous stationary gyro window")
    require(fifo_cal, "validationResidualDps", "gyro held-out comparison")
    require(fifo_cal, "maxTemperatureSpanC", "gyro capture temperature-span gate")

    # Same six ordinary faces, automatically followed by held-out samples.
    require(accel_h, "INDEPENDENT_VALIDATION_FAILED", "accel independent validation flag")
    require(fifo_cal, "captureValidationFace", "automatic held-out face capture")
    require(fifo_cal, "validationDetection.face != result.detectedFace", "same-face held-out proof")
    require(fifo_cal, "maxValidationAxisResidualG", "all-face held-out residual gate")
    forbid(setup_cli, "45 degree", "new exact diagonal pose requirement")

    # Temperature fit must be validated without fitting the held-out temperature bin.
    require(temp_fit, "leaveOneBinOutValidation", "temperature leave-one-bin-out validation")
    require(temp_fit, "validationMaxDps", "temperature worst-bin gate")

    # Identity mapping is never silently accepted and tap is not calibration readiness.
    require(setup_cli, "Type identity explicitly", "explicit identity instruction")
    require(setup_cli, "A blank line aborts", "blank identity rejection")
    require(setup_cli, "bool production() const { return calibration9dof() && slimevr(); }",
            "tap-independent production calibration readiness")


    # Final setup verification uses the already-prepared coherent output pair;
    # it requires no diagonal pose and blocks commit on real output faults.
    require(setup_cli, "setupVerifyOutputRuntime", "final setup output verification")
    require(setup_cli, "setup verify", "standalone setup verify command")
    require(setup_cli, "No diagonal or precisely measured angle is required",
            "user-friendly arbitrary-face verification")
    require(setup_cli, "verify_output", "setup transaction verification rollback")
    require(setup_verifier, "linearAccelerationRmsG", "final linear-acceleration residual gate")
    require(setup_verifier, "maximumQuaternionStepDeg", "final quaternion continuity gate")
    require(serial_context, "PreparedOutputRuntime* preparedOutput", "prepared output command wiring")
    require(command_hooks, "objects.preparedOutput = &g_preparedOutput", "prepared output runtime binding")
    require(setup_cli, "blank mag axis mapping is not accepted",
            "blank manual axis mapping rejection")
    require(setup_cli, 'if (is(line, "identity"))', "explicit identity branch")

    # Autonomous transactions keep the device awake and candidate polling is throttled.
    require(controller_h, "blocksMotionLightSleep", "sleep transaction blocker")
    require(app, "calibrationBlocksMotionSleep", "app sleep admission integration")
    require(hooks, "calibrationBlocksMotionSleep", "runtime hook wiring")
    require(controller_h, "kCandidateAbsentProbeIntervalMs", "absent-candidate NVS probe backoff")

    print("# calibration_0023a_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
