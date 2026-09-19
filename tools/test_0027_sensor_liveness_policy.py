#!/usr/bin/env python3
"""Focused source-contract checks for patch 0027 sensor recovery."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(text: str, token: str, label: str) -> None:
    if token not in text:
        raise AssertionError(f"missing {label}: {token}")


def forbid(text: str, token: str, label: str) -> None:
    if token in text:
        raise AssertionError(f"forbidden {label}: {token}")


def main() -> int:
    health = read("src/runtime/tracker_health_state.hpp")
    progress = read("src/runtime/sensor_progress_watchdog.hpp")
    recovery = read("src/runtime/sensor_recovery_controller.hpp")
    boot = read("src/runtime/boot_health.hpp")
    profile_contract = read("src/build_config/profile_contract.hpp")
    app = read("src/app/tracker_app.cpp")
    hardware = read("src/app/tracker_hardware_context.hpp")
    fifo = read("src/connection/lsm6dsv_fifo.cpp")
    processor = read("src/runtime/fifo_runtime_processor.cpp")
    pipeline = read("src/runtime/imu_sample_pipeline.cpp")
    runtime_types = read("src/runtime/tracker_runtime_types.hpp")
    network_commands = read("src/serial/tracker_network_commands.cpp")
    setup_commands = read("src/serial/tracker_setup_commands.cpp")
    compat_commands = read("src/serial/tracker_slimevr_serial_compat_commands.cpp")
    tests = read("tests/native/test_sensor_liveness_recovery.cpp")
    fifo_tests = read("tests/native/test_fifo_runtime_processor.cpp")
    pair_tests = read("tests/native/test_fifo_pair_coherency.cpp")
    config_tests = read("tests/native/test_config_storage.cpp")

    for code in (
        "ImuNoProgress",
        "FifoDrainFailed",
        "FifoResetFailed",
        "FifoDiscontinuity",
        "SpiPlausibilityFailed",
        "RecoveryExhausted",
    ):
        require(health, code, f"typed health code {code}")

    for token in (
        "lastIrqAtUs_",
        "lastDrainAtUs_",
        "lastAcceptedGyroAtUs_",
        "lastOrientationAtUs_",
        "IntentionalSleep",
        "BlockingScan",
        "SensorReinit",
        "static_cast<uint32_t>(nowUs - base)",
    ):
        require(progress, token, "progress watchdog contract")

    require(recovery, "kAttemptsBeforeExhausted", "bounded recovery attempts")
    require(recovery, "kExhaustedRetryMs", "bounded exhausted backoff")
    require(recovery, "deadlineReached", "wrap-safe recovery deadline")
    require(boot, "sequenceInv", "retained record inverse")
    require(boot, "crc32", "retained record CRC")
    require(boot, "kSafeModeCrashBoots", "safe-mode crash threshold")
    require(
        profile_contract,
        "Sensor progress liveness requires TRACKER_ENABLE_PREPARED_OUTPUT_SNAPSHOT=1",
        "prepared-output liveness profile contract",
    )

    require(fifo, "Stop all FIFO routes", "transaction start")
    require(fifo, "Software epoch changes only", "hardware-before-software commit")
    require(fifo, "ReconfigureVerify", "register read-back failure")
    require(processor, "recoveryQuarantined_ = true", "old-epoch quarantine")
    require(processor, "rawCount > rawFree || magCount > magFree", "whole-batch preflight")
    require(pipeline, "degradedGyroOutputAllowed", "bounded gyro-only publication")
    require(pipeline, "ImuQualityResult recoveryQuality = quality", "recovery-only quality copy")
    forbid(pipeline, "ImuQualityResult outputQuality = quality", "normal-path quality copy")
    require(runtime_types, "RECOVERY_DEGRADED", "stale acceleration marker")

    require(app, "esp_task_wdt_add", "task watchdog registration")
    if app.count("esp_task_wdt_reset()") != 1:
        raise AssertionError("task watchdog must have exactly one app-loop feed site")
    require(app, "feedTaskWatchdogAfterMandatoryLoop();", "mandatory-loop feed")
    require(app, "esp_task_wdt_delete", "intentional-sleep watchdog suspension")
    require(app, "setSafeModeWriteInhibit", "safe-mode write gate")
    if app.index("setSafeModeWriteInhibit(safeModeActive_)") > app.index(
        "trackerBootstrapLoadConfigAndApplyRuntime"
    ):
        raise AssertionError("safe-mode gate must precede config repair/migration load")
    require(app, "bounded probes continue", "exhausted recovery remains live")
    recovery_start = app.index("bool TrackerApp::performTransactionalFifoRecovery()")
    recovery_end = app.index("bool TrackerApp::performFullSensorReinit()", recovery_start)
    recovery_body = app[recovery_start:recovery_end]
    hardware_reset = recovery_body.index("deps_.runtime.fifo->resetFifo()")
    event_reset = recovery_body.index("deps_.runtime.fifoEvents->reset()")
    interrupt_attach = recovery_body.index("attachFifoInterrupt")
    if not hardware_reset < event_reset < interrupt_attach:
        raise AssertionError(
            "FIFO event epoch must reset after verified hardware reset and before interrupt attach"
        )
    require(network_commands, "trackerRecoverSensorStreamAfterBlockingWifiScan(ctx, \"net_scan\")", "network scan recovery")
    require(setup_commands, "trackerRecoverSensorStreamAfterBlockingWifiScan(ctx, \"setup_wifi_scan\")", "setup scan recovery")
    require(compat_commands, "trackerRecoverSensorStreamAfterBlockingWifiScan(ctx, \"get_wifiscan\")", "compat scan recovery")

    isr_start = hardware.index("static void IRAM_ATTR onFifoInt1()")
    isr_end = hardware.index("}", isr_start)
    forbid(hardware[isr_start:isr_end], "micros", "clock read in FIFO ISR")
    forbid(hardware, "g_fifoLastIrqUs", "write-only IRQ timestamp")

    for fixed_header in (progress, recovery, boot):
        for token in ("<vector>", "<memory>", "sqrt", "acos"):
            forbid(fixed_header, token, "dynamic/expensive recovery mechanism")

    for token in (
        "testProgressSuppressionAndWrap",
        "testBoundedRecoveryOrderAndTerminalBackoff",
        "testBootHealthRejectsRandomAndClearsAfterStable",
    ):
        require(tests, token, "0027 native acceptance")
    require(fifo_tests, "testWholeBatchCapacityPreflight", "batch atomicity regression")
    require(fifo_tests, "testDrainFailureRequestsRecoveryWithoutPublishing", "drain failure regression")
    require(fifo_tests, "testEventSourceRebasesPreexistingIrqCount", "IRQ epoch regression")
    require(pair_tests, "testResetReadbackFailureKeepsSoftwareEpoch", "reset rollback regression")
    require(pair_tests, "testEveryResetTransportStepTerminatesAndRetries", "per-step recovery faults")
    require(config_tests, "testSafeModeLoadPerformsNoRepairOrMigrationWrites", "read-only safe-mode load")

    print("PASS 0027 sensor liveness/recovery policy")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
