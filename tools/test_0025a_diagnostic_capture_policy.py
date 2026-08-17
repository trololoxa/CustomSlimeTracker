#!/usr/bin/env python3
"""Guard 0025a diagnostic capture, lifecycle, and hot-path invariants."""

from __future__ import annotations

import re
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


def main() -> int:
    tuning = read("src/build_config/network_tuning.hpp")
    remote_h = read("src/network/wifi_remote_console.hpp")
    remote = read("src/network/wifi_remote_console.cpp")
    app = read("src/app/tracker_app.cpp")
    hooks = read("src/app/hooks/tracker_app_runtime_hooks.hpp")
    machine = read("src/runtime/machine_log_runtime.cpp")
    pipeline = read("src/runtime/imu_sample_pipeline.cpp")
    capture = read("tools/capture_telnet_log.py")
    validator = read("tools/replay/logver3_contract.py")
    tests = read("tools/test_capture_telnet_log.py") + read("tools/test_logver3_contract.py")
    testing = read("docs/testing.md")
    project = read("docs/project_status.md")
    runner = read("tools/run_standalone_tests.py")

    require(tuning, "TRACKER_REMOTE_CONSOLE_SESSION_LEASE_MS 30000UL", "bounded TCP lease")
    require(remote_h, "WifiRemoteConsoleSessionLease sessionLease_", "lease state")
    require(remote, "sessionLease_.noteActivity(nowMs)", "input renews lease")
    require(remote, "sessionLease_.expired", "firmware lease enforcement")
    require(remote, "clientContext_.closeCommandSession", "disconnect cleanup hook")
    require(runner, 'pathlib.Path("src/network/wifi_remote_console.cpp")', "enabled TCP compile gate")
    require(app, "remoteConsoleBlocksMotionSleep(millis())", "preflight sleep blocker")
    require(hooks, "sessionBlocksMotionSleep(nowMs)", "sleep/lease composition")
    require(capture, "TELNET_IAC_NOP = b\"\\xff\\xf1\"", "Telnet NOP keepalive")
    require(capture, "KEEPALIVE_INTERVAL_S = 5.0", "bounded host keepalive cadence")
    require(capture, 'transport.send("test stop")', "failure-path test cleanup")
    require(capture, 'for cleanup in ("log finish", "log off")', "failure-path log cleanup")

    require(machine, "LOGVER,3,E1", "E1 schema")
    require(machine, "LOGFMT,NET", "network schema")
    require(machine, "LOGFMT,TESTSUM", "immutable test summary schema")
    require(machine, "machineLogNetworkDue", "one-hertz NET cadence")
    require(machine, "const TrackerWifiManagerStatus wifiStatus", "deferred Wi-Fi snapshot")
    require(machine, "const SlimeVROutputRuntimeStatus slimeStatus", "deferred UDP snapshot")
    due = machine.index("if (!machineLogNetworkDue")
    wifi_copy = machine.index("const TrackerWifiManagerStatus wifiStatus", due)
    if wifi_copy < due:
        raise SystemExit("network status is copied before the one-hertz due gate")

    # Every unquoted CSV declaration must fit both the 512-byte USB staging
    # record and the larger TCP staging record, including CRLF.
    declarations = re.findall(r'out\.println\("(LOGFMT,[^"]+)"\);', machine)
    if not declarations:
        raise SystemExit("no LOGFMT declarations found")
    oversized = [(line.split(",", 2)[1], len(line) + 2) for line in declarations if len(line) + 2 > 512]
    if oversized:
        raise SystemExit(f"LOGFMT exceeds USB record staging: {oversized}")

    require(pipeline, "deps.logState == nullptr || deps.logState->accepting()", "early logger gate")
    require(pipeline, "tempEval, currentGyroBiasRadS", "shared bias/temp values")
    forbid(pipeline, "gyroTempComp.snapshot", "extra logger snapshot in IMU path")

    require(capture, 'choices=("static", "runtime")', "static/runtime capture modes")
    require(capture, "promote_validated_capture_bundle", "pair-consistent promotion")
    require(capture, "Attempt both restorations independently", "independent rollback")
    require(capture, "Backups are deliberately not removed here", "recoverable rollback backup")
    require(capture, 'f"test summary {args.capture}"', "post-window exact summary")
    require(capture, 'if args.capture == "static":', "strict static preflight")
    require(validator, "health_passed", "diagnostic health result")
    require(validator, 'capture_kind == "static"', "static fail-closed semantics")
    require(validator, "NET counters show Wi-Fi/UDP/deadline failures", "UDP health contract")
    require(validator, "last_udp_error is intentionally historical", "stale UDP errno semantics")
    require(validator, "MAG semantic trust/heading contract failed", "MAG semantic contract")
    require(validator, "YAW semantic gate/apply contract failed", "YAW semantic contract")
    require(validator, "temperature BIAS source lacks", "BIAS semantic contract")

    for evidence in (
        "test_second_destination_replace_failure_rolls_back_pair",
        "test_incomplete_rollback_restores_other_file_and_retains_backup",
        "test_runtime_capture_preserves_udp_health_failure",
        "test_stale_udp_errno_after_recovery_does_not_fail_clean_window",
        "test_yaw_gate_loss_after_ready_row_fails",
        "test_lease_expiration_fails",
        "test_untrusted_mag_fails",
    ):
        require(tests, evidence, f"regression {evidence}")

    require(testing, "0025a diagnostic capture", "testing documentation")
    require(project, "0025a_cable_free_diagnostic_capture_hardening", "project status")
    print("# 0025a_diagnostic_capture_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
