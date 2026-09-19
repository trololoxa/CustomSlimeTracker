#!/usr/bin/env python3
"""Guard 0027d recovery feedback, tap reconfiguration and diagnostics."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from quality_gate_runtime import project_temp_directory, quality_gate_environment


ROOT = Path(__file__).resolve().parents[1]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(haystack: str, needle: str, label: str) -> None:
    if needle not in haystack:
        raise SystemExit(f"missing {label}: {needle}")


def compiler() -> str:
    explicit = os.environ.get("CXX")
    if explicit and shutil.which(explicit):
        return explicit
    for name in ("g++", "clang++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    raise SystemExit("no C++ compiler available for 0027d policy")


def main() -> int:
    progress = text("src/runtime/sensor_progress_watchdog.hpp")
    recovery = text("src/runtime/sensor_recovery_controller.hpp")
    app_h = text("src/app/tracker_app.hpp")
    app = text("src/app/tracker_app.cpp")
    hooks = text("src/app/hooks/tracker_app_runtime_hooks.hpp")
    command_hooks = text("src/app/hooks/tracker_app_command_hooks.hpp")
    tracking = text("src/runtime/tracking_state_controller.cpp")
    status = text("src/runtime/runtime_status_reporter.cpp")
    native = text("tests/native/test_sensor_liveness_recovery.cpp")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/0027d_recovery_feedback_and_tap_report.md")
    check_all = text("tools/check_all.py")

    require(progress, "sensorProgressOrientationExpected", "publication expectation policy")
    require(progress, "lastTriggeredFault", "persistent watchdog fault diagnosis")
    require(recovery, "sensorRecoveryReinitializesDevice", "full-device recovery classifier")
    require(app_h, "orientationPublicationExpected", "side-effect-free app callback")
    require(app, "sensorRecovery_.confirmProgress(millis())", "typed recovery completion")
    require(app, "sensorRecoveryReinitializesDevice(action)", "tap reconfiguration gate")
    require(hooks, "g_fifoInterruptAttached", "owned interrupt attachment state")
    require(hooks, "appOrientationPublicationExpected", "tracking publication contract wiring")
    require(command_hooks, "sensorRecoveryController", "recovery status wiring")
    require(tracking, "tracking_recovery_monotonic_gyro_samples", "monotonic gyro status")
    require(tracking, "tracking_degraded_gyro_output_allowed", "degraded output status")
    require(status, "sensor_progress_last_triggered_fault", "watchdog root-cause status")
    require(status, "sensor_recovery_request_count", "recovery episode status")
    require(native, "testOrientationPublicationContract", "publication contract regression")

    completion = app[app.index("bool TrackerApp::serviceSensorRuntimeRecovery"):app.index("void TrackerApp::reportRuntimeRecoveryFailure")]
    require(completion, "if (sensorRecoveryReinitializesDevice(action)) call(deps_.callbacks.setupTapRuntime)",
            "tap configured only after a full-device reinit")
    if completion.count("setupTapRuntime") != 1:
        raise SystemExit("0027d requires exactly one gated tap setup in hardware recovery")

    cxx = compiler()
    env = quality_gate_environment(ROOT, scope="0027d-recovery-feedback")
    with project_temp_directory(ROOT, "tracker-0027d-") as raw:
        tmp = Path(raw)
        exe = tmp / ("test_sensor_liveness_recovery.exe" if os.name == "nt" else "test_sensor_liveness_recovery")
        subprocess.run(
            [
                cxx,
                "-std=c++20",
                "-O2",
                "-Wall",
                "-Wextra",
                "-Wshadow",
                "-I", str(ROOT / "src"),
                "-I", str(ROOT / "tests/native"),
                str(ROOT / "tests/native/test_sensor_liveness_recovery.cpp"),
                "-o", str(exe),
            ],
            cwd=ROOT,
            env=env,
            check=True,
        )
        subprocess.run([str(exe)], cwd=ROOT, env=env, check=True)
        for source, defines in (
            ("src/runtime/runtime_status_reporter.cpp", ()),
            ("src/app/tracker_app.cpp", ("-DARDUINO",)),
        ):
            obj = tmp / (Path(source).stem + ".o")
            subprocess.run(
                [
                    cxx,
                    "-std=c++20",
                    "-O2",
                    "-Wall",
                    "-Wextra",
                    "-Wshadow",
                    *defines,
                    "-I", str(ROOT / "src"),
                    "-I", str(ROOT / "tests/native"),
                    "-c", str(ROOT / source),
                    "-o", str(obj),
                ],
                cwd=ROOT,
                env=env,
                check=True,
            )

    require(check_all, "test_0027d_recovery_feedback_and_tap_policy.py", "aggregate gate entry")
    require(testing, "0027d recovery feedback and tap reconfiguration", "testing documentation")
    require(project, "0027d_recovery_feedback_and_tap", "project status entry")
    require(report, "after 0027c", "additive patch ordering")

    print("# 0027d_recovery_feedback_and_tap_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
