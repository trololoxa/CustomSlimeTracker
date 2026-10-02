#!/usr/bin/env python3
"""Guard recovery_feedback recovery feedback, tap reconfiguration and diagnostics."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, require, run_contract_command,
)

import os

from quality_gate_runtime import project_temp_directory, quality_gate_environment


ROOT = Path(__file__).resolve().parents[2]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


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
        raise SystemExit("recovery_feedback requires exactly one gated tap setup in hardware recovery")

    cxx = compiler()
    env = quality_gate_environment(ROOT, scope="recovery_feedback-recovery-feedback")
    with project_temp_directory(ROOT, "tracker-recovery_feedback-") as raw:
        tmp = Path(raw)
        exe = tmp / ("test_sensor_liveness_recovery.exe" if os.name == "nt" else "test_sensor_liveness_recovery")
        run_contract_command(
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
        run_contract_command([str(exe)], cwd=ROOT, env=env, check=True)
        for source, defines in (
            ("src/runtime/runtime_status_reporter.cpp", ()),
            ("src/app/tracker_app.cpp", ("-DARDUINO",)),
        ):
            obj = tmp / (Path(source).stem + ".o")
            run_contract_command(
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


    print("# recovery_feedback: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
