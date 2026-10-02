#!/usr/bin/env python3
"""Guard progress_clock_domains watchdog clock-domain separation and hot-path cost."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, require, run_contract_command,
)

import os

from quality_gate_runtime import project_temp_directory, quality_gate_environment


ROOT = Path(__file__).resolve().parents[2]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def main() -> int:
    progress = text("src/runtime/sensor_progress_watchdog.hpp")
    pipeline = text("src/runtime/imu_sample_pipeline.cpp")
    native = text("tests/native/test_sensor_liveness_recovery.cpp")
    check_all = text("tools/check_all.py")

    require(progress, "void noteAcceptedGyro()", "timestamp-free gyro progress edge")
    require(progress, "void noteOrientationPublication()", "timestamp-free orientation progress edge")
    require(progress, "observeProducerProgress(nowUs);", "watchdog-domain observation")
    require(progress, "acceptedGyroProgress_ != observedAcceptedGyroProgress_", "gyro sequence edge")
    require(progress, "orientationProgress_ != observedOrientationProgress_", "orientation sequence edge")
    producer_api = progress[progress.index("void noteAcceptedGyro()"):
                            progress.index("void setSuppressed")]
    forbid(producer_api, "micros()", "hot-path clock read")
    forbid(pipeline, "noteAcceptedGyro(raw.t_us)", "cross-domain gyro timestamp")
    forbid(pipeline, "noteOrientationPublication(raw.t_us)", "cross-domain orientation timestamp")
    require(pipeline, "noteAcceptedGyro();", "gyro progress wiring")
    require(pipeline, "noteOrientationPublication();", "orientation progress wiring")
    require(native, "testProducerProgressUsesWatchdogClockDomain", "clock-domain regression")

    cxx = compiler()
    env = quality_gate_environment(ROOT, scope="progress_clock_domains-progress-clock")
    with project_temp_directory(ROOT, "tracker-progress_clock_domains-") as raw:
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
                "-c", str(ROOT / "src/runtime/imu_sample_pipeline.cpp"),
                "-o", str(tmp / "imu_sample_pipeline.o"),
            ],
            cwd=ROOT,
            env=env,
            check=True,
        )


    print("# progress_clock_domains: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
