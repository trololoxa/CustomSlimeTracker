#!/usr/bin/env python3
"""Guard 0027e watchdog clock-domain separation and hot-path cost."""

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


def forbid(haystack: str, needle: str, label: str) -> None:
    if needle in haystack:
        raise SystemExit(f"forbidden {label}: {needle}")


def compiler() -> str:
    explicit = os.environ.get("CXX")
    if explicit and shutil.which(explicit):
        return explicit
    for name in ("g++", "clang++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    raise SystemExit("no C++ compiler available for 0027e policy")


def main() -> int:
    progress = text("src/runtime/sensor_progress_watchdog.hpp")
    pipeline = text("src/runtime/imu_sample_pipeline.cpp")
    native = text("tests/native/test_sensor_liveness_recovery.cpp")
    check_all = text("tools/check_all.py")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/0027e_progress_clock_domain_report.md")

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
    env = quality_gate_environment(ROOT, scope="0027e-progress-clock")
    with project_temp_directory(ROOT, "tracker-0027e-") as raw:
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
                "-c", str(ROOT / "src/runtime/imu_sample_pipeline.cpp"),
                "-o", str(tmp / "imu_sample_pipeline.o"),
            ],
            cwd=ROOT,
            env=env,
            check=True,
        )

    require(check_all, "test_0027e_progress_clock_domain_policy.py", "aggregate gate entry")
    require(testing, "0027e progress clock-domain separation", "testing documentation")
    require(project, "0027e_progress_clock_domain", "project status entry")
    require(report, "after 0027d", "additive patch ordering")

    print("# 0027e_progress_clock_domain_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
