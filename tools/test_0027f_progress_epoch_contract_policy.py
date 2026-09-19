#!/usr/bin/env python3
"""Guard 0027f suppress/resume epoch semantics and documentation."""

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
    raise SystemExit("no C++ compiler available for 0027f policy")


def main() -> int:
    progress = text("src/runtime/sensor_progress_watchdog.hpp")
    native = text("tests/native/test_sensor_liveness_recovery.cpp")
    check_all = text("tools/check_all.py")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/0027f_progress_epoch_contract_report.md")

    require(progress, "IRQ/drain producers publish local", "local timestamp documentation")
    require(progress, "timestamp-free sequence", "sequence-edge documentation")
    require(native, "testSuppressedEpochProgressIsNotReused", "epoch-boundary regression")
    require(native, "resumed.lastAcceptedGyroAtUs == 0u", "discarded suppressed gyro proof")
    require(native, "resumed.lastOrientationAtUs == 0u", "discarded suppressed orientation proof")
    require(native, "SensorProgressFault::NoAcceptedGyro", "fresh gyro requirement")
    require(native, "SensorProgressFault::NoOrientationPublication", "fresh orientation requirement")
    require(check_all, "test_0027f_progress_epoch_contract_policy.py", "aggregate gate entry")
    require(testing, "0027f progress epoch contract", "testing documentation")
    require(project, "0027f_progress_epoch_contract", "project status entry")
    require(report, "additive after 0027e", "additive patch ordering")
    require(report, "production executable behavior", "runtime scope boundary")

    cxx = compiler()
    env = quality_gate_environment(ROOT, scope="0027f-progress-epoch")
    with project_temp_directory(ROOT, "tracker-0027f-") as raw:
        tmp = Path(raw)
        exe = tmp / (
            "test_sensor_liveness_recovery.exe"
            if os.name == "nt"
            else "test_sensor_liveness_recovery"
        )
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

    print("# 0027f_progress_epoch_contract_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
