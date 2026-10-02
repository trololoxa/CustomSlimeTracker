#!/usr/bin/env python3
"""Guard progress_epochs suppress/resume epoch semantics and documentation."""

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
    native = text("tests/native/test_sensor_liveness_recovery.cpp")
    check_all = text("tools/check_all.py")

    require(progress, "IRQ/drain producers publish local", "local timestamp documentation")
    require(progress, "timestamp-free sequence", "sequence-edge documentation")
    require(native, "testSuppressedEpochProgressIsNotReused", "epoch-boundary regression")
    require(native, "resumed.lastAcceptedGyroAtUs == 0u", "discarded suppressed gyro proof")
    require(native, "resumed.lastOrientationAtUs == 0u", "discarded suppressed orientation proof")
    require(native, "SensorProgressFault::NoAcceptedGyro", "fresh gyro requirement")
    require(native, "SensorProgressFault::NoOrientationPublication", "fresh orientation requirement")

    cxx = compiler()
    env = quality_gate_environment(ROOT, scope="progress_epochs-progress-epoch")
    with project_temp_directory(ROOT, "tracker-progress_epochs-") as raw:
        tmp = Path(raw)
        exe = tmp / (
            "test_sensor_liveness_recovery.exe"
            if os.name == "nt"
            else "test_sensor_liveness_recovery"
        )
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

    print("# progress_epochs: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
