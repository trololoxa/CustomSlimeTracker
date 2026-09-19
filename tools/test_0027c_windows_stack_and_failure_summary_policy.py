#!/usr/bin/env python3
"""Guard 0027c Windows yaw-stack and actionable check_all summaries."""

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
    raise SystemExit("no C++ compiler available for 0027c stack policy")


def stack_usage(directory: Path) -> dict[str, int]:
    usage: dict[str, int] = {}
    for path in directory.glob("*.su"):
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            parts = line.split("\t")
            if len(parts) < 2:
                continue
            try:
                size = int(parts[1])
            except ValueError:
                continue
            name = parts[0].split(":", 3)[-1]
            usage[name] = max(size, usage.get(name, 0))
    return usage


def measured(usage: dict[str, int], needle: str) -> int:
    matches = [size for name, size in usage.items() if needle in name]
    if not matches:
        raise SystemExit(f"0027c stack policy did not find function: {needle}")
    return max(matches)


def main() -> int:
    yaw_h = text("src/sensor/mag_yaw_correction.hpp")
    yaw = text("src/sensor/mag_yaw_correction.cpp")
    check_all = text("tools/check_all.py")
    aggregation = text("tools/test_check_all_aggregation_policy.py")
    fifo_test = text("tests/native/test_fifo_runtime_processor.cpp")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/0027c_windows_stack_and_failure_summary_report.md")

    phases = (
        "initializeUpdate",
        "evaluateMagValidity",
        "evaluateHorizontalTrust",
        "evaluateMotionTrust",
        "evaluateInnovation",
        "updateCooldown",
        "computeCorrection",
    )
    require(yaw, "TRACKER_MAG_YAW_NOINLINE", "yaw no-inline phase boundary")
    for phase in phases:
        require(yaw_h, phase, f"yaw phase declaration {phase}")
        require(yaw, f"MagYawCorrectionController::{phase}", f"yaw phase definition {phase}")

    require(check_all, "stderr=subprocess.PIPE", "child stderr capture")
    require(check_all, "failure += \"\\n\" + result.stderr.rstrip()", "failure detail retention")
    require(check_all, "print_aggregated_failure(failure)", "detailed final summary")
    require(aggregation, "root stack-budget cause", "summary detail regression")

    require(fifo_test, "class SilentTestStream final", "native negative-path log sink")
    require(fifo_test, "setFailDataRead(true)", "FIFO drain-failure injection")
    require(fifo_test, "pendingFault() == FifoRuntimeFault::DrainFailed", "FIFO fault assertion")

    cxx = compiler()
    target = subprocess.check_output([cxx, "-dumpmachine"], text=True).strip().lower()
    if "x86_64" in target or "amd64" in target:
        with project_temp_directory(ROOT, "tracker-0027c-ms-stack-") as raw:
            tmp = Path(raw)
            for optimization in ("-O2", "-Os"):
                obj = tmp / f"yaw-{optimization[1:]}.o"
                subprocess.run(
                    [
                        cxx,
                        "-std=c++20",
                        optimization,
                        "-mabi=ms",
                        "-fstack-usage",
                        "-I", str(ROOT / "src"),
                        "-I", str(ROOT / "tests/native"),
                        "-c", str(ROOT / "src/sensor/mag_yaw_correction.cpp"),
                        "-o", str(obj),
                    ],
                    cwd=ROOT,
                    env=quality_gate_environment(ROOT, scope="0027c-ms-stack"),
                    check=True,
                )
                usage = stack_usage(tmp)
                public = measured(
                    usage,
                    "MagYawCorrectionController::update(const tracker::MagYawCorrectionInputView",
                )
                helper_max = max(measured(usage, phase) for phase in phases)
                if public > 96:
                    raise SystemExit(
                        f"0027c Microsoft-ABI public yaw frame {public} exceeds 96"
                    )
                if helper_max > 96 or public + helper_max > 192:
                    raise SystemExit(
                        "0027c Microsoft-ABI nested yaw stack exceeds predecessor peak: "
                        f"public={public}, helper={helper_max}"
                    )

    require(check_all, "test_0027c_windows_stack_and_failure_summary_policy.py", "aggregate gate entry")
    require(testing, "0027c Windows stack and failure summary", "testing documentation")
    require(project, "0027c_windows_stack_and_failure_summary", "project status entry")
    require(report, "0027b", "additive patch ordering")

    print("# 0027c_windows_stack_and_failure_summary_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
