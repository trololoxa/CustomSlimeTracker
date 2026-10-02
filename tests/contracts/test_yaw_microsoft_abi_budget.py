#!/usr/bin/env python3
"""Guard yaw_microsoft_abi_budget Windows yaw-stack and actionable check_all summaries."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, parse_stack_usage as stack_usage, require, run_contract_command,
)

import subprocess

from quality_gate_runtime import project_temp_directory, quality_gate_environment


ROOT = Path(__file__).resolve().parents[2]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def measured(usage: dict[str, int], needle: str) -> int:
    matches = [size for name, size in usage.items() if needle in name]
    if not matches:
        raise SystemExit(f"yaw_microsoft_abi_budget stack policy did not find function: {needle}")
    return max(matches)


def main() -> int:
    yaw_h = text("src/sensor/mag_yaw_correction.hpp")
    yaw = text("src/sensor/mag_yaw_correction.cpp")
    check_all = text("tools/check_all.py")
    aggregation = text("tests/tooling/test_check_all_aggregation.py")
    fifo_test = text("tests/native/test_fifo_runtime_processor.cpp")

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
    target = run_contract_command([cxx, "-dumpmachine"], stdout=subprocess.PIPE,
                                  text=True, timeout_s=30).stdout.strip().lower()
    if "x86_64" in target or "amd64" in target:
        with project_temp_directory(ROOT, "tracker-yaw_microsoft_abi_budget-ms-stack-") as raw:
            tmp = Path(raw)
            for optimization in ("-O2", "-Os"):
                obj = tmp / f"yaw-{optimization[1:]}.o"
                run_contract_command(
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
                    env=quality_gate_environment(ROOT, scope="yaw_microsoft_abi_budget-ms-stack"),
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
                        f"yaw_microsoft_abi_budget Microsoft-ABI public yaw frame {public} exceeds 96"
                    )
                if helper_max > 96 or public + helper_max > 192:
                    raise SystemExit(
                        "yaw_microsoft_abi_budget Microsoft-ABI nested yaw stack exceeds predecessor peak: "
                        f"public={public}, helper={helper_max}"
                    )


    print("# yaw_microsoft_abi_budget: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
