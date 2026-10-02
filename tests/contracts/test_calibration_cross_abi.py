#!/usr/bin/env python3
"""Guard calibration_cross_abi stack, aggregate-test and power-loss hardening."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import require

import subprocess
from unittest import mock

import check_all as check_all_module

ROOT = Path(__file__).resolve().parents[2]


def require_non_aborting_aggregation() -> None:
    """Verify the behavior instead of pinning one subprocess API spelling."""
    summary = check_all_module.CheckSummary()
    completed = subprocess.CompletedProcess(["synthetic-policy-failure"], 17)
    with mock.patch.object(
        check_all_module,
        "run_bounded_process",
        return_value=completed,
    ):
        passed = check_all_module.run_checked(
            summary,
            ["synthetic-policy-failure"],
            "synthetic policy failure",
        )
    if passed or summary.failures != ["synthetic policy failure (exit=17)"]:
        raise SystemExit("aggregate runner did not preserve a non-zero child result")


def main() -> int:
    controller_h = (ROOT / "src/runtime/calibration_autonomy_controller.hpp").read_text(encoding="utf-8")
    controller = (ROOT / "src/runtime/calibration_autonomy_controller.cpp").read_text(encoding="utf-8")
    native_test = (ROOT / "tests/native/test_calibration_autonomy.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")
    standalone = (ROOT / "tools/run_standalone_tests.py").read_text(encoding="utf-8")
    autonomy_policy = (ROOT / "tests/contracts/test_calibration_autonomy.py").read_text(encoding="utf-8")

    require(controller, "uint8_t bestIndex[6]", "compact best-session indices")
    require(controller, "uint8_t secondIndex[6]", "compact held-out-session indices")
    require(controller, "sessions_[secondIndex[face]]", "held-out lookup without Session copies")
    require(autonomy_policy, 'buildAccelProposal", 640', "cross-ABI 640-byte accel stack ceiling")

    require(check_all, "class CheckSummary", "aggregated check_all summary")
    require_non_aborting_aggregation()
    require(check_all, "check_all: FAIL (", "final consolidated failure report")
    require(standalone, "class NativeFailure", "native failure aggregation")
    require(standalone, "All independent compile/link/run stages are attempted", "runner continuation contract")
    require(standalone, "print_failure_summary", "native consolidated failure report")

    require(native_test, "testPowerLossDuringAcceptCleanup", "accept cleanup reboot coverage")
    require(native_test, "testPowerLossDuringRollbackCleanup", "rollback cleanup reboot coverage")

    print("# calibration_cross_abi: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
