#!/usr/bin/env python3
"""Guard axis_alignment_stack cross-ABI guided-axis stack hardening."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import require


ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    setup = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    base_policy = (ROOT / "tests/contracts/test_magnetic_coverage_reservoir.py").read_text(encoding="utf-8")

    require(setup, "#define TRACKER_SETUP_NOINLINE __attribute__((noinline))", "GCC/Clang noinline boundary")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupAutoSolveMagAxis", "static solver isolation")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupAutoSolveMagAxisDynamic", "dynamic solver isolation")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupTryApplyDynamicAxisAlignment", "dynamic apply isolation")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupTryApplyStaticAxisAlignment", "static apply isolation")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupPromptManualAxisMapping", "manual prompt isolation")

    require(base_policy, 'require_limit(usage, "setupRunAxisAlignment", 512)', "reduced orchestrator ceiling")
    require(base_policy, 'require_limit(usage, "setupTryApplyDynamicAxisAlignment", 768)', "dynamic helper ceiling")
    require(base_policy, 'require_limit(usage, "setupAutoSolveMagAxis", 768)', "solver ceiling")

    # Predecessors are independent entries in check_all.py. Re-running them
    # here made the aggregate gate quadratic and added no coverage.
    print("# axis_alignment_stack: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
