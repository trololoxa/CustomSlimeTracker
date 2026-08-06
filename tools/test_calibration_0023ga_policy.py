#!/usr/bin/env python3
"""Guard 0023ga cross-ABI guided-axis stack hardening."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def main() -> int:
    setup = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    base_policy = (ROOT / "tools/test_calibration_0023g_policy.py").read_text(encoding="utf-8")
    project = (ROOT / "docs/project_status.md").read_text(encoding="utf-8")
    testing = (ROOT / "docs/testing.md").read_text(encoding="utf-8")
    report = (ROOT / "docs/0023ga_axis_alignment_stack_hardening_report.md").read_text(encoding="utf-8")

    require(setup, "#define TRACKER_SETUP_NOINLINE __attribute__((noinline))", "GCC/Clang noinline boundary")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupAutoSolveMagAxis", "static solver isolation")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupAutoSolveMagAxisDynamic", "dynamic solver isolation")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupTryApplyDynamicAxisAlignment", "dynamic apply isolation")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupTryApplyStaticAxisAlignment", "static apply isolation")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupPromptManualAxisMapping", "manual prompt isolation")
    require(setup, "TRACKER_SETUP_NOINLINE bool setupRunAxisAlignment", "orchestrator isolation")

    require(base_policy, 'require_limit(usage, "setupRunAxisAlignment", 512)', "reduced orchestrator ceiling")
    require(base_policy, 'require_limit(usage, "setupTryApplyDynamicAxisAlignment", 768)', "dynamic helper ceiling")
    require(base_policy, 'require_limit(usage, "setupAutoSolveMagAxis", 768)', "solver ceiling")
    require(project, "0023ga_axis_alignment_stack_hardening", "project status entry")
    require(testing, "0023ga cross-ABI guided-axis stack regression", "testing entry")
    require(report, "1056 bytes", "recorded MSYS2 failure")

    # Predecessors are independent entries in check_all.py. Re-running them
    # here made the aggregate gate quadratic and added no coverage.
    print("# calibration_0023ga_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
