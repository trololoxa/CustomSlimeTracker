#!/usr/bin/env python3
"""Guard the additive 0027b Windows-host and hot-path stack hardening."""

from __future__ import annotations

import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(haystack: str, needle: str, label: str) -> None:
    if needle not in haystack:
        raise SystemExit(f"missing {label}: {needle}")


def forbid(haystack: str, needle: str, label: str) -> None:
    if needle in haystack:
        raise SystemExit(f"forbidden {label}: {needle}")


def main() -> int:
    remote = text("src/network/wifi_remote_console.cpp")
    runner = text("tools/run_standalone_tests.py")
    field_h = text("src/sensor/mag_field_reliability.hpp")
    field = text("src/sensor/mag_field_reliability.cpp")
    bias = text("src/runtime/runtime_gyro_bias_controller.cpp")
    hot_policy = text("tools/test_0026b_hotpath_optimization_policy.py")
    headroom_policy = text("tools/test_pre_0024_hotpath_headroom_policy.py")
    check_all = text("tools/check_all.py")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/0027b_cross_host_quality_gate_hardening_report.md")

    socket_include = "#if defined(ARDUINO_ARCH_ESP32)\n#include <sys/socket.h>\n#endif"
    require(remote, socket_include, "target-only socket header")
    require(remote, "int remoteConsoleSendNonblocking(", "socket portability boundary")
    require(remote, "return ::send(socketFd, data, len, MSG_DONTWAIT);", "unchanged ESP/lwIP nonblocking send")
    require(remote, "errno = EWOULDBLOCK;", "compile-only host stub result")
    if remote.count("remoteConsoleSendNonblocking(") != 3:
        raise SystemExit("remote console must have one shim definition and two call sites")
    require(runner, 'pathlib.Path("src/network/wifi_remote_console.cpp")', "enabled remote-console host compile gate")

    phases = (
        "initializeReliabilityOutput",
        "updateHeadingRateEvidence",
        "updateReferenceEvidence",
        "updateStationaryHeadingEvidence",
        "advanceReliabilityState",
        "finalizeReliabilityOutput",
    )
    for phase in phases:
        require(field_h, phase, f"magnetic phase declaration {phase}")
        phase_pattern = re.compile(
            rf"TRACKER_MAG_FIELD_NOINLINE\s+(?:void|bool)\s+"
            rf"MagFieldReliabilityMonitor::{phase}\s*\("
        )
        if phase_pattern.search(field) is None:
            raise SystemExit(f"missing no-inline magnetic phase definition: {phase}")
        require(hot_policy, f'require_limit(usage, "{phase}"', f"magnetic phase stack ceiling {phase}")
    require(field, "const uint64_t phaseState =", "single magnetic phase handoff")
    require(field, "advanceReliabilityState(in, cfg, out, phaseState);", "state phase wiring")
    require(field, "finalizeReliabilityOutput(in, out, phaseState);", "output phase wiring")

    for phase in (
        "evaluateCompletedRuntimeBiasWindow",
        "rejectCompletedRuntimeBiasWindow",
        "primeCompletedRuntimeBiasWindow",
        "applyCompletedRuntimeBiasWindow",
    ):
        phase_pattern = re.compile(
            rf"TRACKER_RUNTIME_BIAS_NOINLINE\s+(?:uint8_t|void)\s+{phase}\s*\("
        )
        if phase_pattern.search(bias) is None:
            raise SystemExit(f"missing no-inline bias phase definition: {phase}")
        require(headroom_policy, f'("{phase}",', f"bias phase stack ceiling {phase}")
    require(bias, "const uint8_t evaluationFlags", "compact bias phase handoff")
    forbid(bias, "RuntimeBiasWindowEvaluation evaluation", "duplicate bias summary owner")

    runtime_code = re.sub(r"//.*?$|/\*.*?\*/", "", remote + field + bias,
                          flags=re.MULTILINE | re.DOTALL)
    forbid(runtime_code, "new ", "heap allocation in hardening paths")
    forbid(runtime_code, "std::vector", "dynamic container in hardening paths")

    aggregate_entry = '("tools/test_0027b_cross_host_quality_gate_policy.py", "0027b cross-host quality-gate hardening")'
    try:
        ast.parse(check_all, filename="tools/check_all.py")
    except SyntaxError as exc:
        raise SystemExit(f"tools/check_all.py is not valid Python: {exc}") from exc
    from check_all import TOOL_CHECKS
    if ast.literal_eval(aggregate_entry) not in TOOL_CHECKS:
        raise SystemExit("required policy missing from aggregate TOOL_CHECKS registry")
    require(testing, "0027b cross-host quality-gate hardening", "testing documentation")
    require(project, "0027b_cross_host_quality_gate_hardening", "project status entry")
    require(report, "0027a", "additive patch ordering")

    print("# 0027b_cross_host_quality_gate_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
