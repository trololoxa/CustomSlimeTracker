#!/usr/bin/env python3
"""Guard host_gate_portability host quality-gate portability and successor-policy sync."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import forbid, require


ROOT = Path(__file__).resolve().parents[2]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def main() -> int:
    runtime = text("tools/quality_gate_runtime.py")
    runtime_test = text("tests/tooling/test_quality_gate_runtime.py")
    check_all = text("tools/check_all.py")
    mag_policy = text("tests/contracts/test_mag_heading_reliability.py")
    ge = text("tests/contracts/test_magnetic_realtime_admission.py")
    hot = text("tests/contracts/test_magnetic_hotpath_budget.py")
    udp = text("tests/contracts/test_udp_tx_recovery.py")
    pressure = text("tests/contracts/test_network_pressure_pacing.py")

    require(runtime, "def strongest_supported_sanitizer_flags(", "shared sanitizer link probe")
    require(runtime, '"address-undefined"', "ASan+UBSan preference")
    require(runtime, '"undefined"', "UBSan fallback")
    require(runtime, 'return "none", ()', "explicit unsupported-sanitizer result")
    require(runtime_test, "test_sanitizer_probe_falls_back_to_ubsan_when_address_runtime_is_missing", "UBSan fallback regression")
    require(runtime_test, "test_sanitizer_probe_reports_environment_limitation_when_no_runtime_links", "unsupported sanitizer regression")

    # UDP recovery owns the shared runtime sanitizer variants; pressure checks
    # retain source/stack constraints without compiling those variants twice.
    sanitizer_policies = (
        "tests/contracts/test_magnetic_robust_fit.py",
        "tests/contracts/test_udp_tx_recovery.py",
        "tests/contracts/test_hotpath_headroom.py",
        "tests/contracts/test_tracking_deadlines.py",
        "tests/contracts/test_transform_cache.py",
        "tests/contracts/test_imu_hotpath_budget.py",
    )
    for path in sanitizer_policies:
        policy = text(path)
        require(policy, "strongest_supported_sanitizer_flags", f"sanitizer capability use in {path}")
        forbid(policy, '"-fsanitize=address,undefined"', f"unconditional ASan/UBSan requirement in {path}")

    require(mag_policy, "wrapPi(heading.yawInnovationRad)", "post-host_gate_portability AHRS-yaw-invariant policy needle")
    forbid(mag_policy, "in.heading.yawInnovationRad", "pre-view stale mag policy needle")

    require(ge, 'require_limit(usage, "MagCalibrationCollector::compute", 1792)', "final host_gate_portability fit stack ceiling")
    forbid(ge, 'require_limit(usage, "MagCalibrationCollector::compute", 1536)', "stale predecessor fit stack ceiling")

    for policy, label in ((udp, "host_gate_portability"), (pressure, "host_gate_portability")):
        require(policy, "runtime_code = re.sub", f"comment-stripped no-heap scan in {label}")
        require(policy, 'forbid(runtime_code, "new "', f"code-only no-heap gate in {label}")

    for needle in (
        'require_limit(usage, "updateFieldReliabilitySnapshot", 256)',
        'require_limit(usage, "updateYawCorrectionSnapshot", 352)',
        'require_limit(usage, "processRawSample", 192)',
        'require_limit(usage, "MagFieldReliabilityMonitor::update", 384)',
        'require_limit(usage, "MagYawCorrectionController::update(const tracker::MagYawCorrectionInputView", 96)',
        'require_limit(usage, "SlimeVROutputRuntime::sendRotation", 320)',
    ):
        require(hot, needle, "host_gate_portability cross-ABI stack margin")


    print("# host_gate_portability: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
