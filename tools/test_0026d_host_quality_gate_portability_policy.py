#!/usr/bin/env python3
"""Guard 0026d host quality-gate portability and successor-policy sync."""

from __future__ import annotations

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
    runtime = text("tools/quality_gate_runtime.py")
    runtime_test = text("tools/test_quality_gate_runtime.py")
    check_all = text("tools/check_all.py")
    mag_policy = text("tools/test_mag_heading_reliability_policy.py")
    ge = text("tools/test_calibration_0023ge_policy.py")
    hot = text("tools/test_0026b_hotpath_optimization_policy.py")
    udp = text("tools/test_slimevr_udp_tx_recovery_0023gl_policy.py")
    pressure = text("tools/test_pre_0024ad_network_pressure_policy.py")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/0026d_host_quality_gate_portability_and_policy_sync_report.md")

    require(runtime, "def strongest_supported_sanitizer_flags(", "shared sanitizer link probe")
    require(runtime, '"address-undefined"', "ASan+UBSan preference")
    require(runtime, '"undefined"', "UBSan fallback")
    require(runtime, 'return "none", ()', "explicit unsupported-sanitizer result")
    require(runtime_test, "test_sanitizer_probe_falls_back_to_ubsan_when_address_runtime_is_missing", "UBSan fallback regression")
    require(runtime_test, "test_sanitizer_probe_reports_environment_limitation_when_no_runtime_links", "unsupported sanitizer regression")

    sanitizer_policies = (
        "tools/test_calibration_0023gk_policy.py",
        "tools/test_slimevr_udp_tx_recovery_0023gl_policy.py",
        "tools/test_pre_0024_hotpath_headroom_policy.py",
        "tools/test_pre_0024a_tracking_deadline_policy.py",
        "tools/test_pre_0024ab_hotpath_transform_cache_policy.py",
        "tools/test_pre_0024ac_imu_hotpath_slack_policy.py",
        "tools/test_pre_0024ad_network_pressure_policy.py",
    )
    for path in sanitizer_policies:
        policy = text(path)
        require(policy, "strongest_supported_sanitizer_flags", f"sanitizer capability use in {path}")
        forbid(policy, '"-fsanitize=address,undefined"', f"unconditional ASan/UBSan requirement in {path}")

    require(mag_policy, "wrapPi(heading.yawInnovationRad)", "post-0026b AHRS-yaw-invariant policy needle")
    forbid(mag_policy, "in.heading.yawInnovationRad", "pre-view stale mag policy needle")

    require(ge, 'require_limit(usage, "MagCalibrationCollector::compute", 1792)', "final 0023gk fit stack ceiling")
    forbid(ge, 'require_limit(usage, "MagCalibrationCollector::compute", 1536)', "stale predecessor fit stack ceiling")

    for policy, label in ((udp, "0023gl"), (pressure, "pre-0024ad")):
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
        require(hot, needle, "0026b cross-ABI stack margin")

    require(check_all, '("tools/test_0026d_host_quality_gate_portability_policy.py", "0026d host quality-gate portability")', "aggregate gate entry")
    require(testing, "0026d host quality-gate portability", "testing documentation")
    require(project, "0026d_host_quality_gate_portability_and_policy_sync", "project status entry")
    require(report, "cannot find -lasan", "Windows sanitizer failure evidence")
    require(report, "in.heading.yawInnovationRad", "stale magnetic policy evidence")

    print("# 0026d_host_quality_gate_portability_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
