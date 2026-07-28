#!/usr/bin/env python3
"""Run project quality gates and report all failures together.

Independent checks are never aborted by an earlier failure. Every runnable
native, policy, replay and PlatformIO gate is attempted, then a consolidated
summary is printed at the end.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

from check_all_policy import parse_size_metrics

ROOT = Path(__file__).resolve().parents[1]
REQUIRED_PIO_ENVS = (
    "BOARD_LOLIN_C3_MINI_PRODUCTION",
    "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG",
    "BOARD_LOLIN_C3_MINI_SLIM",
)
DEBUG_ENV = "BOARD_LOLIN_C3_MINI_DEBUG"
DEBUG_LINKCHECK_ENV = "BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK"


@dataclass
class PioBuildResult:
    environment: str
    returncode: int
    output: str

    @property
    def ok(self) -> bool:
        return self.returncode == 0


@dataclass
class CheckSummary:
    failures: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def fail(self, description: str) -> None:
        self.failures.append(description)

    def warn(self, description: str) -> None:
        self.warnings.append(description)


def run_command(cmd: Sequence[str], *, cwd: Path = ROOT) -> int:
    printable = " ".join(cmd)
    print(f"\n$ {printable}", flush=True)
    try:
        proc = subprocess.run(cmd, cwd=str(cwd), check=False)
        return proc.returncode
    except OSError as exc:
        print(f"# command launch failed: {exc}", file=sys.stderr, flush=True)
        return 127


def run_checked(
    summary: CheckSummary,
    cmd: Sequence[str],
    description: str,
    *,
    cwd: Path = ROOT,
) -> bool:
    returncode = run_command(cmd, cwd=cwd)
    if returncode == 0:
        print(f"# PASS {description}", flush=True)
        return True
    summary.fail(f"{description} (exit={returncode})")
    print(f"# FAIL {description} (exit={returncode})", flush=True)
    return False


def run_native_tests(summary: CheckSummary, clean: bool) -> None:
    cmd = [sys.executable, "tools/run_standalone_tests.py"]
    if clean:
        cmd.append("--clean")
    run_checked(summary, cmd, "standalone native test suite")


def run_replay_gate(
    summary: CheckSummary,
    fixture: Path,
    output: Path,
    *,
    description: str,
    min_duration_s: str,
    max_yaw_drift_deg_min: str | None = None,
) -> bool:
    output.unlink(missing_ok=True)
    cmd = [
        sys.executable,
        "tools/replay/replay_machine_log.py",
        str(fixture),
        "--output",
        str(output),
        "--min-duration-s",
        min_duration_s,
        "--max-fifo-fallback-rows",
        "0",
        "--max-fifo-fault-rows",
        "0",
        "--max-recovering-rows",
        "0",
    ]
    if max_yaw_drift_deg_min is not None:
        cmd.extend(["--max-yaw-drift-deg-min", max_yaw_drift_deg_min])
    return run_checked(summary, cmd, description)


def run_project_contract_checks(summary: CheckSummary) -> None:
    checks = (
        ("tools/validate_source_filters.py", "source-filter validation"),
        ("tools/validate_profile_matrix.py", "profile-matrix validation"),
        ("tools/validate_documentation.py", "documentation validation"),
    )
    for script, description in checks:
        run_checked(summary, [sys.executable, script], description)


def run_tool_smokes(summary: CheckSummary) -> None:
    out_dir = ROOT / "build" / "tool_smoke"
    out_dir.mkdir(parents=True, exist_ok=True)

    checks = (
        ("tools/test_build_identity.py", "build identity policy"),
        ("tools/test_check_all_policy.py", "check_all policy"),
        ("tools/test_check_all_aggregation_policy.py", "check_all aggregation policy"),
        ("tools/test_run_standalone_tests_policy.py", "standalone runner policy"),
        ("tools/test_slimevr_session_contract_policy.py", "SlimeVR session contract"),
        ("tools/test_calibration_storage_contract_policy.py", "calibration storage contract"),
        ("tools/test_calibration_storage_stack_policy.py", "calibration storage stack budget"),
        ("tools/test_calibration_autonomy_policy.py", "calibration autonomy policy"),
        ("tools/test_calibration_0023a_policy.py", "0023a calibration policy"),
        ("tools/test_calibration_0023b_policy.py", "0023b hardening policy"),
        ("tools/test_calibration_0023c_policy.py", "0023c recovery policy"),
        ("tools/test_calibration_0023d_policy.py", "0023d sleep recovery policy"),
        ("tools/test_calibration_0023e_policy.py", "0023e setup/hotpath policy"),
        ("tools/test_calibration_0023f_policy.py", "0023f full setup calibration policy"),
        ("tools/test_calibration_0023g_policy.py", "0023g magnetic coverage reservoir policy"),
        ("tools/test_calibration_0023ga_policy.py", "0023ga axis alignment stack hardening policy"),
        ("tools/test_calibration_0023gb_policy.py", "0023gb magnetometer fit metric policy"),
        ("tools/test_calibration_0023gc_policy.py", "0023gc mag fit stack/profile build policy"),
        ("tools/test_calibration_0023gd_policy.py", "0023gd magnetometer math/alignment policy"),
        ("tools/test_calibration_0023ge_policy.py", "0023ge magnetometer audit hardening policy"),
        ("tools/test_calibration_0023gf_policy.py", "0023gf mag callback cross-ABI policy"),
        ("tools/test_calibration_0023gg_policy.py", "0023gg magnetic timestamp/setup acceptance policy"),
        ("tools/test_slimevr_wifi_provisioning_0023gh_policy.py", "0023gh SlimeVR Wi-Fi provisioning compatibility"),
        ("tools/test_slimevr_connect_trackers_0023gi_policy.py", "0023gi SlimeVR Connect Trackers handshake/build date"),
        ("tools/test_slimevr_connect_trackers_0023gj_policy.py", "0023gj already-connected Connect Trackers session restart"),
        ("tools/test_calibration_0023gk_policy.py", "0023gk magnetometer robust-fit acceptance policy"),
        ("tools/test_slimevr_udp_tx_recovery_0023gl_policy.py", "0023gl SlimeVR UDP TX recovery policy"),
        ("tools/test_calibration_integration_policy.py", "calibration integration policy"),
        ("tools/test_mag_heading_reliability_policy.py", "mag heading reliability policy"),
    )
    for script, description in checks:
        run_checked(summary, [sys.executable, script], description)

    fixture = ROOT / "tests" / "fixtures" / "e0_static_smoke.log"
    if fixture.exists():
        before = out_dir / "e0_static_smoke_before.json"
        after = out_dir / "e0_static_smoke_after.json"
        magr = out_dir / "e0_static_smoke_magr.json"
        before_ok = run_replay_gate(
            summary,
            fixture,
            before,
            description="60-second replay gate",
            min_duration_s="60",
        )
        after.unlink(missing_ok=True)
        after_ok = run_checked(
            summary,
            [
                sys.executable,
                "tools/replay/replay_machine_log.py",
                str(fixture),
                "--output",
                str(after),
            ],
            "replay metrics generation",
        )
        magr.unlink(missing_ok=True)
        run_checked(
            summary,
            [
                sys.executable,
                "tools/replay/replay_machine_log.py",
                str(fixture),
                "--output",
                str(magr),
                "--require-magr",
                "--min-magr-rows",
                "1",
            ],
            "MAGR replay gate",
        )
        if before_ok and after_ok and before.exists() and after.exists():
            run_checked(
                summary,
                [
                    sys.executable,
                    "tools/replay/compare_replay_metrics.py",
                    str(before),
                    str(after),
                ],
                "replay metric comparison",
            )
        else:
            summary.fail("replay metric comparison blocked by failed prerequisite")
            print("# FAIL replay metric comparison blocked by failed prerequisite", flush=True)

    baseline = ROOT / "tests" / "fixtures" / "replay" / "baseline_replay_001.log"
    if baseline.exists():
        run_replay_gate(
            summary,
            baseline,
            out_dir / "baseline_replay_001.json",
            description="600-second baseline replay gate",
            min_duration_s="600",
            max_yaw_drift_deg_min="2.0",
        )


def pio_executable(explicit: str | None = None) -> str | None:
    if explicit:
        return explicit
    env_pio = os.environ.get("PIO")
    if env_pio:
        return env_pio
    return shutil.which("pio") or shutil.which("platformio")


def run_pio_build(pio: str, environment: str, out_dir: Path) -> PioBuildResult:
    cmd = [pio, "run", "-e", environment]
    print(f"\n$ {' '.join(cmd)}", flush=True)
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        output = proc.stdout or ""
        returncode = proc.returncode
    except OSError as exc:
        output = f"command launch failed: {exc}\n"
        returncode = 127
    print(output, end="" if output.endswith("\n") else "\n")
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f"{environment}.log").write_text(output, encoding="utf-8", errors="replace")
    return PioBuildResult(environment, returncode, output)


def print_size_summary(result: PioBuildResult) -> None:
    metrics = parse_size_metrics(result.output)
    if not metrics:
        return
    rendered = " ".join(
        f"{metric.kind}={metric.percent:.1f}%({metric.used}/{metric.total})"
        for metric in metrics
    )
    print(f"# SIZE {result.environment}: {rendered}")


def run_default_pio_policy(pio: str) -> tuple[list[str], list[str]]:
    out_dir = ROOT / "build" / "check_all" / "platformio"
    failures: list[str] = []
    warnings: list[str] = []

    environments = (*REQUIRED_PIO_ENVS, DEBUG_LINKCHECK_ENV, DEBUG_ENV)
    for environment in environments:
        result = run_pio_build(pio, environment, out_dir)
        print_size_summary(result)
        if result.ok:
            suffix = " (compile/type/link-symbol validation)" if environment == DEBUG_LINKCHECK_ENV else ""
            print(f"# PASS {environment}{suffix}")
        else:
            failures.append(f"{environment}: mandatory PlatformIO build failed")
            print(f"# FAIL {environment}")
    return failures, warnings


def run_explicit_pio_envs(pio: str, environments: Sequence[str]) -> tuple[list[str], list[str]]:
    out_dir = ROOT / "build" / "check_all" / "platformio"
    failures: list[str] = []
    for environment in environments:
        result = run_pio_build(pio, environment, out_dir)
        print_size_summary(result)
        if result.ok:
            print(f"# PASS {environment}")
        else:
            failures.append(f"{environment}: explicitly requested build failed")
            print(f"# FAIL {environment}")
    return failures, []


def run_pio_builds(
    explicit_envs: Sequence[str] | None,
    require_pio: bool,
    pio_bin: str | None = None,
) -> tuple[list[str], list[str], bool]:
    pio = pio_executable(pio_bin)
    if not pio:
        msg = "PlatformIO executable not found; skipping ESP32 builds."
        failures: list[str] = []
        if require_pio:
            failures.append(
                msg + " Install PlatformIO, set PIO=path-to-pio, or pass --pio-bin path-to-pio."
            )
        print("\n# " + msg)
        print("# Re-run with --require-pio on a machine where ESP32 compilation is expected.")
        return failures, [], True

    if explicit_envs:
        failures, warnings = run_explicit_pio_envs(pio, explicit_envs)
    else:
        failures, warnings = run_default_pio_policy(pio)
    return failures, warnings, False


def main() -> int:
    parser = argparse.ArgumentParser(description="Run tracker quality checks")
    parser.add_argument("--clean", action="store_true", help="clean native test build artifacts first")
    parser.add_argument("--skip-native", action="store_true", help="skip standalone native tests")
    parser.add_argument("--skip-pio", action="store_true", help="skip PlatformIO builds")
    parser.add_argument("--skip-tool-smoke", action="store_true", help="skip Python tool smoke tests")
    parser.add_argument("--require-pio", action="store_true", help="fail if PlatformIO is not installed")
    parser.add_argument("--pio-bin", help="explicit PlatformIO executable path; also available through PIO=...")
    parser.add_argument(
        "--pio-env",
        action="append",
        dest="pio_envs",
        help="strictly build one PlatformIO environment; can be passed multiple times",
    )
    args = parser.parse_args()

    summary = CheckSummary()

    if not args.skip_native:
        run_native_tests(summary, args.clean)

    run_project_contract_checks(summary)

    if not args.skip_tool_smoke:
        run_tool_smokes(summary)

    pio_skipped = False
    if not args.skip_pio:
        failures, warnings, pio_skipped = run_pio_builds(
            args.pio_envs,
            args.require_pio,
            args.pio_bin,
        )
        summary.failures.extend(failures)
        summary.warnings.extend(warnings)

    if summary.failures:
        print(f"\n# check_all: FAIL ({len(summary.failures)} failure(s))")
        for failure in summary.failures:
            print(f"# FAIL {failure}")
        for warning in summary.warnings:
            print(f"# WARN {warning}")
        return 1

    if summary.warnings:
        print("\n# check_all: PASS WITH WARNINGS")
        for warning in summary.warnings:
            print(f"# WARN {warning}")
        return 0

    if pio_skipped:
        print("\n# check_all: PASS (PlatformIO skipped)")
    else:
        print("\n# check_all: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
