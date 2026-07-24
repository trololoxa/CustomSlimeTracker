#!/usr/bin/env python3
"""Run local project quality checks with profile-aware PlatformIO policy.

Production, Production Diagnostic, Slim, Debug and the explicit Debug
link-check are mandatory. Every ESP32-C3 profile uses the same 4 MiB no-OTA
partition table with one 3 MiB factory app, so any image-size overflow is a
real project-contract failure.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
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


def run(cmd: Sequence[str], *, cwd: Path = ROOT) -> None:
    printable = " ".join(cmd)
    print(f"\n$ {printable}", flush=True)
    subprocess.run(cmd, cwd=str(cwd), check=True)


def run_native_tests(clean: bool) -> None:
    cmd = [sys.executable, "tools/run_standalone_tests.py"]
    if clean:
        cmd.append("--clean")
    run(cmd)


def run_replay_gate(
    fixture: Path,
    output: Path,
    *,
    min_duration_s: str,
    max_yaw_drift_deg_min: str | None = None,
) -> None:
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
    run(cmd)


def run_project_contract_checks() -> None:
    run([sys.executable, "tools/validate_source_filters.py"])
    run([sys.executable, "tools/validate_profile_matrix.py"])
    run([sys.executable, "tools/validate_documentation.py"])


def run_tool_smokes() -> None:
    out_dir = ROOT / "build" / "tool_smoke"
    out_dir.mkdir(parents=True, exist_ok=True)

    run([sys.executable, "tools/test_build_identity.py"])
    run([sys.executable, "tools/test_check_all_policy.py"])
    run([sys.executable, "tools/test_run_standalone_tests_policy.py"])
    run([sys.executable, "tools/test_slimevr_session_contract_policy.py"])
    run([sys.executable, "tools/test_calibration_storage_contract_policy.py"])
    run([sys.executable, "tools/test_calibration_storage_stack_policy.py"])
    run([sys.executable, "tools/test_calibration_integration_policy.py"])

    fixture = ROOT / "tests" / "fixtures" / "e0_static_smoke.log"
    if fixture.exists():
        before = out_dir / "e0_static_smoke_before.json"
        after = out_dir / "e0_static_smoke_after.json"
        run_replay_gate(fixture, before, min_duration_s="60")
        run([
            sys.executable,
            "tools/replay/replay_machine_log.py",
            str(fixture),
            "--output",
            str(after),
        ])
        run([
            sys.executable,
            "tools/replay/replay_machine_log.py",
            str(fixture),
            "--output",
            str(out_dir / "e0_static_smoke_magr.json"),
            "--require-magr",
            "--min-magr-rows",
            "1",
        ])
        run([
            sys.executable,
            "tools/replay/compare_replay_metrics.py",
            str(before),
            str(after),
        ])

    baseline = ROOT / "tests" / "fixtures" / "replay" / "baseline_replay_001.log"
    if baseline.exists():
        run_replay_gate(
            baseline,
            out_dir / "baseline_replay_001.json",
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
    proc = subprocess.run(
        cmd,
        cwd=str(ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    print(proc.stdout, end="" if proc.stdout.endswith("\n") else "\n")
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f"{environment}.log").write_text(proc.stdout, encoding="utf-8", errors="replace")
    return PioBuildResult(environment, proc.returncode, proc.stdout)


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

    for environment in REQUIRED_PIO_ENVS:
        result = run_pio_build(pio, environment, out_dir)
        print_size_summary(result)
        if result.ok:
            print(f"# PASS {environment}")
        else:
            failures.append(f"{environment}: mandatory PlatformIO build failed")
            print(f"# FAIL {environment}")

    # Explicitly compile the complete Debug source set in its dedicated gate.
    # Every environment now shares the same 3 MiB no-OTA app partition, so a
    # size overflow is a real project-contract failure rather than an accepted
    # wearable-layout warning.
    linkcheck = run_pio_build(pio, DEBUG_LINKCHECK_ENV, out_dir)
    print_size_summary(linkcheck)
    if linkcheck.ok:
        print(f"# PASS {DEBUG_LINKCHECK_ENV} (compile/type/link-symbol validation)")
    else:
        failures.append(f"{DEBUG_LINKCHECK_ENV}: Debug compile/type/link-symbol validation failed")
        print(f"# FAIL {DEBUG_LINKCHECK_ENV}")

    debug = run_pio_build(pio, DEBUG_ENV, out_dir)
    print_size_summary(debug)
    if debug.ok:
        print(f"# PASS {DEBUG_ENV}")
    else:
        failures.append(f"{DEBUG_ENV}: mandatory Debug build failed")
        print(f"# FAIL {DEBUG_ENV}")

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
        if require_pio:
            raise SystemExit(msg + " Install PlatformIO, set PIO=path-to-pio, or pass --pio-bin path-to-pio.")
        print("\n# " + msg)
        print("# Re-run with --require-pio on a machine where ESP32 compilation is expected.")
        return [], [], True

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

    if not args.skip_native:
        run_native_tests(args.clean)

    run_project_contract_checks()

    if not args.skip_tool_smoke:
        run_tool_smokes()

    failures: list[str] = []
    warnings: list[str] = []
    pio_skipped = False
    if not args.skip_pio:
        failures, warnings, pio_skipped = run_pio_builds(args.pio_envs, args.require_pio, args.pio_bin)

    if failures:
        print("\n# check_all: FAIL")
        for failure in failures:
            print(f"# FAIL {failure}")
        return 1

    if warnings:
        print("\n# check_all: PASS WITH WARNINGS")
        for warning in warnings:
            print(f"# WARN {warning}")
        return 0

    if pio_skipped:
        print("\n# check_all: PASS (PlatformIO skipped)")
    else:
        print("\n# check_all: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
