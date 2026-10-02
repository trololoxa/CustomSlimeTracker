#!/usr/bin/env python3
"""Run project quality gates and report all failures together.

Independent checks are never aborted by an earlier failure. Every runnable
native, policy, replay and PlatformIO gate is attempted, then a consolidated
summary is printed at the end.
"""

from __future__ import annotations

import argparse
import math
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

from suite_inventory import check_registry
from gate_reporting import GateReport, failed_checks, canonical_check_ids, completed_child_report
from check_all_policy import parse_size_metrics
from quality_gate_runtime import quality_gate_environment, run_bounded_process
from release_manifest import (
    build_manifest,
    require_release_source,
    source_manifest,
    write_manifest,
)

ROOT = Path(__file__).resolve().parents[1]
_REPORT: GateReport | None = None
REQUIRED_PIO_ENVS = (
    "BOARD_LOLIN_C3_MINI_PRODUCTION",
    "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG",
    "BOARD_LOLIN_C3_MINI_SLIM",
)
DEBUG_ENV = "BOARD_LOLIN_C3_MINI_DEBUG"
DEBUG_LINKCHECK_ENV = "BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK"
DEFAULT_TOOL_TIMEOUT_S = 180.0
DEFAULT_NATIVE_SUITE_TIMEOUT_S = 1800.0
DEFAULT_PIO_TIMEOUT_S = 1800.0
STALE_PIO_ARTIFACT_RETURNCODE = 125
REQUIRED_RELEASE_ARTIFACTS = ("firmware.bin", "firmware.elf")
STRICT_LOGVER3_GATE = ROOT / "tools" / "replay" / "strict_logver3_gate.py"
STRICT_LOGVER3_FIXTURE = ROOT / "tests" / "fixtures" / "replay" / "logver3_static_golden.log"
STRICT_LOGVER3_GOLDEN = ROOT / "tests" / "fixtures" / "replay" / "logver3_static_golden.json"


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


@dataclass(frozen=True)
class CheckCommandResult:
    returncode: int
    stderr: str = ""


def run_command(
    cmd: Sequence[str],
    *,
    cwd: Path = ROOT,
    timeout_s: float = DEFAULT_TOOL_TIMEOUT_S,
) -> CheckCommandResult:
    if _REPORT is not None:
        result = _REPORT.run(cmd, cwd=cwd, timeout_s=timeout_s,
                             env={**quality_gate_environment(ROOT, scope="check-all"),
                                  "PYTHONIOENCODING": "utf-8"})
        if (result.returncode == 0 and len(cmd) > 1
                and cmd[1] == "tools/run_standalone_tests.py"):
            log = _REPORT.directory / _REPORT.data["commands"][-1]["log"]
            try:
                child = completed_child_report(log, ROOT, runner="native", scope="full-native")
                _REPORT.data["commands"][-1]["child_report"] = str(child)
            except (OSError, ValueError) as exc:
                # Preserve the actual child exit separately; the gate failure is
                # missing evidence, not a fabricated compiler/sanitizer finding.
                detail = f"native report validation failed: {exc}"
                _REPORT.data["commands"][-1].update(
                    process_returncode=0, returncode=1, evidence_error=detail)
                with log.open("a", encoding="utf-8") as stream:
                    stream.write("\n" + detail + "\n")
                _REPORT.save()
                return CheckCommandResult(1, detail)
            _REPORT.save()
        return CheckCommandResult(result.returncode, result.stderr)
    printable = shlex.join(cmd)
    print(f"\n$ {printable}", flush=True)
    try:
        proc = run_bounded_process(
            cmd,
            cwd=str(cwd),
            timeout_s=timeout_s,
            env=quality_gate_environment(ROOT, scope="check-all"),
            stderr=subprocess.PIPE,
            text=True,
        )
        stderr = proc.stderr or ""
        if stderr:
            print(stderr, end="" if stderr.endswith("\n") else "\n", file=sys.stderr, flush=True)
        return CheckCommandResult(proc.returncode, stderr)
    except subprocess.TimeoutExpired as exc:
        captured = exc.stderr or ""
        if isinstance(captured, bytes):
            captured = captured.decode("utf-8", errors="replace")
        message = f"command timed out after {timeout_s:g}s"
        detail = f"{captured.rstrip()}\n{message}" if captured else message
        print(detail, file=sys.stderr, flush=True)
        return CheckCommandResult(124, detail)
    except OSError as exc:
        detail = f"command launch failed: {exc}"
        print(detail, file=sys.stderr, flush=True)
        return CheckCommandResult(127, detail)


def run_checked(
    summary: CheckSummary,
    cmd: Sequence[str],
    description: str,
    *,
    cwd: Path = ROOT,
    timeout_s: float = DEFAULT_TOOL_TIMEOUT_S,
) -> bool:
    result = run_command(cmd, cwd=cwd, timeout_s=timeout_s)
    if result.returncode == 0:
        print(f"# PASS {description}", flush=True)
        return True
    failure = f"{description} (exit={result.returncode})"
    if result.stderr.strip():
        failure += "\n" + result.stderr.rstrip()
    summary.fail(failure)
    print(f"# FAIL {description} (exit={result.returncode})", flush=True)
    return False


def print_aggregated_failure(failure: str) -> None:
    lines = failure.splitlines()
    print(f"# FAIL {lines[0]}")
    for line in lines[1:]:
        print(f"#   {line}")


def run_native_tests(
    summary: CheckSummary,
    clean: bool,
    *,
    suite_timeout_s: float,
    command_timeout_s: float | None,
    build_timeout_s: float | None = None,
    test_timeout_s: float | None = None,
    sanitizer: str = "none",
) -> None:
    cmd = [
        sys.executable,
        "tools/run_standalone_tests.py",
        "--sanitizer",
        sanitizer,
    ]
    for option, value in (("--timeout-s", command_timeout_s),
                          ("--build-timeout-s", build_timeout_s),
                          ("--test-timeout-s", test_timeout_s)):
        if value is not None:
            cmd.extend([option, str(value)])
    if _REPORT is not None and _REPORT.verbose:
        cmd.append("--verbose")
    if clean:
        cmd.append("--clean")
    description = "standalone native test suite"
    if sanitizer != "none":
        description += f" ({sanitizer})"
    run_checked(
        summary,
        cmd,
        description,
        timeout_s=suite_timeout_s,
    )


def run_replay_gate(
    summary: CheckSummary,
    fixture: Path,
    output: Path,
    *,
    description: str,
    min_duration_s: str,
    max_yaw_drift_deg_min: str | None = None,
    timeout_s: float = DEFAULT_TOOL_TIMEOUT_S,
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
    return run_checked(summary, cmd, description, timeout_s=timeout_s)


CONTRACT_CHECKS = (
    ("tools/validate_test_structure.py", "test structure validation"),
    ("tools/validate_source_filters.py", "source-filter validation"),
    ("tools/validate_profile_matrix.py", "profile-matrix validation"),
    ("tools/validate_documentation.py", "documentation validation"),
)

def run_project_contract_checks(summary: CheckSummary, *, timeout_s: float) -> None:
    for script, description in CONTRACT_CHECKS:
        run_checked(summary, [sys.executable, script], description, timeout_s=timeout_s)


TOOL_CHECKS = (
    ("tests/tooling/test_maintenance_structure.py", "maintenance structure"),
    ("tests/tooling/test_build_identity.py", "build identity"),
    ("tests/tooling/test_check_all.py", "check all"),
    ("tests/tooling/test_check_all_aggregation.py", "check all aggregation"),
    ("tests/tooling/test_run_standalone_tests.py", "run standalone tests"),
    ("tests/tooling/test_quality_gate_runtime.py", "quality gate runtime"),
    ("tests/tooling/test_sanitizer_infrastructure.py", "sanitizer infrastructure"),
    ("tests/tooling/test_environment_doctor.py", "environment doctor"),
    ("tests/tooling/test_selective_runners.py", "selective runners"),
    ("tests/tooling/test_gate_reporting.py", "gate reporting"),
    ("tests/tooling/test_workflow_deadlines.py", "workflow deadlines"),
    ("tests/tooling/test_algorithm_accuracy.py", "algorithm accuracy"),
    ("tests/tooling/test_cross_platform_verification.py", "cross platform verification"),
    ("tests/tooling/test_device_workflow.py", "device workflow"),
    ("tests/tooling/test_release_manifest.py", "release manifest"),
    ("tests/tooling/test_logver3_contract.py", "logver3 contract"),
    ("tests/tooling/test_capture_telnet_log.py", "capture telnet log"),
    ("tests/contracts/test_diagnostic_capture_lifecycle.py", "diagnostic capture lifecycle"),
    ("tests/contracts/test_remote_console_parity.py", "remote console parity"),
    ("tests/contracts/test_motion_and_magnetic_admission.py", "motion and magnetic admission"),
    ("tests/contracts/test_magnetic_hotpath_budget.py", "magnetic hotpath budget"),
    ("tests/contracts/test_command_hook_order.py", "command hook order"),
    ("tests/contracts/test_host_gate_portability.py", "host gate portability"),
    ("tests/contracts/test_sensor_liveness.py", "sensor liveness"),
    ("tests/contracts/test_host_stack_portability.py", "host stack portability"),
    ("tests/contracts/test_yaw_microsoft_abi_budget.py", "yaw microsoft abi budget"),
    ("tests/contracts/test_recovery_feedback.py", "recovery feedback"),
    ("tests/contracts/test_progress_clock_domains.py", "progress clock domains"),
    ("tests/contracts/test_progress_epochs.py", "progress epochs"),
    ("tests/contracts/test_semantic_config_transaction.py", "semantic config transaction"),
    ("tests/contracts/test_transaction_quaternion_admission.py", "transaction quaternion admission"),
    ("tests/contracts/test_config_compatibility_migration.py", "config compatibility migration"),
    ("tests/contracts/test_calibration_recovery_deadlines.py", "calibration recovery deadlines"),
    ("tests/contracts/test_recovery_transaction_integration.py", "recovery transaction integration"),
    ("tests/contracts/test_slimevr_session_contract.py", "slimevr session contract"),
    ("tests/contracts/test_calibration_storage_contract.py", "calibration storage contract"),
    ("tests/contracts/test_calibration_storage_stack.py", "calibration storage stack"),
    ("tests/contracts/test_calibration_autonomy.py", "calibration autonomy"),
    ("tests/contracts/test_calibration_ownership.py", "calibration ownership"),
    ("tests/contracts/test_calibration_cross_abi.py", "calibration cross abi"),
    ("tests/contracts/test_calibration_tracking_recovery.py", "calibration tracking recovery"),
    ("tests/contracts/test_sleep_epoch_recovery.py", "sleep epoch recovery"),
    ("tests/contracts/test_setup_feature_profiles.py", "setup feature profiles"),
    ("tests/contracts/test_guided_setup.py", "guided setup"),
    ("tests/contracts/test_magnetic_coverage_reservoir.py", "magnetic coverage reservoir"),
    ("tests/contracts/test_axis_alignment_stack.py", "axis alignment stack"),
    ("tests/contracts/test_magnetic_fit_metrics.py", "magnetic fit metrics"),
    ("tests/contracts/test_magnetic_fit_profiles.py", "magnetic fit profiles"),
    ("tests/contracts/test_magnetic_math_alignment.py", "magnetic math alignment"),
    ("tests/contracts/test_magnetic_realtime_admission.py", "magnetic realtime admission"),
    ("tests/contracts/test_magnetic_callback_abi.py", "magnetic callback abi"),
    ("tests/contracts/test_magnetic_timestamp_acceptance.py", "magnetic timestamp acceptance"),
    ("tests/contracts/test_wifi_provisioning.py", "wifi provisioning"),
    ("tests/contracts/test_slimevr_handshake.py", "slimevr handshake"),
    ("tests/contracts/test_slimevr_session_restart.py", "slimevr session restart"),
    ("tests/contracts/test_magnetic_robust_fit.py", "magnetic robust fit"),
    ("tests/contracts/test_udp_tx_recovery.py", "udp tx recovery"),
    ("tests/contracts/test_hotpath_headroom.py", "hotpath headroom"),
    ("tests/contracts/test_tracking_deadlines.py", "tracking deadlines"),
    ("tests/contracts/test_transform_cache.py", "transform cache"),
    ("tests/contracts/test_imu_hotpath_budget.py", "imu hotpath budget"),
    ("tests/contracts/test_network_pressure_pacing.py", "network pressure pacing"),
    ("tests/contracts/test_calibration_integration.py", "calibration integration"),
    ("tests/contracts/test_mag_heading_reliability.py", "mag heading reliability"),
)

def run_tool_smokes(summary: CheckSummary, *, timeout_s: float) -> None:
    out_dir = ROOT / "build" / "tool_smoke"
    out_dir.mkdir(parents=True, exist_ok=True)

    for script, description in TOOL_CHECKS:
        run_checked(summary, [sys.executable, script], description, timeout_s=timeout_s)

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
            timeout_s=timeout_s,
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
            timeout_s=timeout_s,
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
            timeout_s=timeout_s,
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
                timeout_s=timeout_s,
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
            timeout_s=timeout_s,
        )


def pio_executable(explicit: str | None = None) -> str | None:
    if explicit:
        return explicit
    env_pio = os.environ.get("PIO")
    if env_pio:
        return env_pio
    return shutil.which("pio") or shutil.which("platformio")


def run_pio_build(
    pio: str,
    environment: str,
    out_dir: Path,
    *,
    timeout_s: float = DEFAULT_PIO_TIMEOUT_S,
    clean_first: bool = False,
) -> PioBuildResult:
    def invoke(cmd: list[str]) -> tuple[int, str]:
        if _REPORT is not None:
            result = _REPORT.run(cmd, cwd=ROOT, timeout_s=timeout_s,
                                 env=quality_gate_environment(ROOT, scope="platformio"))
            log = _REPORT.directory / _REPORT.data["commands"][-1]["log"]
            return result.returncode, log.read_text(encoding="utf-8", errors="replace")
        print(f"\n$ {shlex.join(cmd)}", flush=True)
        try:
            proc = run_bounded_process(
                cmd,
                cwd=str(ROOT),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout_s=timeout_s,
                env=quality_gate_environment(ROOT, scope="platformio"),
            )
            output = proc.stdout or ""
            returncode = proc.returncode
        except subprocess.TimeoutExpired as exc:
            captured = exc.stdout or ""
            if isinstance(captured, bytes):
                captured = captured.decode("utf-8", errors="replace")
            output = captured + f"\ncommand timed out after {timeout_s:g}s\n"
            returncode = 124
        except OSError as exc:
            output = f"command launch failed: {exc}\n"
            returncode = 127
        print(output, end="" if output.endswith("\n") else "\n")
        return returncode, output

    output_parts: list[str] = []
    returncode = 0
    if clean_first:
        clean_cmd = [pio, "run", "-e", environment, "-t", "clean"]
        returncode, clean_output = invoke(clean_cmd)
        output_parts.append(f"$ {shlex.join(clean_cmd)}\n{clean_output}")
        if returncode == 0:
            stale = pio_artifacts(environment)
            if stale:
                names = ", ".join(path.name for path in stale)
                stale_output = (
                    "release clean completed but stale firmware artifact(s) remain: "
                    f"{names}\n"
                )
                print(stale_output, end="")
                output_parts.append(stale_output)
                returncode = STALE_PIO_ARTIFACT_RETURNCODE

    if returncode == 0:
        build_cmd = [pio, "run", "-e", environment]
        returncode, build_output = invoke(build_cmd)
        output_parts.append(f"$ {shlex.join(build_cmd)}\n{build_output}")

    output = "\n".join(output_parts)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f"{environment}.log").write_text(output, encoding="utf-8", errors="replace")
    return PioBuildResult(environment, returncode, output)


def pio_version(pio: str, timeout_s: float) -> str:
    try:
        proc = run_bounded_process(
            [pio, "--version"],
            cwd=str(ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout_s=min(timeout_s, 30.0),
            env=quality_gate_environment(ROOT, scope="platformio-version"),
        )
    except (OSError, subprocess.TimeoutExpired):
        return "unknown"
    value = (proc.stdout or "").strip()
    return value if proc.returncode == 0 and value else "unknown"


def pio_artifacts(environment: str) -> list[Path]:
    build_dir = ROOT / ".pio" / "build" / environment
    names = ("firmware.bin", "firmware.elf", "partitions.bin", "bootloader.bin")
    return [build_dir / name for name in names if (build_dir / name).is_file()]


def write_pio_manifest(
    pio: str,
    environment: str,
    out_dir: Path,
    *,
    require_clean: bool,
    timeout_s: float,
) -> Path:
    artifacts = pio_artifacts(environment)
    if require_clean:
        available_names = {path.name for path in artifacts}
        missing = [name for name in REQUIRED_RELEASE_ARTIFACTS if name not in available_names]
        if missing:
            raise ValueError(
                f"{environment}: missing release artifact(s): {', '.join(missing)}"
            )
    manifest = build_manifest(
        ROOT,
        environment,
        artifacts,
        toolchains={"platformio": pio_version(pio, timeout_s)},
        require_clean=require_clean,
    )
    output = out_dir / "manifests" / f"{environment}.json"
    write_manifest(output, manifest)
    return output


def print_size_summary(result: PioBuildResult) -> None:
    metrics = parse_size_metrics(result.output)
    if not metrics:
        return
    rendered = " ".join(
        f"{metric.kind}={metric.percent:.1f}%({metric.used}/{metric.total})"
        for metric in metrics
    )
    print(f"# SIZE {result.environment}: {rendered}")


def run_default_pio_policy(
    pio: str,
    *,
    release: bool = False,
    timeout_s: float = DEFAULT_PIO_TIMEOUT_S,
) -> tuple[list[str], list[str]]:
    out_dir = ROOT / "build" / "check_all" / "platformio"
    failures: list[str] = []
    warnings: list[str] = []

    environments = (*REQUIRED_PIO_ENVS, DEBUG_LINKCHECK_ENV, DEBUG_ENV)
    for environment in environments:
        result = run_pio_build(
            pio,
            environment,
            out_dir,
            timeout_s=timeout_s,
            clean_first=release,
        )
        print_size_summary(result)
        if result.ok:
            suffix = " (compile/type/link-symbol validation)" if environment == DEBUG_LINKCHECK_ENV else ""
            print(f"# PASS {environment}{suffix}")
            try:
                manifest = write_pio_manifest(
                    pio,
                    environment,
                    out_dir,
                    require_clean=release,
                    timeout_s=timeout_s,
                )
                print(f"# MANIFEST {manifest.relative_to(ROOT)}")
            except (OSError, ValueError) as exc:
                failures.append(f"{environment}: release manifest failed: {exc}")
                print(f"# FAIL {environment} manifest: {exc}")
        else:
            failures.append(f"{environment}: mandatory PlatformIO build failed")
            print(f"# FAIL {environment}")
    return failures, warnings


def run_explicit_pio_envs(
    pio: str,
    environments: Sequence[str],
    *,
    timeout_s: float = DEFAULT_PIO_TIMEOUT_S,
) -> tuple[list[str], list[str]]:
    out_dir = ROOT / "build" / "check_all" / "platformio"
    failures: list[str] = []
    for environment in environments:
        result = run_pio_build(pio, environment, out_dir, timeout_s=timeout_s)
        print_size_summary(result)
        if result.ok:
            print(f"# PASS {environment}")
            try:
                manifest = write_pio_manifest(
                    pio,
                    environment,
                    out_dir,
                    require_clean=False,
                    timeout_s=timeout_s,
                )
                print(f"# MANIFEST {manifest.relative_to(ROOT)}")
            except (OSError, ValueError) as exc:
                failures.append(f"{environment}: build manifest failed: {exc}")
                print(f"# FAIL {environment} manifest: {exc}")
        else:
            failures.append(f"{environment}: explicitly requested build failed")
            print(f"# FAIL {environment}")
    return failures, []


def run_pio_builds(
    explicit_envs: Sequence[str] | None,
    require_pio: bool,
    pio_bin: str | None = None,
    *,
    release: bool = False,
    timeout_s: float = DEFAULT_PIO_TIMEOUT_S,
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
        failures, warnings = run_explicit_pio_envs(pio, explicit_envs, timeout_s=timeout_s)
    else:
        failures, warnings = run_default_pio_policy(
            pio,
            release=release,
            timeout_s=timeout_s,
        )
    return failures, warnings, False


def mode_errors(args: argparse.Namespace) -> list[str]:
    errors: list[str] = []
    if getattr(args, "checks", None) or getattr(args, "failed_from", None):
        if (args.host_only or args.release or args.clean or args.require_pio or args.pio_envs
                or args.skip_native or args.skip_tool_smoke or args.skip_pio):
            errors.append("focused checks cannot be combined with full/host/release/skip/build options")
    if args.host_only:
        if args.require_pio:
            errors.append("--host-only cannot be combined with --require-pio")
        if args.pio_envs:
            errors.append("--host-only cannot be combined with --pio-env")
        if args.skip_native or args.skip_tool_smoke:
            errors.append("--host-only requires all host checks; --skip-native/--skip-tool-smoke are forbidden")
    if args.release:
        if args.skip_native or args.skip_pio or args.skip_tool_smoke:
            errors.append("--release forbids every --skip-* option")
        if args.pio_envs:
            errors.append("--release always builds the complete five-environment matrix")
    for name in (
        "tool_timeout_s",
        "native_suite_timeout_s",
        "pio_timeout_s",
    ):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            errors.append(f"--{name.replace('_', '-')} must be positive")
    for name in ("native_command_timeout_s", "native_build_timeout_s", "native_test_timeout_s"):
        value = getattr(args, name, None)
        if value is not None and (not math.isfinite(value) or value <= 0):
            errors.append(f"--{name.replace('_', '-')} must be positive and finite")
    return errors


def apply_mode(args: argparse.Namespace) -> None:
    if args.host_only:
        args.skip_pio = True
    if args.release:
        args.clean = True
        args.require_pio = True


def release_preflight(root: Path = ROOT) -> str | None:
    try:
        require_release_source(source_manifest(root))
    except ValueError as exc:
        return str(exc)
    required_logver3 = (
        root / STRICT_LOGVER3_GATE.relative_to(ROOT),
        root / STRICT_LOGVER3_FIXTURE.relative_to(ROOT),
        root / STRICT_LOGVER3_GOLDEN.relative_to(ROOT),
    )
    missing = [str(path.relative_to(root)) for path in required_logver3 if not path.is_file()]
    if missing:
        return (
            "strict LOGVER3 release gate is not installed; missing: "
            + ", ".join(missing)
        )
    return None


def run_release_logver3_gate(summary: CheckSummary, *, timeout_s: float) -> None:
    run_checked(
        summary,
        [
            sys.executable,
            str(STRICT_LOGVER3_GATE.relative_to(ROOT)),
            "--fixture",
            str(STRICT_LOGVER3_FIXTURE.relative_to(ROOT)),
            "--golden",
            str(STRICT_LOGVER3_GOLDEN.relative_to(ROOT)),
        ],
        "strict LOGVER3 static golden gate",
        timeout_s=timeout_s,
    )


def _main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run tracker quality checks")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument(
        "--host-only",
        action="store_true",
        help="run the complete host gate and explicitly omit target builds",
    )
    modes.add_argument(
        "--release",
        action="store_true",
        help="require clean Git identity, every host gate, five target builds and manifests",
    )
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
    parser.add_argument(
        "--tool-timeout-s",
        type=float,
        default=DEFAULT_TOOL_TIMEOUT_S,
        help="timeout for each validator, policy or replay subprocess",
    )
    parser.add_argument(
        "--native-suite-timeout-s",
        type=float,
        default=DEFAULT_NATIVE_SUITE_TIMEOUT_S,
        help="timeout for the complete standalone native runner",
    )
    parser.add_argument(
        "--native-command-timeout-s",
        type=float,
        default=None,
        help="legacy override for both native build and test deadlines",
    )
    parser.add_argument("--native-build-timeout-s", type=float, help="per native compile/link deadline")
    parser.add_argument("--native-test-timeout-s", type=float, help="per native test execution deadline")
    parser.add_argument(
        "--pio-timeout-s",
        type=float,
        default=DEFAULT_PIO_TIMEOUT_S,
        help="timeout for each PlatformIO environment build",
    )
    try:
        registry = check_registry((*CONTRACT_CHECKS, *TOOL_CHECKS))
    except ValueError as exc:
        parser.error(str(exc))
    parser.add_argument("--list-checks", action="store_true", help="list exact check IDs without executing")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--check", action="append", dest="checks", metavar="ID", help="run only selected checks; partial evidence")
    selection.add_argument("--failed-from", type=Path, help="rerun failed IDs from a completed focused JSON report")
    parser.add_argument("--verbose", action="store_true", help="print full command logs after each command")
    args = parser.parse_args(argv)
    if args.list_checks:
        for name, (_, description) in registry.items():
            print(f"{name}: {description}")
        return 0
    if args.failed_from:
        try:
            args.checks = failed_checks(args.failed_from, {key: value[0] for key, value in registry.items()})
        except (OSError, ValueError) as exc:
            parser.error(str(exc))

    args.checks = canonical_check_ids(args.checks)
    unknown = [name for name in (args.checks or []) if name not in registry]
    if unknown:
        parser.error("unknown check ID(s): " + ", ".join(unknown) + "; use --list-checks")
    errors = mode_errors(args)
    if errors:
        parser.error("; ".join(errors))
    apply_mode(args)

    if args.release:
        preflight_error = release_preflight()
        if preflight_error:
            print(f"# check_all: FAIL release preflight: {preflight_error}")
            return 1

    global _REPORT
    _REPORT = GateReport(ROOT, "check-all", verbose=args.verbose)
    selected = list(dict.fromkeys(args.checks or []))
    _REPORT.configure("focused" if selected else "release" if args.release else "host" if args.host_only else "default",
                      selected, argv=list(argv if argv is not None else sys.argv[1:]))
    summary = CheckSummary()
    if selected:
        for name in selected:
            script, description = registry[name]
            run_checked(summary, [sys.executable, script], description, timeout_s=args.tool_timeout_s)
        _REPORT.data["failures"] = summary.failures
        _REPORT.save()
        print(f"# check_all: {'FAIL' if summary.failures else 'PASS'} (focused partial checks: {len(selected)}; full gate NOT verified)")
        return int(bool(summary.failures))

    if not args.skip_native:
        run_native_tests(
            summary,
            args.clean,
            suite_timeout_s=args.native_suite_timeout_s,
            command_timeout_s=args.native_command_timeout_s,
            build_timeout_s=args.native_build_timeout_s,
            test_timeout_s=args.native_test_timeout_s,
        )
        if args.release:
            # GCC's address sanitizer links leak checking by default. The
            # address/undefined gate disables it at link-runtime level, then a
            # separate leak-only executable matrix owns leak failures.
            run_native_tests(
                summary,
                True,
                suite_timeout_s=args.native_suite_timeout_s,
                command_timeout_s=args.native_command_timeout_s,
                build_timeout_s=args.native_build_timeout_s,
                test_timeout_s=args.native_test_timeout_s,
                sanitizer="address-undefined",
            )
            run_native_tests(
                summary,
                True,
                suite_timeout_s=args.native_suite_timeout_s,
                command_timeout_s=args.native_command_timeout_s,
                build_timeout_s=args.native_build_timeout_s,
                test_timeout_s=args.native_test_timeout_s,
                sanitizer="leak",
            )

    run_project_contract_checks(summary, timeout_s=args.tool_timeout_s)

    if not args.skip_tool_smoke:
        run_tool_smokes(summary, timeout_s=args.tool_timeout_s)

    if args.release:
        run_release_logver3_gate(summary, timeout_s=args.tool_timeout_s)

    pio_skipped = False
    if not args.skip_pio:
        failures, warnings, pio_skipped = run_pio_builds(
            args.pio_envs,
            args.require_pio,
            args.pio_bin,
            release=args.release,
            timeout_s=args.pio_timeout_s,
        )
        summary.failures.extend(failures)
        summary.warnings.extend(warnings)

    if _REPORT is not None:
        _REPORT.data["failures"] = summary.failures
        _REPORT.data["warnings"] = summary.warnings
        _REPORT.save()
    if summary.failures:
        print(f"\n# check_all: FAIL ({len(summary.failures)} failure(s))")
        for failure in summary.failures:
            if _REPORT is None:
                print_aggregated_failure(failure)
            else:
                print(f"# FAIL {failure.splitlines()[0]}")
        for warning in summary.warnings:
            print(f"# WARN {warning}")
        return 1

    if summary.warnings:
        print("\n# check_all: PASS WITH WARNINGS")
        for warning in summary.warnings:
            print(f"# WARN {warning}")
        return 0

    if pio_skipped or args.skip_pio:
        if args.skip_native or args.skip_tool_smoke:
            print("\n# check_all: PASS (partial host checks; target build not verified)")
        else:
            print("\n# check_all: PASS (host-verified, target build not verified)")
    elif args.release:
        print("\n# check_all: PASS (release gate, manifests generated)")
    else:
        print("\n# check_all: PASS")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    global _REPORT
    previous = _REPORT
    _REPORT = None
    code, interrupted = 130, True
    try:
        code = _main(argv)
        interrupted = False
        return code
    except KeyboardInterrupt:
        code = 130
        return code
    except BaseException:
        code = 1
        raise
    finally:
        try:
            if _REPORT is not None:
                _REPORT.finish(code, interrupted=interrupted)
        finally:
            _REPORT = previous


if __name__ == "__main__":
    raise SystemExit(main())
