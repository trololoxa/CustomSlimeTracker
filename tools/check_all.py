#!/usr/bin/env python3
"""Run project quality gates and report all failures together.

Independent checks are never aborted by an earlier failure. Every runnable
native, policy, replay and PlatformIO gate is attempted, then a consolidated
summary is printed at the end.
"""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

from check_all_policy import parse_size_metrics
from quality_gate_runtime import quality_gate_environment, run_bounded_process
from release_manifest import (
    build_manifest,
    require_release_source,
    source_manifest,
    write_manifest,
)

ROOT = Path(__file__).resolve().parents[1]
REQUIRED_PIO_ENVS = (
    "BOARD_LOLIN_C3_MINI_PRODUCTION",
    "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG",
    "BOARD_LOLIN_C3_MINI_SLIM",
)
DEBUG_ENV = "BOARD_LOLIN_C3_MINI_DEBUG"
DEBUG_LINKCHECK_ENV = "BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK"
DEFAULT_TOOL_TIMEOUT_S = 180.0
DEFAULT_NATIVE_SUITE_TIMEOUT_S = 1800.0
DEFAULT_NATIVE_COMMAND_TIMEOUT_S = 180.0
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


def run_command(
    cmd: Sequence[str],
    *,
    cwd: Path = ROOT,
    timeout_s: float = DEFAULT_TOOL_TIMEOUT_S,
) -> int:
    printable = shlex.join(cmd)
    print(f"\n$ {printable}", flush=True)
    try:
        proc = run_bounded_process(
            cmd,
            cwd=str(cwd),
            timeout_s=timeout_s,
            env=quality_gate_environment(ROOT, scope="check-all"),
        )
        return proc.returncode
    except subprocess.TimeoutExpired:
        print(f"# command timed out after {timeout_s:g}s", file=sys.stderr, flush=True)
        return 124
    except OSError as exc:
        print(f"# command launch failed: {exc}", file=sys.stderr, flush=True)
        return 127


def run_checked(
    summary: CheckSummary,
    cmd: Sequence[str],
    description: str,
    *,
    cwd: Path = ROOT,
    timeout_s: float = DEFAULT_TOOL_TIMEOUT_S,
) -> bool:
    returncode = run_command(cmd, cwd=cwd, timeout_s=timeout_s)
    if returncode == 0:
        print(f"# PASS {description}", flush=True)
        return True
    summary.fail(f"{description} (exit={returncode})")
    print(f"# FAIL {description} (exit={returncode})", flush=True)
    return False


def run_native_tests(
    summary: CheckSummary,
    clean: bool,
    *,
    suite_timeout_s: float,
    command_timeout_s: float,
    sanitizer: str = "none",
) -> None:
    cmd = [
        sys.executable,
        "tools/run_standalone_tests.py",
        "--timeout-s",
        str(command_timeout_s),
        "--sanitizer",
        sanitizer,
    ]
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


def run_project_contract_checks(summary: CheckSummary, *, timeout_s: float) -> None:
    checks = (
        ("tools/validate_source_filters.py", "source-filter validation"),
        ("tools/validate_profile_matrix.py", "profile-matrix validation"),
        ("tools/validate_documentation.py", "documentation validation"),
    )
    for script, description in checks:
        run_checked(summary, [sys.executable, script], description, timeout_s=timeout_s)


def run_tool_smokes(summary: CheckSummary, *, timeout_s: float) -> None:
    out_dir = ROOT / "build" / "tool_smoke"
    out_dir.mkdir(parents=True, exist_ok=True)

    checks = (
        ("tools/test_build_identity.py", "build identity policy"),
        ("tools/test_check_all_policy.py", "check_all policy"),
        ("tools/test_check_all_aggregation_policy.py", "check_all aggregation policy"),
        ("tools/test_run_standalone_tests_policy.py", "standalone runner policy"),
        ("tools/test_quality_gate_runtime.py", "quality-gate runtime policy"),
        ("tools/test_release_manifest.py", "release-manifest policy"),
        ("tools/test_logver3_contract.py", "strict LOGVER3 contract"),
        ("tools/test_capture_telnet_log.py", "cable-free capture promotion policy"),
        ("tools/test_0025a_diagnostic_capture_policy.py", "0025a diagnostic capture lifecycle/policy"),
        ("tools/test_0025b_remote_cli_transport_parity_policy.py", "0025b remote CLI transport parity"),
        ("tools/test_0026a_motion_policy_and_magnetic_reliability_hardening.py", "0026a motion-policy/magnetic hardening"),
        ("tools/test_0026b_hotpath_optimization_policy.py", "0026b hot-path optimization"),
        ("tools/test_0026c_command_hook_declaration_order.py", "0026c command-hook declaration order"),
        ("tools/test_0026d_host_quality_gate_portability_policy.py", "0026d host quality-gate portability"),
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
        ("tools/test_pre_0024_hotpath_headroom_policy.py", "pre-0024 hotpath headroom policy"),
        ("tools/test_pre_0024a_tracking_deadline_policy.py", "pre-0024a tracking deadline policy"),
        ("tools/test_pre_0024ab_hotpath_transform_cache_policy.py", "pre-0024ab transform cache policy"),
        ("tools/test_pre_0024ac_imu_hotpath_slack_policy.py", "pre-0024ac IMU hotpath/slack policy"),
        ("tools/test_pre_0024ad_network_pressure_policy.py", "pre-0024ad network pressure pacing/recovery policy"),
        ("tools/test_calibration_integration_policy.py", "calibration integration policy"),
        ("tools/test_mag_heading_reliability_policy.py", "mag heading reliability policy"),
    )
    for script, description in checks:
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
        "native_command_timeout_s",
        "pio_timeout_s",
    ):
        if getattr(args, name) <= 0:
            errors.append(f"--{name.replace('_', '-')} must be positive")
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


def main(argv: Sequence[str] | None = None) -> int:
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
        default=DEFAULT_NATIVE_COMMAND_TIMEOUT_S,
        help="timeout forwarded to each native compile/link/test subprocess",
    )
    parser.add_argument(
        "--pio-timeout-s",
        type=float,
        default=DEFAULT_PIO_TIMEOUT_S,
        help="timeout for each PlatformIO environment build",
    )
    args = parser.parse_args(argv)

    errors = mode_errors(args)
    if errors:
        parser.error("; ".join(errors))
    apply_mode(args)

    if args.release:
        preflight_error = release_preflight()
        if preflight_error:
            print(f"# check_all: FAIL release preflight: {preflight_error}")
            return 1

    summary = CheckSummary()

    if not args.skip_native:
        run_native_tests(
            summary,
            args.clean,
            suite_timeout_s=args.native_suite_timeout_s,
            command_timeout_s=args.native_command_timeout_s,
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
                sanitizer="address-undefined",
            )
            run_native_tests(
                summary,
                True,
                suite_timeout_s=args.native_suite_timeout_s,
                command_timeout_s=args.native_command_timeout_s,
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


if __name__ == "__main__":
    raise SystemExit(main())
