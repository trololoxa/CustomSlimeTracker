#!/usr/bin/env python3
"""Run local project quality checks.

The script is intentionally cross-platform: use it directly on Windows
(`python tools/check_all.py`) or through tools/check_all.sh from Git Bash/WSL.
PlatformIO builds are optional by default so the native test gate can run on
machines that do not have the ESP32 toolchain installed.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable, Sequence

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PIO_ENVS = ("BOARD_LOLIN_C3_MINI", "BOARD_LOLIN_C3_MINI_DIAG")


def run(cmd: Sequence[str], *, cwd: Path = ROOT) -> None:
    printable = " ".join(cmd)
    print(f"\n$ {printable}", flush=True)
    subprocess.run(cmd, cwd=str(cwd), check=True)


def run_native_tests(clean: bool) -> None:
    cmd = [sys.executable, "tools/run_standalone_tests.py"]
    if clean:
        cmd.append("--clean")
    run(cmd)


def run_tool_smokes() -> None:
    fixture = ROOT / "tests" / "fixtures" / "e0_static_smoke.log"
    if fixture.exists():
        run([
            sys.executable,
            "tools/replay/replay_machine_log.py",
            str(fixture),
            "--min-duration-s",
            "60",
            "--max-fifo-fallback-rows",
            "0",
            "--max-fifo-fault-rows",
            "0",
            "--max-recovering-rows",
            "0",
        ])


def pio_executable(explicit: str | None = None) -> str | None:
    if explicit:
        return explicit
    env_pio = os.environ.get("PIO")
    if env_pio:
        return env_pio
    return shutil.which("pio") or shutil.which("platformio")


def run_pio_builds(envs: Iterable[str], require_pio: bool, pio_bin: str | None = None) -> None:
    pio = pio_executable(pio_bin)
    if not pio:
        msg = "PlatformIO executable not found; skipping ESP32 builds."
        if require_pio:
            raise SystemExit(msg + " Install PlatformIO, set PIO=path-to-pio, or pass --pio-bin path-to-pio.")
        print("\n# " + msg)
        print("# Re-run with --require-pio on a machine where ESP32 compilation is expected.")
        return

    for env in envs:
        run([pio, "run", "-e", env])


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
        help="PlatformIO environment to build; can be passed multiple times",
    )
    args = parser.parse_args()

    if not args.skip_native:
        run_native_tests(args.clean)

    if not args.skip_tool_smoke:
        run_tool_smokes()

    if not args.skip_pio:
        run_pio_builds(args.pio_envs or DEFAULT_PIO_ENVS, args.require_pio, args.pio_bin)

    print("\n# check_all: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
