#!/usr/bin/env python3
"""Build firmware profiles and save PlatformIO size reports.

The normal Debug profile is advisory when it fails only because the wearable
partition is too small. Product profiles remain strict.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path
from typing import Sequence

from check_all_policy import is_size_only_failure, parse_size_metrics

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ENVS = (
    "BOARD_LOLIN_C3_MINI_PRODUCTION",
    "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG",
    "BOARD_LOLIN_C3_MINI_SLIM",
    "BOARD_LOLIN_C3_MINI_DEBUG",
)
DEBUG_ENV = "BOARD_LOLIN_C3_MINI_DEBUG"


def pio_executable(explicit: str | None = None) -> str | None:
    if explicit:
        return explicit
    env_pio = os.environ.get("PIO")
    if env_pio:
        return env_pio
    return shutil.which("pio") or shutil.which("platformio")


def run_capture(cmd: Sequence[str], output_path: Path) -> tuple[int, str]:
    printable = " ".join(cmd)
    print(f"\n$ {printable}", flush=True)
    proc = subprocess.run(
        cmd,
        cwd=str(ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(proc.stdout, encoding="utf-8", errors="replace")
    print(proc.stdout, end="" if proc.stdout.endswith("\n") else "\n")
    return proc.returncode, proc.stdout


def main() -> int:
    parser = argparse.ArgumentParser(description="Build firmware profiles and save size reports")
    parser.add_argument("--pio-bin", help="explicit PlatformIO executable path; also available through PIO=...")
    parser.add_argument(
        "--env",
        action="append",
        dest="envs",
        help="PlatformIO environment to size; can be passed multiple times",
    )
    args = parser.parse_args()

    pio = pio_executable(args.pio_bin)
    if not pio:
        raise SystemExit("PlatformIO executable not found. Install PlatformIO, set PIO=path-to-pio, or pass --pio-bin.")

    out_dir = ROOT / "build" / "firmware_size"
    envs = tuple(args.envs or DEFAULT_ENVS)
    failures: list[str] = []
    warnings: list[str] = []
    for env in envs:
        returncode, output = run_capture([pio, "run", "-e", env, "-t", "size"], out_dir / f"{env}.txt")
        metrics = parse_size_metrics(output)
        if metrics:
            print("# SIZE " + env + ": " + " ".join(
                f"{metric.kind}={metric.percent:.1f}%({metric.used}/{metric.total})" for metric in metrics
            ))
        if returncode == 0:
            continue
        if env == DEBUG_ENV and is_size_only_failure(output):
            warnings.append(f"{env}: expected advisory size overflow")
        else:
            failures.append(f"{env}: size build failed")

    print(f"\n# firmware size reports saved to: {out_dir}")
    for warning in warnings:
        print(f"# WARN {warning}")
    for failure in failures:
        print(f"# FAIL {failure}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
