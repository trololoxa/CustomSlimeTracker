#!/usr/bin/env python3
"""Build firmware profiles and save PlatformIO size reports.

This is a lightweight size gate for the optimization work. It intentionally uses
PlatformIO's own `-t size` target instead of parsing board-specific linker maps.
The raw report for every environment is saved under build/firmware_size/ so it
can be attached to optimization notes and compared between patches.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ENVS = (
    "BOARD_LOLIN_C3_MINI_DEBUG",
    "BOARD_LOLIN_C3_MINI_PRODUCTION",
    "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG",
    "BOARD_LOLIN_C3_MINI_SLIM",
)


def pio_executable(explicit: str | None = None) -> str | None:
    if explicit:
        return explicit
    env_pio = os.environ.get("PIO")
    if env_pio:
        return env_pio
    return shutil.which("pio") or shutil.which("platformio")


def run_capture(cmd: Sequence[str], output_path: Path) -> None:
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
    print(proc.stdout)
    if proc.returncode != 0:
        raise subprocess.CalledProcessError(proc.returncode, cmd)


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
    for env in envs:
        run_capture([pio, "run", "-e", env, "-t", "size"], out_dir / f"{env}.txt")

    print(f"\n# firmware size reports saved to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
