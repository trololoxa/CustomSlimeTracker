#!/usr/bin/env python3
"""Read-only identity check before moving verification between checkouts."""
from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path

from quality_gate_runtime import run_bounded_process

ROOT = Path(__file__).resolve().parents[1]


def check(root: Path, expected: str | None) -> tuple[int, str]:
    if expected is not None and not re.fullmatch(r"[0-9a-fA-F]{40}|[0-9a-fA-F]{64}", expected):
        return 2, "FAIL expected commit must be a complete hexadecimal Git object ID"
    try:
        head = run_bounded_process(["git", "rev-parse", "HEAD"], cwd=root, timeout_s=5,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        status = run_bounded_process(["git", "status", "--porcelain", "--untracked-files=normal"],
                                     cwd=root, timeout_s=5,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    except (OSError, subprocess.TimeoutExpired):
        return 1, "FAIL Git identity unavailable (missing executable or deadline)"
    if head.returncode or status.returncode:
        return 1, "FAIL Git identity unavailable; run git status locally"
    commit = head.stdout.strip()
    if not re.fullmatch(r"[0-9a-fA-F]{40}|[0-9a-fA-F]{64}", commit):
        return 1, "FAIL invalid Git HEAD"
    if status.stdout.strip():
        return 1, f"FAIL source={commit} dirty; review git status --short; nothing changed"
    if expected is not None and commit.lower() != expected.lower():
        return 1, f"FAIL source={commit} expected={expected.lower()}; nothing changed"
    return 0, f"PASS source={commit} clean (ignored inputs and submodule contents not certified)"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expect-commit", help="full commit printed by the source checkout")
    args = parser.parse_args()
    code, message = check(ROOT, args.expect_commit)
    print(message)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
