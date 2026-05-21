#!/usr/bin/env python3
"""Validate PlatformIO source filters used by firmware build profiles.

The goal is intentionally narrow: catch stale or misspelled `-<path>` entries
before a profile silently stops excluding a translation unit. The script does
not try to emulate PlatformIO's glob engine; it validates the concrete paths we
use in this project.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PLATFORMIO_INI = ROOT / "platformio.ini"
SRC = ROOT / "src"

EXCLUDE_RE = re.compile(r"^\s*-<([^>]+)>\s*(?:[;#].*)?$")
ENV_RE = re.compile(r"^\s*\[env:([^\]]+)\]\s*$")


def iter_filter_excludes() -> list[tuple[str, str, int]]:
    env = "<global>"
    out: list[tuple[str, str, int]] = []
    for lineno, line in enumerate(PLATFORMIO_INI.read_text(encoding="utf-8").splitlines(), start=1):
        env_match = ENV_RE.match(line)
        if env_match:
            env = env_match.group(1)
            continue
        match = EXCLUDE_RE.match(line)
        if match:
            out.append((env, match.group(1), lineno))
    return out


def main() -> int:
    errors: list[str] = []
    seen: set[tuple[str, str]] = set()

    for env, rel, lineno in iter_filter_excludes():
        key = (env, rel)
        if key in seen:
            errors.append(f"{PLATFORMIO_INI}:{lineno}: duplicate source filter exclude in {env}: {rel}")
            continue
        seen.add(key)

        # This project uses concrete file excludes. Keep globs out of the
        # profile matrix unless the validator is extended intentionally.
        if any(ch in rel for ch in "*?"):
            continue

        path = SRC / rel
        if not path.exists():
            errors.append(f"{PLATFORMIO_INI}:{lineno}: source filter exclude in {env} points to missing path: src/{rel}")

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    print(f"# validate_source_filters: OK ({len(seen)} excludes checked)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
