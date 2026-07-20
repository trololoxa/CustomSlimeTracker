#!/usr/bin/env python3
"""Validate project documentation against the committed firmware baseline.

This deliberately checks only stable, high-value contracts. It is not a prose
linter. The goal is to catch broken local links and a few stale statements that
previously contradicted `platformio.ini` or the real runtime.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"

DEFAULT_ENV = "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG"
COMMITTED_ENVS = (
    "BOARD_LOLIN_C3_MINI_DEBUG",
    "BOARD_LOLIN_C3_MINI_PRODUCTION",
    "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG",
    "BOARD_LOLIN_C3_MINI_SLIM",
)

REQUIRED_DOCS = (
    "current_implementation.md",
    "architecture.md",
    "build_profiles.md",
    "cli_reference.md",
    "config_schema.md",
    "coordinate_frames.md",
    "module_inventory.md",
    "profile_contract.md",
    "project_status.md",
    "slimevr_network.md",
    "source_filter_matrix.md",
    "testing.md",
    "tracking_pipeline.md",
)

LOCAL_LINK_RE = re.compile(r"\[[^\]]*\]\(([^)]+)\)")
STALE_PHRASES = {
    "docs/project_status.md": (
        "The old DIAG alias was removed",
        "use the explicit Debug/Production/Slim environments",
    ),
}


def strip_link_target(raw: str) -> str:
    target = raw.strip()
    if target.startswith("<") and target.endswith(">"):
        target = target[1:-1]
    # Markdown permits an optional title after a whitespace separator. Project
    # local links do not use spaces in paths, so keep the first token.
    target = target.split(maxsplit=1)[0]
    target = target.split("#", 1)[0].split("?", 1)[0]
    return unquote(target)


def check_local_links(errors: list[str]) -> int:
    checked = 0
    for doc in sorted(DOCS.glob("*.md")):
        text = doc.read_text(encoding="utf-8")
        for match in LOCAL_LINK_RE.finditer(text):
            raw = match.group(1).strip()
            if not raw or raw.startswith(("#", "http://", "https://", "mailto:")):
                continue
            target = strip_link_target(raw)
            if not target:
                continue
            resolved = (doc.parent / target).resolve()
            checked += 1
            try:
                resolved.relative_to(ROOT.resolve())
            except ValueError:
                errors.append(f"{doc.relative_to(ROOT)}: local link escapes repository: {raw}")
                continue
            if not resolved.exists():
                errors.append(f"{doc.relative_to(ROOT)}: broken local link: {raw}")
    return checked


def require_contains(errors: list[str], rel: str, needles: tuple[str, ...]) -> None:
    path = ROOT / rel
    if not path.exists():
        errors.append(f"missing required documentation file: {rel}")
        return
    text = path.read_text(encoding="utf-8")
    for needle in needles:
        if needle not in text:
            errors.append(f"{rel}: missing required contract text: {needle!r}")


def main() -> int:
    errors: list[str] = []

    for name in REQUIRED_DOCS:
        if not (DOCS / name).is_file():
            errors.append(f"missing required documentation file: docs/{name}")

    checked_links = check_local_links(errors)

    require_contains(
        errors,
        "docs/build_profiles.md",
        (*COMMITTED_ENVS, DEFAULT_ENV, "committed default environment"),
    )
    require_contains(
        errors,
        "docs/testing.md",
        (*COMMITTED_ENVS, "Hardware/runtime test budget"),
    )
    require_contains(
        errors,
        "docs/current_implementation.md",
        (
            DEFAULT_ENV,
            "git rev-parse --short HEAD",
            "Known limitations and planned corrections",
            "Acceleration packet 4 is not emitted",
        ),
    )
    require_contains(
        errors,
        "docs/source_filter_matrix.md",
        ("## Production Diagnostic", "runtime/runtime_profiler.cpp"),
    )
    require_contains(
        errors,
        "docs/coordinate_frames.md",
        ("sensorToDevice is not applied by the runtime",),
    )
    require_contains(
        errors,
        "docs/production_firmware_roadmap.md",
        ("Historical planning document",),
    )

    for rel, phrases in STALE_PHRASES.items():
        path = ROOT / rel
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8")
        for phrase in phrases:
            if phrase in text:
                errors.append(f"{rel}: stale statement is forbidden: {phrase!r}")

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    print(f"# validate_documentation: OK ({len(REQUIRED_DOCS)} required docs, {checked_links} local links)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
