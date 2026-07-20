#!/usr/bin/env python3
"""Validate documentation structure without pretending to validate prose meaning.

Semantic accuracy is reviewed against code. Automation only catches missing
canonical documents and broken repository-local links.
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


def main() -> int:
    errors: list[str] = []

    for name in REQUIRED_DOCS:
        if not (DOCS / name).is_file():
            errors.append(f"missing required documentation file: docs/{name}")

    checked_links = check_local_links(errors)

    # Documentation meaning is reviewed with the code change. Automated checks
    # deliberately stop at structural failures: missing canonical documents and
    # broken repository-local links. Profile/runtime contracts belong in their
    # dedicated code/config validators rather than brittle prose assertions.

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    print(f"# validate_documentation: OK ({len(REQUIRED_DOCS)} required docs, {checked_links} local links)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
