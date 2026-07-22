#!/usr/bin/env python3
"""Pure classification/parsing helpers for tools/check_all.py."""

from __future__ import annotations

import re
from dataclasses import dataclass

SIZE_OVERFLOW_PATTERNS = (
    re.compile(r"region [`'].*[`'] overflowed by", re.IGNORECASE),
    re.compile(r"will not fit in region", re.IGNORECASE),
    re.compile(r"program size .* greater than maximum allowed", re.IGNORECASE),
    re.compile(r"sketch too big", re.IGNORECASE),
    re.compile(r"flash overflowed by", re.IGNORECASE),
    re.compile(r"ram overflowed by", re.IGNORECASE),
)

SOURCE_ERROR_RE = re.compile(r"^.+?:[0-9]+(?::[0-9]+)?:\s+(?:fatal\s+)?error:", re.IGNORECASE | re.MULTILINE)
NON_SIZE_LINK_FAILURE_PATTERNS = (
    re.compile(r"fatal error:", re.IGNORECASE),
    re.compile(r"undefined reference", re.IGNORECASE),
    re.compile(r"multiple definition", re.IGNORECASE),
    re.compile(r"cannot find -l", re.IGNORECASE),
    re.compile(r"No such file or directory", re.IGNORECASE),
)

SIZE_LINE_RE = re.compile(
    r"^(RAM|Flash):\s+.*?([0-9]+(?:\.[0-9]+)?)%\s+\(used\s+([0-9]+)\s+bytes\s+from\s+([0-9]+)\s+bytes\)",
    re.MULTILINE,
)


@dataclass(frozen=True)
class SizeMetric:
    kind: str
    percent: float
    used: int
    total: int


def is_size_overflow(output: str) -> bool:
    return any(pattern.search(output) for pattern in SIZE_OVERFLOW_PATTERNS)


def has_non_size_build_failure(output: str) -> bool:
    if SOURCE_ERROR_RE.search(output):
        return True
    return any(pattern.search(output) for pattern in NON_SIZE_LINK_FAILURE_PATTERNS)


def is_size_only_failure(output: str) -> bool:
    return is_size_overflow(output) and not has_non_size_build_failure(output)


def parse_size_metrics(output: str) -> tuple[SizeMetric, ...]:
    metrics = []
    for match in SIZE_LINE_RE.finditer(output):
        metrics.append(
            SizeMetric(
                kind=match.group(1),
                percent=float(match.group(2)),
                used=int(match.group(3)),
                total=int(match.group(4)),
            )
        )
    return tuple(metrics)
