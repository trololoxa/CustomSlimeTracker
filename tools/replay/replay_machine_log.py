#!/usr/bin/env python3
"""Replay/score an E0 machine-readable tracker log.

This is the first lightweight replay gate. It does not re-run firmware AHRS yet;
it parses recorded LOGFMT/CSV frames and computes deterministic metrics that can
be compared between firmware versions.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "logs"))

from parse_e0_log import parse_rows, summarize  # noqa: E402


def get_path(data: dict[str, Any], dotted: str, default: Any = None) -> Any:
    cur: Any = data
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def evaluate(summary: dict[str, Any], args: argparse.Namespace) -> list[str]:
    failures: list[str] = []

    if args.min_duration_s is not None and summary.get("duration_s", 0.0) < args.min_duration_s:
        failures.append(f"duration_s < {args.min_duration_s}: {summary.get('duration_s', 0.0)}")

    if args.max_yaw_drift_deg_min is not None:
        drift = abs(float(get_path(summary, "q.yaw_drift_deg_min_diag", 0.0)))
        if drift > args.max_yaw_drift_deg_min:
            failures.append(f"abs(q.yaw_drift_deg_min_diag) > {args.max_yaw_drift_deg_min}: {drift}")

    if args.max_fifo_fallback_rows is not None:
        fallback = int(get_path(summary, "fifo.fallback_rows", 0))
        if fallback > args.max_fifo_fallback_rows:
            failures.append(f"fifo.fallback_rows > {args.max_fifo_fallback_rows}: {fallback}")

    if args.max_fifo_fault_rows is not None:
        fault_rows = (
            int(get_path(summary, "fifo.overrun_rows", 0))
            + int(get_path(summary, "fifo.full_rows", 0))
            + int(get_path(summary, "fifo.unknown_rows", 0))
        )
        if fault_rows > args.max_fifo_fault_rows:
            failures.append(f"fifo fault rows > {args.max_fifo_fault_rows}: {fault_rows}")

    if args.max_recovering_rows is not None:
        states = get_path(summary, "q.states", {}) or {}
        state_events = get_path(summary, "state_events.states", {}) or {}
        recovering = int(states.get("RECOVERING", 0)) + int(state_events.get("RECOVERING", 0))
        if recovering > args.max_recovering_rows:
            failures.append(f"RECOVERING rows/events > {args.max_recovering_rows}: {recovering}")

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay/score an E0 tracker machine log")
    parser.add_argument("log", type=Path, help="Serial log containing LOGVER/LOGFMT frames")
    parser.add_argument("--pretty", action="store_true", help="pretty-print JSON")
    parser.add_argument("--include-samples", type=int, default=0, help="include first/last N rows per frame type")
    parser.add_argument("--output", type=Path, help="write JSON summary to this path instead of stdout")
    parser.add_argument("--min-duration-s", type=float)
    parser.add_argument("--max-yaw-drift-deg-min", type=float)
    parser.add_argument("--max-fifo-fallback-rows", type=int)
    parser.add_argument("--max-fifo-fault-rows", type=int)
    parser.add_argument("--max-recovering-rows", type=int)
    args = parser.parse_args()

    rows = parse_rows(args.log)
    summary = summarize(rows, include_samples=max(0, args.include_samples))
    failures = evaluate(summary, args)
    summary["replay_gate"] = {"passed": not failures, "failures": failures}

    text = json.dumps(summary, ensure_ascii=False, indent=2 if args.pretty else None, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
