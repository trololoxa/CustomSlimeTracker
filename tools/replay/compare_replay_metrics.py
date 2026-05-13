#!/usr/bin/env python3
"""Compare two replay metric JSON files.

This is intentionally small and deterministic so it can be used both by humans
and by check_all.py. It compares a curated set of quality metrics and fails when
absolute or relative regressions exceed configurable thresholds.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def get_path(data: dict[str, Any], dotted: str, default: float = 0.0) -> float:
    cur: Any = data
    for part in dotted.split('.'):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    try:
        v = float(cur)
        return v if math.isfinite(v) else default
    except Exception:
        return default


DEFAULT_METRICS = [
    "duration_s",
    "counts.Q",
    "counts.FIFO",
    "counts.MAGR",
    "fifo.fallback_rows",
    "fifo.overrun_rows",
    "fifo.full_rows",
    "fifo.unknown_rows",
    "q.confidence.min",
    "q.accel_trust.min",
    "q.yaw_drift_deg_min_diag",
    "mag.trusted_ratio",
    "mag.rejected_ratio",
    "magr.trusted_ratio",
    "magr.raw_axis_span.min_axis",
    "yaw.applied_ratio",
    "state_events.states.RECOVERING",
    "bias.runtime_updates_last",
]

LOWER_IS_BETTER = {
    "fifo.fallback_rows",
    "fifo.overrun_rows",
    "fifo.full_rows",
    "fifo.unknown_rows",
    "q.yaw_drift_deg_min_diag",
    "mag.rejected_ratio",
    "state_events.states.RECOVERING",
}

HIGHER_IS_BETTER = {
    "duration_s",
    "counts.Q",
    "counts.FIFO",
    "counts.MAGR",
    "q.confidence.min",
    "q.accel_trust.min",
    "mag.trusted_ratio",
    "magr.trusted_ratio",
    "magr.raw_axis_span.min_axis",
    "yaw.applied_ratio",
    "bias.runtime_updates_last",
}


def compare_metric(metric: str, before: float, after: float, abs_tol: float, rel_tol: float) -> str | None:
    if metric in LOWER_IS_BETTER:
        delta = after - before
    elif metric in HIGHER_IS_BETTER:
        delta = before - after
    else:
        delta = abs(after - before)

    allowed = abs_tol + rel_tol * max(abs(before), 1.0)
    if delta > allowed:
        return f"{metric}: before={before:.9g} after={after:.9g} regression={delta:.9g} allowed={allowed:.9g}"
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare replay metric JSON files")
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--metric", action="append", dest="metrics", help="metric path to compare; may be repeated")
    parser.add_argument("--abs-tol", type=float, default=1.0e-6)
    parser.add_argument("--rel-tol", type=float, default=0.02)
    parser.add_argument("--pretty", action="store_true")
    args = parser.parse_args()

    before = json.loads(args.before.read_text(encoding="utf-8"))
    after = json.loads(args.after.read_text(encoding="utf-8"))
    metrics = args.metrics or DEFAULT_METRICS

    rows = []
    failures = []
    for metric in metrics:
        b = get_path(before, metric)
        a = get_path(after, metric)
        failure = compare_metric(metric, b, a, args.abs_tol, args.rel_tol)
        rows.append({"metric": metric, "before": b, "after": a, "failure": failure})
        if failure:
            failures.append(failure)

    result = {"passed": not failures, "failures": failures, "metrics": rows}
    print(json.dumps(result, ensure_ascii=False, indent=2 if args.pretty else None, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
