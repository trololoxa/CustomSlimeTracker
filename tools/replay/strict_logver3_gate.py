#!/usr/bin/env python3
"""Run the strict LOGVER3 E1 integrity gate and optional static golden."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

from logver3_contract import validate_logver3


def dotted(data: dict[str, Any], path: str) -> Any:
    value: Any = data
    for part in path.split("."):
        if not isinstance(value, dict) or part not in value:
            raise KeyError(path)
        value = value[part]
    return value


def apply_golden(report: dict[str, Any], fixture: Path, golden_path: Path) -> None:
    try:
        golden = json.loads(golden_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        report["failures"].append(f"cannot read golden JSON: {exc}")
        report["passed"] = False
        return
    if golden.get("contract") != report.get("contract"):
        report["failures"].append("golden contract identifier mismatch")
    digest = hashlib.sha256(fixture.read_bytes()).hexdigest()
    if golden.get("fixture_sha256") != digest:
        report["failures"].append("golden fixture_sha256 mismatch")
    expectations = golden.get("expect")
    if not isinstance(expectations, dict) or not expectations:
        report["failures"].append("golden expect map is missing/empty")
    else:
        for path, rule in expectations.items():
            if not isinstance(rule, dict) or not rule:
                report["failures"].append(f"golden rule {path!r} is invalid")
                continue
            try:
                actual = dotted(report, path)
            except KeyError:
                report["failures"].append(f"golden metric {path!r} is absent")
                continue
            if "eq" in rule and actual != rule["eq"]:
                report["failures"].append(f"{path}={actual!r}, expected {rule['eq']!r}")
            if "min" in rule and actual < rule["min"]:
                report["failures"].append(f"{path}={actual!r} < {rule['min']!r}")
            if "max" in rule and actual > rule["max"]:
                report["failures"].append(f"{path}={actual!r} > {rule['max']!r}")
    report["fixture_sha256"] = digest
    report["passed"] = not report["failures"]


def main() -> int:
    parser = argparse.ArgumentParser(description="Strict LOGVER3 E1 diagnostic-capture gate")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--log", type=Path, help="candidate capture to validate")
    source.add_argument("--fixture", type=Path, help="release fixture to validate")
    parser.add_argument("--golden", type=Path, help="independent threshold/fixture contract JSON")
    parser.add_argument("--capture", default="static", choices=("static", "runtime"))
    parser.add_argument("--output", type=Path, help="write the validation report as JSON")
    parser.add_argument("--min-duration-s", type=float, default=590.0)
    parser.add_argument("--min-rate-hz", type=float, default=19.0)
    parser.add_argument("--max-rate-hz", type=float, default=21.0)
    parser.add_argument("--max-record-age-us", type=int, default=250_000)
    parser.add_argument("--expected-profile", default="ProductionDiag")
    parser.add_argument("--expected-environment", default="BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG")
    parser.add_argument("--no-console-status", action="store_true")
    args = parser.parse_args()

    if args.min_duration_s < 0 or args.min_rate_hz <= 0 or args.max_rate_hz < args.min_rate_hz:
        parser.error("invalid duration/rate bounds")
    if args.max_record_age_us <= 0:
        parser.error("--max-record-age-us must be positive")

    capture = args.log or args.fixture
    assert capture is not None
    report = validate_logver3(
        capture,
        capture_kind=args.capture,
        min_duration_s=args.min_duration_s,
        min_rate_hz=args.min_rate_hz,
        max_rate_hz=args.max_rate_hz,
        max_record_age_us=args.max_record_age_us,
        expected_profile=args.expected_profile,
        expected_environment=args.expected_environment,
        require_console_status=not args.no_console_status,
    )
    if args.fixture is not None and args.golden is None:
        report["failures"].append("release fixture requires --golden")
        report["passed"] = False
    if args.golden is not None:
        if args.capture != "static":
            parser.error("--golden is valid only with --capture static")
        apply_golden(report, capture, args.golden)

    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
