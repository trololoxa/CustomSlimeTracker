#!/usr/bin/env python3
"""
Parse E0 machine-readable tracker logs.

Input: Serial log containing LOGVER/LOGFMT/Q/FIFO/CAL/BIAS/BIASUPD/MAG/YAW/STATE/LOGSUM/TEMPBIN lines.
Output: compact JSON summary that is small enough to paste into ChatGPT.

Usage:
  python tools/logs/parse_e0_log.py tracker.log
  python tools/logs/parse_e0_log.py tracker.log --include-samples 5
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

PREFIXES = {"LOGVER", "LOGFMT", "Q", "FIFO", "CAL", "BIAS", "BIASUPD", "MAG", "YAW", "STATE", "LOGSUM", "LOGSTAT", "TEMPBIN"}


def to_float(x: str, default: float = 0.0) -> float:
    try:
        v = float(x)
        return v if math.isfinite(v) else default
    except Exception:
        return default


def to_int(x: str, default: int = 0) -> int:
    try:
        return int(x, 0)
    except Exception:
        try:
            return int(float(x))
        except Exception:
            return default


def yaw_from_quat_wxyz(w: float, x: float, y: float, z: float) -> float:
    # ZYX yaw in degrees. This is only a diagnostic projection; firmware frame docs still win.
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def angle_diff_deg(a: float, b: float) -> float:
    d = (b - a + 180.0) % 360.0 - 180.0
    return d


def stats(values: Iterable[float]) -> Dict[str, float]:
    xs = [v for v in values if math.isfinite(v)]
    if not xs:
        return {"count": 0, "min": 0.0, "mean": 0.0, "max": 0.0}
    return {
        "count": len(xs),
        "min": min(xs),
        "mean": sum(xs) / len(xs),
        "max": max(xs),
    }


def parse_rows(path: Path) -> Dict[str, List[List[str]]]:
    rows: Dict[str, List[List[str]]] = defaultdict(list)
    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            # Human-readable CLI/status lines are intentionally ignored.
            prefix = line.split(",", 1)[0]
            if prefix not in PREFIXES:
                continue
            try:
                row = next(csv.reader([line]))
            except Exception:
                continue
            if row:
                rows[row[0]].append(row)
    return rows


def summarize(rows: Dict[str, List[List[str]]], include_samples: int = 0) -> Dict[str, Any]:
    q_rows = rows.get("Q", [])
    fifo_rows = rows.get("FIFO", [])
    cal_rows = rows.get("CAL", [])
    bias_rows = rows.get("BIAS", [])
    biasupd_rows = rows.get("BIASUPD", [])
    tempbin_rows = rows.get("TEMPBIN", [])
    mag_rows = rows.get("MAG", [])
    yaw_rows = rows.get("YAW", [])
    state_rows = rows.get("STATE", [])

    q_t = [to_int(r[1]) for r in q_rows if len(r) > 2]
    duration_s = (max(q_t) - min(q_t)) / 1_000_000.0 if len(q_t) >= 2 else 0.0

    q_conf = [to_float(r[9]) for r in q_rows if len(r) > 9]
    q_states = Counter(r[10] for r in q_rows if len(r) > 10)
    accel_trust = [to_float(r[11]) for r in q_rows if len(r) > 11]
    accel_norm = [to_float(r[12]) for r in q_rows if len(r) > 12]
    gyro_dps = [to_float(r[15]) for r in q_rows if len(r) > 15]

    yaws: List[float] = []
    for r in q_rows:
        if len(r) > 8:
            w, x, y, z = map(to_float, r[4:8])
            yaws.append(yaw_from_quat_wxyz(w, x, y, z))
    yaw_delta = angle_diff_deg(yaws[0], yaws[-1]) if len(yaws) >= 2 else 0.0
    yaw_drift_deg_min = yaw_delta / (duration_s / 60.0) if duration_s > 1e-6 else 0.0

    fifo_dt = [to_int(r[3]) for r in fifo_rows if len(r) > 3]
    fallback = sum(to_int(r[5]) for r in fifo_rows if len(r) > 5)
    hw = sum(to_int(r[4]) for r in fifo_rows if len(r) > 4)
    dropped = sum(to_int(r[6]) for r in fifo_rows if len(r) > 6)
    overrun = sum(to_int(r[7]) for r in fifo_rows if len(r) > 7)
    full = sum(to_int(r[8]) for r in fifo_rows if len(r) > 8)
    unknown = sum(to_int(r[9]) for r in fifo_rows if len(r) > 9)

    cal_accel_norm = []
    cal_gyro_norm_dps = []
    for r in cal_rows:
        if len(r) > 9:
            ax, ay, az = map(to_float, r[3:6])
            gx, gy, gz = map(to_float, r[6:9])
            cal_accel_norm.append(math.sqrt(ax * ax + ay * ay + az * az))
            cal_gyro_norm_dps.append(math.degrees(math.sqrt(gx * gx + gy * gy + gz * gz)))

    bias_temp = [to_float(r[3]) for r in bias_rows if len(r) > 3]
    bias_norm = []
    bias_flags = Counter()
    bias_sources = Counter()
    rt_updates_last = 0
    for r in bias_rows:
        if len(r) > 10:
            bx, by, bz = map(to_float, r[4:7])
            bias_norm.append(math.sqrt(bx * bx + by * by + bz * bz))
            bias_sources[r[7]] += 1
            if r[9] != "0x0":
                bias_flags[r[9]] += 1
            rt_updates_last = max(rt_updates_last, to_int(r[11]) if len(r) > 11 else 0)

    biasupd_temp = [to_float(r[3]) for r in biasupd_rows if len(r) > 3]
    biasupd_residual_norm = []
    biasupd_delta_norm = []
    biasupd_trim_norm = []
    biasupd_flags = Counter()
    for r in biasupd_rows:
        if len(r) > 16:
            rx, ry, rz = map(to_float, r[4:7])
            dx, dy, dz = map(to_float, r[10:13])
            tx, ty, tz = map(to_float, r[13:16])
            biasupd_residual_norm.append(math.sqrt(rx * rx + ry * ry + rz * rz))
            biasupd_delta_norm.append(math.sqrt(dx * dx + dy * dy + dz * dz))
            biasupd_trim_norm.append(math.sqrt(tx * tx + ty * ty + tz * tz))
            if r[16] != "0x0":
                biasupd_flags[r[16]] += 1

    tempbin_temps = [to_float(r[4]) for r in tempbin_rows if len(r) > 5]
    tempbin_samples = sum(to_int(r[5]) for r in tempbin_rows if len(r) > 5)

    mag_trusted = sum(to_int(r[11]) for r in mag_rows if len(r) > 11)
    mag_rejected = len(mag_rows) - mag_trusted
    mag_reject_flags = Counter(r[12] for r in mag_rows if len(r) > 12 and r[12] != "0x0")
    heading_valid = sum(to_int(r[8]) for r in mag_rows if len(r) > 8)
    heading_innov = [abs(to_float(r[10])) for r in mag_rows if len(r) > 10]

    yaw_applied = sum(to_int(r[6]) for r in yaw_rows if len(r) > 6)
    yaw_gate_open = sum(to_int(r[4]) for r in yaw_rows if len(r) > 4)
    yaw_trust = [to_float(r[9]) for r in yaw_rows if len(r) > 9]
    yaw_step = [abs(to_float(r[8])) for r in yaw_rows if len(r) > 8]
    yaw_reject_flags = Counter(r[10] for r in yaw_rows if len(r) > 10 and r[10] != "0x0")

    state_events = Counter(r[3] for r in state_rows if len(r) > 3)
    state_reasons = Counter(r[4] for r in state_rows if len(r) > 4)

    warnings: List[str] = []
    if fallback > 0:
        warnings.append(f"fallback timestamps present: {fallback}")
    if overrun or full or unknown:
        warnings.append(f"FIFO faults overrun={overrun} full={full} unknown={unknown}")
    if q_states.get("RECOVERING", 0) or state_events.get("RECOVERING", 0):
        warnings.append("tracking recovery occurred")
    if accel_trust and min(accel_trust) < 0.2:
        warnings.append("accel trust dropped below 0.2")
    if yaw_rows and yaw_applied == 0:
        warnings.append("mag yaw rows present but no yaw correction applied")
    if any((to_int(flag) & 0x8) != 0 for flag in bias_flags):
        warnings.append("temperature compensation out of calibrated range")

    out: Dict[str, Any] = {
        "counts": {k: len(v) for k, v in sorted(rows.items())},
        "duration_s": round(duration_s, 3),
        "q": {
            "states": dict(q_states),
            "confidence": stats(q_conf),
            "yaw_delta_deg": round(yaw_delta, 4),
            "yaw_drift_deg_min_diag": round(yaw_drift_deg_min, 6),
            "accel_trust": stats(accel_trust),
            "accel_norm_g": stats(accel_norm),
            "gyro_dps": stats(gyro_dps),
        },
        "fifo": {
            "dt_us": stats(fifo_dt),
            "hw_rows": hw,
            "fallback_rows": fallback,
            "dropped_before_sum": dropped,
            "overrun_rows": overrun,
            "full_rows": full,
            "unknown_rows": unknown,
        },
        "cal": {
            "accel_norm_g": stats(cal_accel_norm),
            "gyro_norm_dps": stats(cal_gyro_norm_dps),
        },
        "bias": {
            "rows": len(bias_rows),
            "temp_c": stats(bias_temp),
            "bias_norm_dps": stats(bias_norm),
            "sources": dict(bias_sources),
            "flags_top": bias_flags.most_common(8),
            "runtime_updates_last": rt_updates_last,
        },
        "bias_updates": {
            "rows": len(biasupd_rows),
            "temp_c": stats(biasupd_temp),
            "residual_norm_dps": stats(biasupd_residual_norm),
            "delta_norm_dps": stats(biasupd_delta_norm),
            "trim_norm_dps": stats(biasupd_trim_norm),
            "flags_top": biasupd_flags.most_common(8),
        },
        "tempbins": {
            "rows": len(tempbin_rows),
            "temp_c": stats(tempbin_temps),
            "samples_sum": tempbin_samples,
        },
        "mag": {
            "trusted_rows": mag_trusted,
            "rejected_rows": mag_rejected,
            "heading_valid_rows": heading_valid,
            "heading_innovation_abs_deg": stats(heading_innov),
            "reject_flags_top": mag_reject_flags.most_common(8),
        },
        "yaw": {
            "rows": len(yaw_rows),
            "gate_open_rows": yaw_gate_open,
            "applied_rows": yaw_applied,
            "trust": stats(yaw_trust),
            "step_abs_deg": stats(yaw_step),
            "reject_flags_top": yaw_reject_flags.most_common(8),
        },
        "state_events": {
            "states": dict(state_events),
            "reasons": dict(state_reasons),
        },
        "warnings": warnings,
    }

    if include_samples > 0:
        out["samples"] = {}
        for key in ["Q", "FIFO", "CAL", "BIAS", "BIASUPD", "MAG", "YAW", "STATE", "LOGSUM", "LOGSTAT", "TEMPBIN"]:
            rs = rows.get(key, [])
            if rs:
                out["samples"][key] = rs[:include_samples] + ([ ["..."] ] if len(rs) > 2 * include_samples else []) + rs[-include_samples:]

    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path)
    ap.add_argument("--include-samples", type=int, default=0)
    ap.add_argument("--pretty", action="store_true")
    args = ap.parse_args()

    rows = parse_rows(args.log)
    summary = summarize(rows, include_samples=max(0, args.include_samples))
    print(json.dumps(summary, ensure_ascii=False, indent=2 if args.pretty else None, sort_keys=True))


if __name__ == "__main__":
    main()
