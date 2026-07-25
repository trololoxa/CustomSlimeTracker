#!/usr/bin/env python3
"""
Parse E0 machine-readable tracker logs.

Input: Serial log containing LOGVER/LOGFMT/Q/FIFO/CAL/BIAS/BIASUPD/MAG/MAGR/YAW/STATE/LOGSUM/TEMPBIN lines.
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

PREFIXES = {"LOGVER", "LOGFMT", "Q", "FIFO", "CAL", "BIAS", "BIASUPD", "MAG", "MAGR", "YAW", "STATE", "LOGSUM", "LOGSTAT", "TEMPBIN"}


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


def span(values: Iterable[float]) -> float:
    xs = [v for v in values if math.isfinite(v)]
    return max(xs) - min(xs) if xs else 0.0


def ratio(numerator: float, denominator: float) -> float:
    return float(numerator) / float(denominator) if denominator else 0.0


def metric(summary: Dict[str, Any], dotted: str, default: Any = 0) -> Any:
    cur: Any = summary
    for part in dotted.split('.'):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


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
    logver_rows = rows.get("LOGVER", [])
    q_rows = rows.get("Q", [])
    fifo_rows = rows.get("FIFO", [])
    cal_rows = rows.get("CAL", [])
    bias_rows = rows.get("BIAS", [])
    biasupd_rows = rows.get("BIASUPD", [])
    tempbin_rows = rows.get("TEMPBIN", [])
    mag_rows = rows.get("MAG", [])
    magr_rows = rows.get("MAGR", [])
    yaw_rows = rows.get("YAW", [])
    state_rows = rows.get("STATE", [])
    logsum_rows = rows.get("LOGSUM", [])
    logstat_rows = rows.get("LOGSTAT", [])

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
    field_states = Counter(r[14] for r in mag_rows if len(r) > 14)
    field_trusted = sum(to_int(r[15]) for r in mag_rows if len(r) > 15)
    field_flags = Counter(r[16] for r in mag_rows if len(r) > 16 and r[16] != "0x0")
    field_norm_error = [to_float(r[17]) for r in mag_rows if len(r) > 17]
    field_dip_error = [to_float(r[18]) for r in mag_rows if len(r) > 18]
    field_heading_error = [to_float(r[19]) for r in mag_rows if len(r) > 19]

    magr_raw_x = [to_float(r[4]) for r in magr_rows if len(r) > 18]
    magr_raw_y = [to_float(r[5]) for r in magr_rows if len(r) > 18]
    magr_raw_z = [to_float(r[6]) for r in magr_rows if len(r) > 18]
    magr_cal_x = [to_float(r[7]) for r in magr_rows if len(r) > 18]
    magr_cal_y = [to_float(r[8]) for r in magr_rows if len(r) > 18]
    magr_cal_z = [to_float(r[9]) for r in magr_rows if len(r) > 18]
    magr_body_x = [to_float(r[10]) for r in magr_rows if len(r) > 18]
    magr_body_y = [to_float(r[11]) for r in magr_rows if len(r) > 18]
    magr_body_z = [to_float(r[12]) for r in magr_rows if len(r) > 18]
    magr_raw_norm = [to_float(r[13]) for r in magr_rows if len(r) > 18]
    magr_cal_norm = [to_float(r[14]) for r in magr_rows if len(r) > 18]
    magr_body_norm = [to_float(r[15]) for r in magr_rows if len(r) > 18]
    magr_raw_flags = Counter(r[16] for r in magr_rows if len(r) > 18 and r[16] != "0x0")
    magr_reject_flags = Counter(r[17] for r in magr_rows if len(r) > 18 and r[17] != "0x0")
    magr_trusted = sum(to_int(r[18]) for r in magr_rows if len(r) > 18)
    magr_span_x = span(magr_raw_x)
    magr_span_y = span(magr_raw_y)
    magr_span_z = span(magr_raw_z)
    magr_min_axis_span = min(magr_span_x, magr_span_y, magr_span_z) if magr_rows else 0.0

    yaw_applied = sum(to_int(r[6]) for r in yaw_rows if len(r) > 6)
    yaw_gate_open = sum(to_int(r[4]) for r in yaw_rows if len(r) > 4)
    yaw_trust = [to_float(r[9]) for r in yaw_rows if len(r) > 9]
    yaw_step = [abs(to_float(r[8])) for r in yaw_rows if len(r) > 8]
    yaw_reject_flags = Counter(r[10] for r in yaw_rows if len(r) > 10 and r[10] != "0x0")
    yaw_modes = Counter(r[12] for r in yaw_rows if len(r) > 12)
    yaw_reacquire_pending = sum(to_int(r[13]) for r in yaw_rows if len(r) > 13)
    yaw_reacquire_active = sum(to_int(r[14]) for r in yaw_rows if len(r) > 14)
    yaw_field_stable_ms = [to_int(r[15]) for r in yaw_rows if len(r) > 15]
    yaw_heading_rate = [to_float(r[16]) for r in yaw_rows if len(r) > 16]

    state_events = Counter(r[3] for r in state_rows if len(r) > 3)
    state_reasons = Counter(r[4] for r in state_rows if len(r) > 4)

    logsum_last: Dict[str, Any] = {}
    if logsum_rows:
        r = logsum_rows[-1]
        # LOGSUM,uptime_ms,mode,rate_hz,q,cal,fifo,mag,yaw,state,bias,samples,quality_samples,fifo_overruns,fifo_full,large_gaps,recoveries,mag_trusted,mag_rejected,yaw_applied
        names = [
            "uptime_ms", "mode", "rate_hz", "q", "cal", "fifo", "mag", "yaw",
            "state", "bias", "samples", "quality_samples", "fifo_overruns",
            "fifo_full", "large_gaps", "recoveries", "mag_trusted",
            "mag_rejected", "yaw_applied",
        ]
        for idx, name in enumerate(names, start=1):
            if idx >= len(r):
                break
            logsum_last[name] = r[idx] if name == "mode" else to_int(r[idx])

    logstat: Dict[str, Dict[str, Any]] = {}
    for r in logstat_rows:
        if len(r) < 3:
            continue
        name = r[1].lower()
        values = [to_float(x) for x in r[2:]]
        logstat[name] = {"values": values}

    q_count = len(q_rows)
    fifo_count = len(fifo_rows)
    mag_count = len(mag_rows)
    magr_count = len(magr_rows)
    yaw_count = len(yaw_rows)

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

    logver: Dict[str, Any] = {}
    if logver_rows:
        latest = logver_rows[-1]
        if len(latest) >= 3:
            logver["schema_version"] = latest[1]
            logver["schema_name"] = latest[2]
        for i in range(3, len(latest) - 1, 2):
            logver[latest[i]] = latest[i + 1]

    out: Dict[str, Any] = {
        "counts": {k: len(v) for k, v in sorted(rows.items())},
        "logver": logver,
        "duration_s": round(duration_s, 3),
        "rates_hz": {
            "q": round(ratio(q_count, duration_s), 3),
            "fifo": round(ratio(fifo_count, duration_s), 3),
            "mag": round(ratio(mag_count, duration_s), 3),
            "magr": round(ratio(magr_count, duration_s), 3),
            "yaw": round(ratio(yaw_count, duration_s), 3),
        },
        "coverage": {
            "has_header": bool(rows.get("LOGVER") and rows.get("LOGFMT")),
            "has_q": q_count > 0,
            "has_fifo": fifo_count > 0,
            "has_cal": len(cal_rows) > 0,
            "has_bias": len(bias_rows) > 0,
            "has_bias_updates": len(biasupd_rows) > 0,
            "has_mag": mag_count > 0,
            "has_magr": magr_count > 0,
            "has_yaw": yaw_count > 0,
            "has_state_events": len(state_rows) > 0,
            "has_summary": len(logsum_rows) > 0,
        },
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
            "trusted_ratio": round(ratio(mag_trusted, mag_count), 6),
            "rejected_ratio": round(ratio(mag_rejected, mag_count), 6),
            "heading_valid_rows": heading_valid,
            "heading_valid_ratio": round(ratio(heading_valid, mag_count), 6),
            "heading_innovation_abs_deg": stats(heading_innov),
            "reject_flags_top": mag_reject_flags.most_common(8),
            "field_states": dict(field_states),
            "field_trusted_rows": field_trusted,
            "field_trusted_ratio": round(ratio(field_trusted, mag_count), 6),
            "field_flags_top": field_flags.most_common(8),
            "field_norm_relative_error": stats(field_norm_error),
            "field_dip_error_deg": stats(field_dip_error),
            "field_heading_error_deg": stats(field_heading_error),
        },
        "magr": {
            "rows": magr_count,
            "trusted_rows": magr_trusted,
            "trusted_ratio": round(ratio(magr_trusted, magr_count), 6),
            "raw_norm": stats(magr_raw_norm),
            "calibrated_norm": stats(magr_cal_norm),
            "body_norm": stats(magr_body_norm),
            "raw_xyz": {
                "x": stats(magr_raw_x),
                "y": stats(magr_raw_y),
                "z": stats(magr_raw_z),
            },
            "calibrated_xyz": {
                "x": stats(magr_cal_x),
                "y": stats(magr_cal_y),
                "z": stats(magr_cal_z),
            },
            "body_xyz": {
                "x": stats(magr_body_x),
                "y": stats(magr_body_y),
                "z": stats(magr_body_z),
            },
            "raw_axis_span": {
                "x": round(magr_span_x, 6),
                "y": round(magr_span_y, 6),
                "z": round(magr_span_z, 6),
                "min_axis": round(magr_min_axis_span, 6),
            },
            "raw_flags_top": magr_raw_flags.most_common(8),
            "reject_flags_top": magr_reject_flags.most_common(8),
        },
        "yaw": {
            "rows": len(yaw_rows),
            "gate_open_rows": yaw_gate_open,
            "gate_open_ratio": round(ratio(yaw_gate_open, yaw_count), 6),
            "applied_rows": yaw_applied,
            "applied_ratio": round(ratio(yaw_applied, yaw_count), 6),
            "trust": stats(yaw_trust),
            "step_abs_deg": stats(yaw_step),
            "reject_flags_top": yaw_reject_flags.most_common(8),
            "modes": dict(yaw_modes),
            "reacquire_pending_rows": yaw_reacquire_pending,
            "reacquire_active_rows": yaw_reacquire_active,
            "field_stable_ms": stats(yaw_field_stable_ms),
            "heading_rate_deg_s": stats(yaw_heading_rate),
        },
        "state_events": {
            "states": dict(state_events),
            "reasons": dict(state_reasons),
        },
        "logsum_last": logsum_last,
        "logstat": logstat,
        "warnings": warnings,
    }

    if include_samples > 0:
        out["samples"] = {}
        for key in ["Q", "FIFO", "CAL", "BIAS", "BIASUPD", "MAG", "MAGR", "YAW", "STATE", "LOGSUM", "LOGSTAT", "TEMPBIN"]:
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
