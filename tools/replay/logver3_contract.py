#!/usr/bin/env python3
"""Fail-closed structural and capture-integrity checks for tracker LOGVER3."""

from __future__ import annotations

import csv
import math
import re
from collections import Counter
from pathlib import Path
from typing import Any


SCHEMAS: dict[str, tuple[str, ...]] = {
    "Q": ("t_us", "seq", "dt_us", "w", "x", "y", "z", "qflags", "conf", "state",
          "acc_trust", "acc_norm_g", "acc_var_g2", "gyro_trust", "gyro_dps", "recovery"),
    "FIFO": ("t_us", "seq", "dt_us", "hw_ts", "fb_ts", "dropped_before", "overrun",
             "full", "unknown", "quality_flags"),
    "CAL": ("t_us", "seq", "ax_g", "ay_g", "az_g", "gx_rads", "gy_rads", "gz_rads",
            "temp_c", "quality_flags"),
    "BIAS": ("t_us", "seq", "temp_c", "bx_dps", "by_dps", "bz_dps", "source", "quality",
             "flags", "rt_enabled", "rt_updates"),
    "BIASUPD": ("t_us", "seq", "temp_c", "rx_dps", "ry_dps", "rz_dps", "sx_dps", "sy_dps",
                "sz_dps", "dx_dps", "dy_dps", "dz_dps", "trim_x_dps", "trim_y_dps",
                "trim_z_dps", "flags"),
    "MAG": ("t_us", "seq", "mag_seq", "age_ms", "raw_norm", "body_norm", "horiz_norm",
            "heading_valid", "heading_yaw_deg", "heading_innov_deg", "trusted", "reject_flags",
            "dip_deg", "field_state", "field_trusted", "field_flags", "field_norm_error",
            "field_dip_error_deg", "field_heading_error_deg"),
    "MAGR": ("t_us", "seq", "mag_seq", "raw_x", "raw_y", "raw_z", "cal_x", "cal_y", "cal_z",
             "body_x", "body_y", "body_z", "raw_norm", "cal_norm", "body_norm", "raw_flags",
             "reject_flags", "trusted"),
    "YAW": ("t_us", "seq", "valid", "gate_open", "apply_allowed", "applied", "error_deg",
            "step_deg", "trust", "reject_flags", "cooldown_ms", "mode", "reacquire_pending",
            "reacquire_active", "field_stable_ms", "heading_rate_deg_s"),
    "STATE": ("t_us", "seq", "state", "reason", "flags", "conf"),
    "NET": ("t_us", "seq", "wifi_connected", "wifi_disc", "wifi_timeouts",
            "rssi_dbm", "slime_state", "udp_ready", "server_found", "rotation_sent",
            "rotation_due", "rot_missed", "rot_late", "send_failures", "rot_fail",
            "control_fail", "telemetry_fail", "discovery_fail", "tx_pressure_fail",
            "tx_other_fail", "rebind_ok", "rebind_fail", "full_reopens",
            "consecutive_fail", "last_udp_error", "motion_tx_age_ms",
            "tx_pressure_state", "tx_recovery_reason"),
    "TESTSUM": ("kind", "duration_ms", "stopped", "samples", "hw_ts", "fb_ts", "bad_ts",
                "dropped", "recoveries", "fifo_overruns", "fifo_full", "fifo_unknown",
                "net_valid", "wifi_disc", "wifi_timeouts", "udp_fail", "rot_fail",
                "rot_missed", "rot_late", "tx_pressure_fail", "tx_other_fail", "rebind_ok",
                "rebind_fail", "full_reopens", "static_valid", "gyro_mean_dps",
                "gyro_std_dps", "accel_mean_g", "accel_std_g", "temp_start_c", "temp_end_c",
                "mag_valid", "mag_trusted", "mag_rejected"),
    "LOGSUM": ("uptime_ms", "mode", "rate_hz", "q", "cal", "fifo", "mag", "yaw", "state",
               "bias", "net", "samples", "quality_samples", "fifo_overruns", "fifo_full",
               "large_gaps", "recoveries", "mag_trusted", "mag_rejected", "yaw_applied"),
}

FRAME_TYPES = frozenset(SCHEMAS) - {"LOGSUM", "TESTSUM"}
KNOWN_PREFIXES = frozenset({"LOGVER", "LOGFMT", "LOGSTAT", "TEMPBIN", *SCHEMAS})
STRING_FIELDS = {("Q", "state"), ("BIAS", "source"), ("STATE", "state"),
                 ("STATE", "reason"), ("LOGSUM", "mode"), ("TESTSUM", "kind")}
HEX_FIELDS = {"qflags", "quality_flags", "flags", "raw_flags", "reject_flags", "field_flags"}
BOOL_FIELDS = {
    "recovery", "hw_ts", "fb_ts", "overrun", "full", "unknown", "rt_enabled",
    "heading_valid", "trusted", "field_trusted", "valid", "gate_open", "apply_allowed",
    "applied", "reacquire_pending", "reacquire_active", "wifi_connected", "udp_ready",
    "server_found", "stopped", "net_valid", "static_valid", "mag_valid",
}
UINT_FIELDS = {
    "t_us", "seq", "dt_us", "mag_seq", "age_ms", "dropped_before", "rt_updates",
    "field_state", "cooldown_ms", "mode", "field_stable_ms",
    "wifi_disc", "wifi_timeouts", "slime_state", "rotation_sent", "rotation_due",
    "rot_missed", "rot_late", "send_failures", "rot_fail", "control_fail",
    "telemetry_fail", "discovery_fail", "tx_pressure_fail", "tx_other_fail", "rebind_ok",
    "rebind_fail", "full_reopens", "consecutive_fail", "motion_tx_age_ms", "tx_pressure_state",
    "tx_recovery_reason", "duration_ms", "samples", "hw_ts", "fb_ts", "bad_ts",
    "dropped", "recoveries", "fifo_overruns", "fifo_full", "fifo_unknown", "udp_fail",
    "mag_trusted", "mag_rejected",
}
INT_FIELDS = {("NET", "rssi_dbm"), ("NET", "last_udp_error")}
HEX_RE = re.compile(r"0x[0-9a-fA-F]+\Z")
PLAIN_HEX_RE = re.compile(r"[0-9a-fA-F]+\Z")
UINT_RE = re.compile(r"[0-9]+\Z")
GIT_RE = re.compile(r"[0-9a-f]{40}\Z")
MACHINE_PREFIX_RE = re.compile(r"[A-Z][A-Z0-9_]*\Z")


def _uint(value: str) -> int:
    if not UINT_RE.fullmatch(value):
        raise ValueError(f"not an unsigned decimal integer: {value!r}")
    return int(value, 10)


def _hex(value: str) -> int:
    if not HEX_RE.fullmatch(value):
        raise ValueError(f"not a 0x-prefixed integer: {value!r}")
    return int(value, 16)


def _int(value: str) -> int:
    if not re.fullmatch(r"-?[0-9]+", value):
        raise ValueError(f"not a signed decimal integer: {value!r}")
    return int(value, 10)


def _finite(value: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"non-finite number: {value!r}")
    return number


def _value(frame_type: str, field: str, value: str) -> Any:
    if (frame_type, field) in STRING_FIELDS:
        if not value or "," in value or "\r" in value or "\n" in value:
            raise ValueError(f"invalid text field: {value!r}")
        return value
    if (frame_type, field) in INT_FIELDS:
        return _int(value)
    if field in HEX_FIELDS:
        return _hex(value)
    if frame_type == "TESTSUM" and field in UINT_FIELDS:
        return _uint(value)
    if field in BOOL_FIELDS:
        parsed = _uint(value)
        if parsed not in (0, 1):
            raise ValueError(f"boolean is not 0/1: {value!r}")
        return parsed
    if frame_type == "LOGSUM" or field in UINT_FIELDS:
        return _uint(value)
    return _finite(value)


def _parse_pairs(row: list[str], start: int) -> dict[str, str]:
    if (len(row) - start) % 2:
        raise ValueError("header key/value list has odd length")
    result: dict[str, str] = {}
    for index in range(start, len(row), 2):
        key, value = row[index], row[index + 1]
        if not key or key in result:
            raise ValueError(f"duplicate/empty header key: {key!r}")
        result[key] = value
    return result


def _parse_logstat(name: str, values: list[str]) -> list[int | float]:
    name = name.upper()
    if name == "BIAS":
        if len(values) != 9:
            return [_uint(value) for value in values]
        return [*(_uint(value) for value in values[:5]),
                *(_finite(value) for value in values[5:8]),
                _uint(values[8])]
    if name == "MAG":
        if len(values) != 5:
            return [_uint(value) for value in values]
        if not PLAIN_HEX_RE.fullmatch(values[3]) or not PLAIN_HEX_RE.fullmatch(values[4]):
            raise ValueError("MAG flags are not plain hexadecimal integers")
        return [*(_uint(value) for value in values[:3]),
                int(values[3], 16), int(values[4], 16)]
    if name in {"AHRS", "QUALITY", "BACKPRESSURE", "SESSION", "PIPELINE", "DROPS"}:
        return [_uint(value) for value in values]
    return [_finite(value) for value in values]


def validate_logver3(
    path: Path,
    *,
    capture_kind: str = "static",
    min_duration_s: float = 0.0,
    min_rate_hz: float = 19.0,
    max_rate_hz: float = 21.0,
    max_record_age_us: int = 250_000,
    expected_profile: str = "ProductionDiag",
    expected_environment: str = "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG",
    require_console_status: bool = True,
) -> dict[str, Any]:
    if capture_kind not in {"static", "runtime"}:
        raise ValueError("capture_kind must be 'static' or 'runtime'")
    failures: list[str] = []
    health_failures: list[str] = []
    header_rows: list[list[str]] = []
    header_line_numbers: list[int] = []
    declared: dict[str, tuple[str, ...]] = {}
    declaration_order: list[str] = []
    declaration_lines: list[int] = []
    first_data_line: int | None = None
    frames: list[tuple[str, dict[str, Any], int]] = []
    summaries: list[dict[str, Any]] = []
    test_summaries: list[dict[str, Any]] = []
    logstats: dict[str, list[int | float]] = {}
    key_values: dict[str, str] = {}
    key_value_counts: Counter[str] = Counter()
    test_done = False

    try:
        raw_lines = path.read_text(encoding="utf-8", errors="strict").splitlines()
    except (OSError, UnicodeError) as exc:
        return {"passed": False, "failures": [f"cannot read UTF-8 log: {exc}"]}

    for line_number, raw in enumerate(raw_lines, start=1):
        line = raw.rstrip("\r")
        if "\x00" in line:
            failures.append(f"line {line_number}: NUL byte is forbidden")
            continue
        if line == ("STATIC TEST DONE" if capture_kind == "static" else "RUNTIME TEST DONE"):
            test_done = True
        if line.startswith("# ERR"):
            failures.append(f"line {line_number}: firmware/command error present: {line}")
        if "# WARN console dropped" in line:
            failures.append(f"line {line_number}: console drop warning present")
        if "=" in line and "," not in line and not line.startswith("#"):
            key, value = line.split("=", 1)
            if key and " " not in key:
                key_values[key] = value
                key_value_counts[key] += 1
        prefix = line.split(",", 1)[0]
        if prefix not in KNOWN_PREFIXES:
            if "," in line and MACHINE_PREFIX_RE.fullmatch(prefix):
                failures.append(f"line {line_number}: unknown machine frame {prefix!r}")
            continue
        if '"' in line:
            failures.append(f"line {line_number}: quoted CSV fields are forbidden")
            continue
        try:
            row = next(csv.reader([line], strict=True))
        except (csv.Error, StopIteration) as exc:
            failures.append(f"line {line_number}: malformed CSV: {exc}")
            continue
        if not row or row[0] != prefix:
            failures.append(f"line {line_number}: malformed machine prefix")
            continue

        if prefix == "LOGVER":
            header_rows.append(row)
            header_line_numbers.append(line_number)
            continue
        if prefix == "LOGFMT":
            if len(row) < 3:
                failures.append(f"line {line_number}: short LOGFMT")
                continue
            frame_type = row[1]
            if frame_type not in SCHEMAS:
                failures.append(f"line {line_number}: unknown LOGFMT type {frame_type!r}")
                continue
            columns = tuple(row[2:])
            if frame_type in declared:
                failures.append(f"line {line_number}: duplicate LOGFMT for {frame_type}")
            declared[frame_type] = columns
            declaration_order.append(frame_type)
            declaration_lines.append(line_number)
            if columns != SCHEMAS[frame_type]:
                failures.append(f"line {line_number}: {frame_type} schema mismatch")
            continue
        if first_data_line is None:
            first_data_line = line_number
        if prefix == "LOGSTAT":
            if len(row) < 3:
                failures.append(f"line {line_number}: short LOGSTAT")
                continue
            stat_name = row[1].upper()
            if stat_name not in {
                "AHRS", "QUALITY", "MAG", "BIAS", "BACKPRESSURE", "SESSION",
                "PIPELINE", "DROPS",
            }:
                failures.append(f"line {line_number}: unknown LOGSTAT name {row[1]!r}")
                continue
            try:
                logstats[stat_name] = _parse_logstat(stat_name, row[2:])
            except (ValueError, OverflowError) as exc:
                failures.append(f"line {line_number}: LOGSTAT {exc}")
            continue
        if prefix == "TEMPBIN":
            if len(row) != 15:
                failures.append(f"line {line_number}: TEMPBIN field count {len(row)} != 15")
            else:
                try:
                    _uint(row[1])
                    _uint(row[5])
                    _uint(row[14])
                except ValueError as exc:
                    failures.append(f"line {line_number}: TEMPBIN {exc}")
                for index, value in enumerate(row[1:], start=1):
                    if index in {1, 5, 14}:
                        continue
                    try:
                        _finite(value)
                    except ValueError as exc:
                        failures.append(f"line {line_number}: TEMPBIN {exc}")
            continue

        schema = SCHEMAS[prefix]
        if len(row) != len(schema) + 1:
            failures.append(
                f"line {line_number}: {prefix} field count {len(row) - 1} != {len(schema)}"
            )
            continue
        parsed: dict[str, Any] = {}
        for field, raw_value in zip(schema, row[1:]):
            try:
                parsed[field] = _value(prefix, field, raw_value)
            except (ValueError, OverflowError) as exc:
                failures.append(f"line {line_number}: {prefix}.{field}: {exc}")
        if len(parsed) != len(schema):
            continue
        if prefix == "LOGSUM":
            summaries.append(parsed)
        elif prefix == "TESTSUM":
            test_summaries.append(parsed)
        else:
            frames.append((prefix, parsed, line_number))

    if len(header_rows) != 1:
        failures.append(f"expected exactly one LOGVER row, got {len(header_rows)}")
        header: dict[str, str] = {}
    else:
        row = header_rows[0]
        header = {}
        if len(row) < 3 or row[1:3] != ["3", "E1"]:
            failures.append("LOGVER must be exactly version 3 / schema E1")
        try:
            header = _parse_pairs(row, 3)
        except ValueError as exc:
            failures.append(f"LOGVER: {exc}")

    if len(header_line_numbers) == 1:
        header_line = header_line_numbers[0]
        if declaration_lines and min(declaration_lines) <= header_line:
            failures.append("LOGFMT declarations must follow LOGVER")
        if first_data_line is not None and first_data_line <= header_line:
            failures.append("machine data must follow LOGVER")
        if first_data_line is not None and declaration_lines and max(declaration_lines) >= first_data_line:
            failures.append("all LOGFMT declarations must precede machine data")
    if declaration_order != list(SCHEMAS):
        failures.append(
            "LOGFMT declaration order mismatch: got " + ",".join(declaration_order)
        )

    required_header = {"mode", "rate_hz", "config_crc", "config_version", "build_profile", "pio_env", "git"}
    if set(header) != required_header:
        failures.append(f"LOGVER keys mismatch: got {sorted(header)}, expected {sorted(required_header)}")
    if header.get("mode") != "full":
        failures.append(f"capture mode must be full, got {header.get('mode')!r}")
    if header.get("rate_hz") != "20":
        failures.append(f"requested rate must be 20 Hz, got {header.get('rate_hz')!r}")
    if expected_profile and header.get("build_profile") != expected_profile:
        failures.append(f"build_profile must be {expected_profile}, got {header.get('build_profile')!r}")
    if expected_environment and header.get("pio_env") != expected_environment:
        failures.append(f"pio_env must be {expected_environment}, got {header.get('pio_env')!r}")
    if not GIT_RE.fullmatch(header.get("git", "")):
        failures.append("git identity must be a clean 40-hex commit")
    if not HEX_RE.fullmatch(header.get("config_crc", "")):
        failures.append("config_crc must be 0x-prefixed hexadecimal")
    try:
        _uint(header.get("config_version", ""))
    except ValueError:
        failures.append("config_version must be an unsigned integer")

    missing_formats = sorted(set(SCHEMAS) - set(declared))
    if missing_formats:
        failures.append("missing LOGFMT declarations: " + ", ".join(missing_formats))

    groups: dict[int, list[tuple[str, dict[str, Any], int]]] = {}
    sequence_order: list[int] = []
    active_sequence: int | None = None
    for entry in frames:
        sequence = int(entry[1]["seq"])
        if sequence != active_sequence:
            if sequence in groups:
                failures.append(f"line {entry[2]}: sequence {sequence} is non-contiguous/duplicated")
            else:
                sequence_order.append(sequence)
            active_sequence = sequence
        groups.setdefault(sequence, []).append(entry)

    if not sequence_order:
        failures.append("no LOGVER3 frame rows")
    else:
        if sequence_order[0] != 0:
            failures.append(f"first sequence must be 0, got {sequence_order[0]}")
        for previous, current in zip(sequence_order, sequence_order[1:]):
            if current != ((previous + 1) & 0xFFFFFFFF):
                failures.append(f"sequence gap/rewind: {previous} -> {current}")

    expected_groups = {
        "Q": ["Q", "FIFO", "CAL"],
        "MAG": ["MAG", "MAGR", "YAW"],
        "STATE": ["STATE"],
        "BIASUPD": ["BIASUPD"],
        "NET": ["NET"],
    }
    for sequence, entries in groups.items():
        types = [entry[0] for entry in entries]
        counts = Counter(types)
        duplicates = sorted(name for name, count in counts.items() if count != 1)
        if duplicates:
            failures.append(f"sequence {sequence}: duplicate frame types {duplicates}")
        leader = types[0]
        expected = expected_groups.get(leader)
        if leader == "Q" and "BIAS" in types:
            expected = ["Q", "FIFO", "BIAS", "CAL"]
        if expected is None or types != expected:
            failures.append(f"sequence {sequence}: invalid bundle order/types {types}")
            continue
        timestamps = {entry[0]: int(entry[1]["t_us"]) for entry in entries}
        if leader == "Q" and any(value != timestamps["Q"] for value in timestamps.values()):
            failures.append(f"sequence {sequence}: Q bundle timestamps disagree")
        if leader == "MAG" and any(value != timestamps["MAG"] for value in timestamps.values()):
            failures.append(f"sequence {sequence}: MAG bundle timestamps disagree")

    previous_group_timestamp: int | None = None
    for sequence in sequence_order:
        entries = groups[sequence]
        group_timestamp = int(entries[0][1]["t_us"])
        if previous_group_timestamp is not None and group_timestamp < previous_group_timestamp:
            failures.append(
                f"sequence {sequence}: global timestamp rewind "
                f"{previous_group_timestamp} -> {group_timestamp}"
            )
        previous_group_timestamp = group_timestamp

    counts = Counter(frame_type for frame_type, _, _ in frames)
    required_frames = ["Q", "FIFO", "CAL", "BIAS", "NET"]
    if capture_kind == "static":
        required_frames.extend(("MAG", "MAGR", "YAW"))
    for required in required_frames:
        if counts[required] == 0:
            failures.append(f"required full-capture frame {required} is absent")

    for frame_type in ("Q", "MAG", "BIASUPD", "NET"):
        timestamps = [int(row["t_us"]) for name, row, _ in frames if name == frame_type]
        if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
            failures.append(f"{frame_type} timestamps are not strictly increasing")

    def record_health(message: str) -> None:
        health_failures.append(message)
        if capture_kind == "static":
            failures.append(message)

    def delta_u32(current: int, start: int) -> int:
        return (current - start) & 0xFFFFFFFF

    q_rows = [row for name, row, _ in frames if name == "Q"]
    fifo_rows = [row for name, row, _ in frames if name == "FIFO"]
    bias_rows = [row for name, row, _ in frames if name == "BIAS"]
    bias_update_rows = [row for name, row, _ in frames if name == "BIASUPD"]
    mag_rows = [row for name, row, _ in frames if name == "MAG"]
    yaw_rows = [row for name, row, _ in frames if name == "YAW"]
    net_rows = [row for name, row, _ in frames if name == "NET"]
    if len(q_rows) >= 2:
        duration_s = (q_rows[-1]["t_us"] - q_rows[0]["t_us"]) / 1_000_000.0
        rate_hz = (len(q_rows) - 1) / duration_s if duration_s > 0 else 0.0
    else:
        duration_s = 0.0
        rate_hz = 0.0
    if duration_s < min_duration_s:
        record_health(f"duration {duration_s:.3f}s < {min_duration_s:.3f}s")
    if not (min_rate_hz <= rate_hz <= max_rate_hz):
        record_health(
            f"Q rate {rate_hz:.3f} Hz outside {min_rate_hz:.3f}..{max_rate_hz:.3f}"
        )
    if any(row["recovery"] != 0 for row in q_rows):
        record_health("Q recovery rows are present")
    if any(row["hw_ts"] != 1 or row["fb_ts"] != 0 or row["dropped_before"] != 0 or
           row["overrun"] != 0 or row["full"] != 0 or row["unknown"] != 0 for row in fifo_rows):
        record_health("FIFO timestamp/drop/fault contract failed")
    for row in q_rows:
        norm = math.sqrt(sum(float(row[name]) ** 2 for name in ("w", "x", "y", "z")))
        if not 0.98 <= norm <= 1.02:
            failures.append(f"quaternion norm out of range: {norm:.6f}")
            break

    # E1 records must make sensor configuration and UDP health observable, not
    # merely prove that columns were present.
    known_bias_sources = {"none", "bias", "temp", "rt", "bias+rt", "temp+rt"}
    for row in bias_rows:
        source = str(row["source"])
        flags = int(row["flags"])
        if source not in known_bias_sources:
            failures.append(f"unknown BIAS source {source!r}")
            continue
        if int(row["rt_enabled"]) != ((flags >> 4) & 1):
            failures.append("BIAS rt_enabled disagrees with flags bit 4")
        if source in {"none", "rt"}:
            record_health(f"BIAS source {source!r} has no calibrated base model")
        if source in {"temp", "temp+rt"}:
            if (flags & 0x7) != 0x7:
                record_health("temperature BIAS source lacks valid/enabled/range flags")
            if flags & (1 << 3):
                record_health("temperature BIAS sample is outside calibrated range")
    if bias_rows:
        updates_delta = delta_u32(int(bias_rows[-1]["rt_updates"]), int(bias_rows[0]["rt_updates"]))
        if updates_delta > 0 and not bias_update_rows:
            failures.append("BIAS updates advanced but no BIASUPD evidence was captured")

    if any(row["heading_valid"] != 1 or row["trusted"] != 1 or
           row["field_trusted"] != 1 or row["reject_flags"] != 0 or
           row["field_flags"] != 0 for row in mag_rows):
        record_health("MAG semantic trust/heading contract failed")
    if any(row["valid"] != 1 or row["gate_open"] != 1 or
           row["apply_allowed"] != 1 for row in yaw_rows):
        record_health("YAW semantic gate/apply contract failed")

    network_report: dict[str, Any] = {"rows": len(net_rows)}
    if len(net_rows) < 2:
        failures.append(f"NET requires at least two rows, got {len(net_rows)}")
    else:
        first_net, last_net = net_rows[0], net_rows[-1]
        net_duration_s = (int(last_net["t_us"]) - int(first_net["t_us"])) / 1_000_000.0
        net_delta_fields = (
            "wifi_disc", "wifi_timeouts", "rot_missed", "rot_late", "send_failures",
            "rot_fail", "control_fail", "telemetry_fail", "discovery_fail",
            "tx_pressure_fail", "tx_other_fail", "rebind_ok", "rebind_fail", "full_reopens",
        )
        network_deltas = {
            field: delta_u32(int(last_net[field]), int(first_net[field]))
            for field in net_delta_fields
        }
        rotation_delta = delta_u32(int(last_net["rotation_sent"]), int(first_net["rotation_sent"]))
        rotation_rate_hz = rotation_delta / net_duration_s if net_duration_s > 0 else 0.0
        network_report.update({
            "duration_s": round(net_duration_s, 6),
            "rotation_delta": rotation_delta,
            "rotation_rate_hz": round(rotation_rate_hz, 6),
            "deltas": network_deltas,
            "last_consecutive_send_failures": int(last_net["consecutive_fail"]),
            "last_udp_error": int(last_net["last_udp_error"]),
        })
        if any(row["wifi_connected"] != 1 or row["udp_ready"] != 1 or
               row["server_found"] != 1 for row in net_rows):
            record_health("NET reports Wi-Fi/UDP/server unavailable during capture")
        fault_fields = tuple(field for field in net_delta_fields if field != "rebind_ok")
        if any(network_deltas[field] != 0 for field in fault_fields):
            record_health("NET counters show Wi-Fi/UDP/deadline failures during capture")
        last_consecutive_failures = int(last_net["consecutive_fail"])
        last_udp_error = int(last_net["last_udp_error"])
        if last_consecutive_failures != 0:
            record_health("NET ends with consecutive UDP failures")
            if last_udp_error == 0:
                failures.append("NET consecutive failures have no UDP errno evidence")
        # last_udp_error is intentionally historical in SlimeVROutputRuntime:
        # successful sends clear consecutiveSendFailures but retain the last
        # errno for diagnostics. It must not reject a later clean window by
        # itself. The counters above detect an error during this window, while
        # the current pressure state detects an unfinished recovery.
        if any(int(row["tx_pressure_state"]) != 0 for row in net_rows):
            record_health("NET entered or remains in a TX recovery state")
        if not 50.0 <= rotation_rate_hz <= 130.0:
            record_health(f"NET rotation rate {rotation_rate_hz:.3f} Hz is implausible")

    if len(test_summaries) != 1:
        failures.append(f"expected exactly one TESTSUM row, got {len(test_summaries)}")
        test_summary: dict[str, Any] = {}
    else:
        test_summary = test_summaries[0]
        if test_summary.get("kind") != capture_kind:
            failures.append(
                f"TESTSUM kind {test_summary.get('kind')!r} does not match {capture_kind!r}"
            )
        if test_summary.get("duration_ms", 0) == 0 or test_summary.get("samples", 0) == 0:
            failures.append("TESTSUM has an empty measured window")
        if test_summary.get("stopped") != 0:
            record_health("TESTSUM says the test was stopped by command")
        if test_summary.get("net_valid") != 1:
            failures.append("TESTSUM network metrics are not valid")
        test_fault_fields = (
            "wifi_disc", "wifi_timeouts", "udp_fail", "rot_fail", "rot_missed", "rot_late",
            "tx_pressure_fail", "tx_other_fail", "rebind_fail", "full_reopens",
        )
        if any(test_summary.get(field, 0) != 0 for field in test_fault_fields):
            record_health("TESTSUM reports exact network failures in the measured window")
        if capture_kind == "static":
            if test_summary.get("static_valid") != 1 or test_summary.get("mag_valid") != 1:
                failures.append("static TESTSUM metrics are not valid")
            static_fault_fields = (
                "fb_ts", "bad_ts", "dropped", "recoveries", "fifo_overruns",
                "fifo_full", "fifo_unknown", "mag_rejected",
            )
            if any(test_summary.get(field, 0) != 0 for field in static_fault_fields):
                record_health("static TESTSUM reports FIFO/timestamp/MAG faults")
            if test_summary.get("hw_ts") != test_summary.get("samples"):
                record_health("static TESTSUM hardware timestamp count differs from samples")
            if test_summary.get("mag_trusted", 0) == 0:
                record_health("static TESTSUM contains no trusted magnetometer samples")
            if not 0.8 <= float(test_summary.get("accel_mean_g", 0.0)) <= 1.2:
                record_health("static TESTSUM acceleration norm mean is implausible")
        else:
            if test_summary.get("static_valid") != 0 or test_summary.get("mag_valid") != 0:
                failures.append("runtime TESTSUM incorrectly claims static/MAG metrics")

    if not summaries:
        failures.append("LOGSUM is absent")
        final_summary: dict[str, Any] = {}
    else:
        final_summary = summaries[-1]
        for summary_name, frame_name in (("q", "Q"), ("cal", "CAL"), ("fifo", "FIFO"),
                                         ("mag", "MAG"), ("yaw", "YAW"), ("state", "STATE"),
                                         ("bias", "BIAS"), ("net", "NET")):
            if final_summary[summary_name] != counts[frame_name]:
                failures.append(
                    f"LOGSUM {summary_name}={final_summary[summary_name]} but parsed {counts[frame_name]}"
                )
        if final_summary.get("mode") != "full" or final_summary.get("rate_hz") != 20:
            failures.append("LOGSUM mode/rate does not match full 20 Hz capture")
        for field in ("fifo_overruns", "fifo_full", "large_gaps", "recoveries"):
            if final_summary.get(field) != 0:
                record_health(f"LOGSUM {field} must be zero")

    expected_logstat_lengths = {
        "AHRS": 6, "QUALITY": 6, "MAG": 5, "BIAS": 9, "BACKPRESSURE": 1,
        "SESSION": 1, "PIPELINE": 6, "DROPS": 4,
    }
    for name, length in expected_logstat_lengths.items():
        values = logstats.get(name)
        if values is None:
            failures.append(f"LOGSTAT,{name} is absent")
        elif len(values) != length:
            failures.append(f"LOGSTAT,{name} has {len(values)} values, expected {length}")
    if logstats.get("BACKPRESSURE", [1])[0] != 0:
        failures.append("machine-log backpressure drops are non-zero")
    if logstats.get("SESSION", [1])[0] != 0:
        failures.append("capture disconnect aborts are non-zero")
    drops = logstats.get("DROPS", [1, 0, 1, 1])
    if len(drops) == 4 and (drops[0] != 0 or drops[2] != 0 or drops[3] != 0):
        failures.append("producer/shutdown/disconnect drops are non-zero")
    pipeline = logstats.get("PIPELINE", [1, 0, 0, 1, 0, max_record_age_us + 1])
    if len(pipeline) == 6:
        queued, high_water, enqueued, serialized, _, max_age = pipeline
        if queued != 0 or enqueued != serialized:
            failures.append("deferred pipeline did not drain completely")
        if high_water > 12:
            failures.append(f"pipeline high-water {high_water} exceeds capacity 12")
        if max_age > max_record_age_us:
            failures.append(f"pipeline max record age {max_age}us exceeds {max_record_age_us}us")

    if not test_done:
        failures.append(f"{capture_kind.upper()} TEST DONE marker is absent")
    if require_console_status:
        for key in (
            "remote_console_bytes_dropped",
            "remote_console_capture_aborts",
            "remote_console_lease_expirations",
            "remote_console_output_bytes_dropped",
            "remote_console_output_records_dropped",
            "remote_console_output_oversized_records_dropped",
            "remote_console_output_pending_drop_notice_records",
        ):
            if key_value_counts[key] != 1:
                failures.append(
                    f"console status key {key} occurs {key_value_counts[key]} times, expected 1"
                )
                continue
            try:
                value = _uint(key_values[key])
            except (KeyError, ValueError):
                failures.append(f"console status key {key} is absent/invalid")
                continue
            if value != 0:
                failures.append(f"{key}={value}, expected 0")

    return {
        "passed": not failures,
        "failures": failures,
        "health_passed": not health_failures,
        "health_failures": health_failures,
        "contract": f"LOGVER3-E1-{capture_kind}-v1",
        "identity": header,
        "duration_s": round(duration_s, 6),
        "q_rate_hz": round(rate_hz, 6),
        "counts": dict(sorted(counts.items())),
        "pipeline": {
            "queued": pipeline[0] if len(pipeline) == 6 else None,
            "high_water": pipeline[1] if len(pipeline) == 6 else None,
            "enqueued": pipeline[2] if len(pipeline) == 6 else None,
            "serialized": pipeline[3] if len(pipeline) == 6 else None,
            "service_calls": pipeline[4] if len(pipeline) == 6 else None,
            "max_record_age_us": pipeline[5] if len(pipeline) == 6 else None,
        },
        "drops": {
            "backpressure": logstats.get("BACKPRESSURE", [None])[0],
            "producer": drops[0] if len(drops) == 4 else None,
            "service_deferrals": drops[1] if len(drops) == 4 else None,
            "shutdown": drops[2] if len(drops) == 4 else None,
            "disconnect": drops[3] if len(drops) == 4 else None,
        },
        "final_summary": final_summary,
        "test_summary": test_summary,
        "network": network_report,
        "bias": {
            "sources": sorted({str(row["source"]) for row in bias_rows}),
            "updates": len(bias_update_rows),
        },
        "mag": {
            "rows": len(mag_rows),
            "trusted_rows": sum(int(row["trusted"]) for row in mag_rows),
            "rejected_rows": sum(1 for row in mag_rows if int(row["reject_flags"]) != 0),
        },
        "yaw": {
            "rows": len(yaw_rows),
            "gate_open_rows": sum(int(row["gate_open"]) for row in yaw_rows),
            "apply_allowed_rows": sum(int(row["apply_allowed"]) for row in yaw_rows),
            "applied_rows": sum(int(row["applied"]) for row in yaw_rows),
        },
    }
