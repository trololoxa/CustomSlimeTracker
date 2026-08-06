#!/usr/bin/env python3
"""Regression tests for the fail-closed LOGVER3 capture contract."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "replay"))
from logver3_contract import SCHEMAS, validate_logver3  # noqa: E402


def valid_log() -> str:
    lines = [
        "LOGVER,3,E1,mode,full,rate_hz,20,config_crc,0x12345678,config_version,2,"
        "build_profile,ProductionDiag,pio_env,BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG,"
        "git,0123456789abcdef0123456789abcdef01234567",
    ]
    lines.extend("LOGFMT," + name + "," + ",".join(columns) for name, columns in SCHEMAS.items())
    lines.extend(
        (
            "Q,1000000,0,50000,1,0,0,0,0x1,1.0,TRACKING_6DOF,1.0,1.0,0.0,1.0,0.1,0",
            "FIFO,1000000,0,50000,1,0,0,0,0,0,0x1",
            "BIAS,1000000,0,25.0,0.0,0.0,0.0,temp,1.0,0x7,0,0",
            "CAL,1000000,0,0,0,1,0,0,0,25.0,0x1",
            "MAG,1000000,1,1,1,500,500,250,1,10,0,1,0x0,45,2,1,0x0,0,0,0",
            "MAGR,1000000,1,1,1,2,3,1,2,3,1,2,3,3.74,3.74,3.74,0x0,0x0,1",
            "YAW,1000000,1,1,1,1,0,0,0,1,0x0,0,0,0,0,1000,0",
            "NET,1000000,2,1,0,0,-55,4,1,1,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0",
            "Q,1050000,3,50000,1,0,0,0,0x1,1.0,TRACKING_6DOF,1.0,1.0,0.0,1.0,0.1,0",
            "FIFO,1050000,3,50000,1,0,0,0,0,0,0x1",
            "CAL,1050000,3,0,0,1,0,0,0,25.0,0x1",
            "MAG,1050000,4,2,2,500,500,250,1,10,0,1,0x0,45,2,1,0x0,0,0,0",
            "MAGR,1050000,4,2,4,5,6,4,5,6,4,5,6,8.77,8.77,8.77,0x0,0x0,1",
            "YAW,1050000,4,1,1,1,0,0,0,1,0x0,0,0,0,0,1050,0",
            "NET,1050000,5,1,0,0,-55,4,1,1,105,105,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0",
            "TESTSUM,static,60000,0,1000,1000,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0.01,0.02,1.0,0.001,25.0,25.1,1,100,0",
            "LOGSUM,60000,full,20,2,2,2,2,2,0,1,2,2,2,0,0,0,0,1,0,0",
            "LOGSTAT,AHRS,2,2,2,0,0,0",
            "LOGSTAT,QUALITY,2,2,0,0,0,0",
            "LOGSTAT,MAG,1,1,0,0,0",
            "LOGSTAT,BIAS,1,1,1,0,0,0.00000000,0.00000000,0.00000000,0",
            "LOGSTAT,BACKPRESSURE,0",
            "LOGSTAT,SESSION,0",
            "LOGSTAT,PIPELINE,0,2,3,3,10,1000",
            "LOGSTAT,DROPS,0,0,0,0",
            "STATIC TEST DONE",
            "remote_console_bytes_dropped=0",
            "remote_console_capture_aborts=0",
            "remote_console_lease_expirations=0",
            "remote_console_output_bytes_dropped=0",
            "remote_console_output_records_dropped=0",
            "remote_console_output_oversized_records_dropped=0",
            "remote_console_output_pending_drop_notice_records=0",
        )
    )
    return "\n".join(lines) + "\n"


def set_record_field(
    text: str,
    record: str,
    field: str,
    value: str,
    *,
    occurrence: int | None = None,
) -> str:
    lines = text.splitlines()
    matches = [index for index, line in enumerate(lines) if line.startswith(record + ",")]
    selected = matches if occurrence is None else [matches[occurrence]]
    column = 1 + SCHEMAS[record].index(field)
    for index in selected:
        values = lines[index].split(",")
        values[column] = value
        lines[index] = ",".join(values)
    return "\n".join(lines) + "\n"


class Logver3ContractTests(unittest.TestCase):
    def validate(self, text: str) -> dict[str, object]:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.log"
            path.write_text(text, encoding="utf-8")
            return validate_logver3(path, min_duration_s=0.0)

    def test_valid_contract_passes(self) -> None:
        result = self.validate(valid_log())
        self.assertTrue(result["passed"], result["failures"])
        self.assertEqual(result["q_rate_hz"], 20.0)

    def test_wrong_version_fails(self) -> None:
        result = self.validate(valid_log().replace("LOGVER,3,E1", "LOGVER,2,E1", 1))
        self.assertFalse(result["passed"])
        self.assertTrue(any("version 3" in item for item in result["failures"]))

    def test_nonfinite_numeric_fails(self) -> None:
        result = self.validate(valid_log().replace("50000,1,0,0,0,0x1", "50000,nan,0,0,0,0x1", 1))
        self.assertFalse(result["passed"])
        self.assertTrue(any("non-finite" in item for item in result["failures"]))

    def test_bundle_loss_fails(self) -> None:
        text = valid_log().replace("CAL,1050000,3,0,0,1,0,0,0,25.0,0x1\n", "")
        result = self.validate(text)
        self.assertFalse(result["passed"])
        self.assertTrue(any("invalid bundle" in item for item in result["failures"]))

    def test_drop_counter_fails(self) -> None:
        result = self.validate(valid_log().replace("LOGSTAT,DROPS,0,0,0,0", "LOGSTAT,DROPS,1,0,0,0"))
        self.assertFalse(result["passed"])
        self.assertTrue(any("drops are non-zero" in item for item in result["failures"]))

    def test_remote_capture_abort_fails(self) -> None:
        result = self.validate(valid_log().replace(
            "remote_console_capture_aborts=0", "remote_console_capture_aborts=1"
        ))
        self.assertFalse(result["passed"])
        self.assertTrue(any("remote_console_capture_aborts=1" in item for item in result["failures"]))

    def test_unknown_machine_frame_fails(self) -> None:
        result = self.validate(valid_log() + "FUTURE,1,2,3\n")
        self.assertFalse(result["passed"])
        self.assertTrue(any("unknown machine frame" in item for item in result["failures"]))

    def test_quoted_csv_field_fails(self) -> None:
        result = self.validate(valid_log().replace(
            ",TRACKING_6DOF,", ',"TRACKING_6DOF",', 1
        ))
        self.assertFalse(result["passed"])
        self.assertTrue(any("quoted CSV" in item for item in result["failures"]))

    def test_firmware_error_line_fails(self) -> None:
        result = self.validate(valid_log() + "# ERR unexpected capture failure\n")
        self.assertFalse(result["passed"])
        self.assertTrue(any("firmware/command error" in item for item in result["failures"]))

    def test_logfmt_declaration_reordering_fails(self) -> None:
        first = "LOGFMT,Q," + ",".join(SCHEMAS["Q"])
        second = "LOGFMT,FIFO," + ",".join(SCHEMAS["FIFO"])
        result = self.validate(valid_log().replace(first + "\n" + second, second + "\n" + first))
        self.assertFalse(result["passed"])
        self.assertTrue(any("declaration order" in item for item in result["failures"]))

    def test_global_timestamp_rewind_fails(self) -> None:
        result = self.validate(valid_log().replace(
            "MAG,1000000,1,", "MAG,999999,1,", 1
        ).replace(
            "MAGR,1000000,1,", "MAGR,999999,1,", 1
        ).replace(
            "YAW,1000000,1,", "YAW,999999,1,", 1
        ))
        self.assertFalse(result["passed"])
        self.assertTrue(any("global timestamp rewind" in item for item in result["failures"]))

    def test_duplicate_console_counter_fails(self) -> None:
        result = self.validate(valid_log() + "remote_console_bytes_dropped=0\n")
        self.assertFalse(result["passed"])
        self.assertTrue(any("occurs 2 times" in item for item in result["failures"]))

    def test_static_udp_failure_fails(self) -> None:
        result = self.validate(valid_log().replace(
            "TESTSUM,static,60000,0,1000,1000,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,",
            "TESTSUM,static,60000,0,1000,1000,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,",
            1,
        ))
        self.assertFalse(result["passed"])
        self.assertTrue(any("network failures" in item for item in result["failures"]))

    def test_untrusted_mag_fails(self) -> None:
        result = self.validate(valid_log().replace(
            "MAG,1000000,1,1,1,500,500,250,1,10,0,1,0x0,45,2,1,0x0,0,0,0",
            "MAG,1000000,1,1,1,500,500,250,1,10,0,0,0x1,45,2,0,0x1,0,0,0",
        ))
        self.assertFalse(result["passed"])
        self.assertTrue(any("MAG semantic" in item for item in result["failures"]))

    def test_yaw_gate_loss_after_ready_row_fails(self) -> None:
        result = self.validate(valid_log().replace(
            "YAW,1050000,4,1,1,1,0,0,0,1,0x0,0,0,0,0,1050,0",
            "YAW,1050000,4,1,0,0,0,0,0,1,0x0,0,0,0,0,1050,0",
        ))
        self.assertFalse(result["passed"])
        self.assertTrue(any("YAW semantic" in item for item in result["failures"]))

    def test_stale_udp_errno_after_recovery_does_not_fail_clean_window(self) -> None:
        text = set_record_field(valid_log(), "NET", "last_udp_error", "12")
        result = self.validate(text)
        self.assertTrue(result["passed"], result["failures"])
        self.assertTrue(result["health_passed"], result["health_failures"])

    def test_active_tx_recovery_state_fails_static_window(self) -> None:
        text = set_record_field(
            valid_log(), "NET", "tx_pressure_state", "1", occurrence=0
        )
        result = self.validate(text)
        self.assertFalse(result["passed"])
        self.assertTrue(any("TX recovery state" in item for item in result["failures"]))

    def test_missing_testsum_fails(self) -> None:
        line = next(item for item in valid_log().splitlines() if item.startswith("TESTSUM,"))
        result = self.validate(valid_log().replace(line + "\n", ""))
        self.assertFalse(result["passed"])
        self.assertTrue(any("TESTSUM" in item for item in result["failures"]))

    def test_lease_expiration_fails(self) -> None:
        result = self.validate(valid_log().replace(
            "remote_console_lease_expirations=0", "remote_console_lease_expirations=1"
        ))
        self.assertFalse(result["passed"])
        self.assertTrue(any("lease_expirations=1" in item for item in result["failures"]))

    def test_runtime_capture_preserves_udp_health_failure(self) -> None:
        text = valid_log().replace("STATIC TEST DONE", "RUNTIME TEST DONE")
        text = text.replace(
            "TESTSUM,static,60000,0,1000,1000,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0.01,0.02,1.0,0.001,25.0,25.1,1,100,0",
            "TESTSUM,runtime,60000,0,1000,1000,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,25.0,25.1,0,0,0",
        )
        result = self.validate_kind(text, "runtime")
        self.assertTrue(result["passed"], result["failures"])
        self.assertFalse(result["health_passed"])

    def validate_kind(self, text: str, kind: str) -> dict[str, object]:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.log"
            path.write_text(text, encoding="utf-8")
            return validate_logver3(path, min_duration_s=0.0, capture_kind=kind)

    def test_legacy_fixture_is_not_a_logver3_candidate(self) -> None:
        result = validate_logver3(
            ROOT / "tests" / "fixtures" / "e0_static_smoke.log",
            min_duration_s=0.0,
            expected_profile="",
            expected_environment="",
            require_console_status=False,
        )
        self.assertFalse(result["passed"])


if __name__ == "__main__":
    unittest.main()
