#!/usr/bin/env python3
"""Regression tests for fail-closed cable-free capture promotion."""

from __future__ import annotations

import contextlib
import io
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from capture_telnet_log import (
    CaptureError,
    CaptureSocket,
    CaptureValidationError,
    TELNET_IAC_NOP,
    main,
    promote_validated_capture,
    promote_validated_capture_bundle,
)


class CapturePromotionTests(unittest.TestCase):
    def test_keepalive_is_telnet_nop_not_an_ascii_command(self) -> None:
        transport = CaptureSocket.__new__(CaptureSocket)
        transport.socket = mock.Mock()
        transport.last_keepalive = 0.0
        with mock.patch("capture_telnet_log.time.monotonic", return_value=10.0):
            transport._maybe_keepalive(force=True)
        transport.socket.sendall.assert_called_once_with(TELNET_IAC_NOP)

    def test_valid_candidate_atomically_replaces_destination(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "capture.log"
            output.write_bytes(b"known-good")

            def validator(candidate: Path) -> dict[str, object]:
                self.assertEqual(candidate.parent, root)
                self.assertIn(".candidate-", candidate.name)
                self.assertEqual(candidate.read_bytes(), b"new-capture")
                return {"passed": True, "contract": "test"}

            report = promote_validated_capture(output, b"new-capture", validator)

            self.assertIs(report["passed"], True)
            self.assertEqual(output.read_bytes(), b"new-capture")
            self.assertEqual(list(root.glob("*.candidate-*.tmp")), [])

    def test_rejected_candidate_preserves_known_good_destination(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "capture.log"
            output.write_bytes(b"known-good")
            rejected = {"passed": False, "failures": ["drop counter is non-zero"]}

            with self.assertRaises(CaptureValidationError) as caught:
                promote_validated_capture(output, b"bad-capture", lambda _: rejected)

            self.assertIs(caught.exception.report, rejected)
            self.assertEqual(output.read_bytes(), b"known-good")
            self.assertEqual(list(root.glob("*.candidate-*.tmp")), [])

    def test_validator_error_does_not_create_destination(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "capture.log"

            def validator(_: Path) -> dict[str, object]:
                raise OSError("synthetic validator failure")

            with self.assertRaisesRegex(OSError, "synthetic validator failure"):
                promote_validated_capture(output, b"candidate", validator)

            self.assertFalse(output.exists())
            self.assertEqual(list(root.glob("*.candidate-*.tmp")), [])

    def test_output_and_manifest_must_be_distinct(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "same.file"
            argv = [
                "capture_telnet_log.py",
                "--host", "192.0.2.1",
                "--output", str(output),
                "--manifest", str(output),
            ]
            with mock.patch.object(sys, "argv", argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaisesRegex(SystemExit, "2"):
                    main()

    def test_bundle_prepares_manifest_before_replacing_pair(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "capture.log"
            manifest = root / "capture.log.manifest.json"
            output.write_bytes(b"old-log")
            manifest.write_bytes(b"old-manifest")

            report = promote_validated_capture_bundle(
                output,
                manifest,
                b"new-log",
                lambda candidate: {"passed": candidate.read_bytes() == b"new-log"},
                lambda validation, data: {
                    "validation": validation,
                    "size": len(data),
                },
            )

            self.assertTrue(report["passed"])
            self.assertEqual(output.read_bytes(), b"new-log")
            self.assertIn(b'"size": 7', manifest.read_bytes())
            self.assertEqual(list(root.glob("*.candidate-*.tmp")), [])
            self.assertEqual(list(root.glob("*.rollback-*.tmp")), [])

    def test_bundle_rejects_hardlinked_destinations(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "capture.log"
            manifest = root / "capture.log.manifest.json"
            output.write_bytes(b"same-inode")
            os.link(output, manifest)
            with self.assertRaisesRegex(ValueError, "must be distinct"):
                promote_validated_capture_bundle(
                    output,
                    manifest,
                    b"new-log",
                    lambda _: {"passed": True},
                    lambda _report, _data: {},
                )

    def test_manifest_factory_error_preserves_previous_pair(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "capture.log"
            manifest = root / "capture.log.manifest.json"
            output.write_bytes(b"old-log")
            manifest.write_bytes(b"old-manifest")

            def fail_manifest(_: dict[str, object], __: bytes) -> dict[str, object]:
                raise OSError("synthetic manifest failure")

            with self.assertRaisesRegex(OSError, "synthetic manifest failure"):
                promote_validated_capture_bundle(
                    output,
                    manifest,
                    b"new-log",
                    lambda _: {"passed": True},
                    fail_manifest,
                )

            self.assertEqual(output.read_bytes(), b"old-log")
            self.assertEqual(manifest.read_bytes(), b"old-manifest")

    def test_second_destination_replace_failure_rolls_back_pair(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "capture.log"
            manifest = root / "capture.log.manifest.json"
            output.write_bytes(b"old-log")
            manifest.write_bytes(b"old-manifest")
            real_replace = __import__("os").replace

            def replace_with_manifest_failure(source: object, destination: object) -> None:
                src = Path(source)  # type: ignore[arg-type]
                dst = Path(destination)  # type: ignore[arg-type]
                if ".candidate-" in src.name and dst == manifest:
                    raise OSError("synthetic second replace failure")
                real_replace(source, destination)

            with mock.patch("capture_telnet_log.os.replace", side_effect=replace_with_manifest_failure):
                with self.assertRaisesRegex(OSError, "synthetic second replace failure"):
                    promote_validated_capture_bundle(
                        output,
                        manifest,
                        b"new-log",
                        lambda _: {"passed": True},
                        lambda _report, _data: {"generation": "new"},
                    )

            self.assertEqual(output.read_bytes(), b"old-log")
            self.assertEqual(manifest.read_bytes(), b"old-manifest")

    def test_incomplete_rollback_restores_other_file_and_retains_backup(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "capture.log"
            manifest = root / "capture.log.manifest.json"
            output.write_bytes(b"old-log")
            manifest.write_bytes(b"old-manifest")
            real_replace = os.replace

            def replace_with_install_and_rollback_failures(
                source: object, destination: object
            ) -> None:
                src = Path(source)  # type: ignore[arg-type]
                dst = Path(destination)  # type: ignore[arg-type]
                if ".candidate-" in src.name and dst == manifest:
                    raise OSError("synthetic manifest install failure")
                if ".rollback-" in src.name and dst == output:
                    raise OSError("synthetic capture rollback failure")
                real_replace(source, destination)

            with mock.patch(
                "capture_telnet_log.os.replace",
                side_effect=replace_with_install_and_rollback_failures,
            ):
                with self.assertRaisesRegex(CaptureError, "rollback is incomplete"):
                    promote_validated_capture_bundle(
                        output,
                        manifest,
                        b"new-log",
                        lambda _: {"passed": True},
                        lambda _report, _data: {"generation": "new"},
                    )

            # The independent manifest restoration still ran, while the one
            # old generation that could not be restored remains recoverable.
            self.assertEqual(manifest.read_bytes(), b"old-manifest")
            backups = list(root.glob("capture.log.rollback-*.tmp"))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_bytes(), b"old-log")


if __name__ == "__main__":
    unittest.main()
