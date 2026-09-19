#!/usr/bin/env python3
"""DEV-01 behavioral tests: assertions, effective sanitizers and strict gates."""
from __future__ import annotations

import contextlib
import io
import json
import os
import subprocess
import unittest
from pathlib import Path
from unittest import mock

import run_standalone_tests as native
import sanitizer_probe as probe
from quality_gate_runtime import project_temp_directory, run_bounded_process

ROOT = Path(__file__).resolve().parents[1]


class AssertionTests(unittest.TestCase):
    def test_real_assertions_and_exit_codes(self) -> None:
        with project_temp_directory(ROOT, "assertion-selftest-") as raw:
            exe = Path(raw) / ("assertions.exe" if os.name == "nt" else "assertions")
            command = [native.find_compiler(None), "-std=c++20", "-O2", "-Werror",
                       "-Itests/native", "tests/fixtures/assertion_probe.cpp", "-o", str(exe)]
            built = run_bounded_process(command, cwd=ROOT, timeout_s=30,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
            for case in range(19):
                with self.subTest(case=case):
                    result = run_bounded_process([str(exe), str(case)], timeout_s=5,
                                                 stdout=subprocess.PIPE,
                                                 stderr=subprocess.PIPE, text=True)
                    expected = 0 if case < 5 else 1
                    self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
                    if expected:
                        self.assertIn("FAIL", result.stderr)
                        self.assertIn("failures=1", result.stderr)
                        self.assertNotIn("PASS", result.stdout)
                    else:
                        self.assertIn("PASS assertion_probe", result.stdout)
                        self.assertEqual(result.stderr, "")


class SanitizerProbeTests(unittest.TestCase):
    def fake_run(self, command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        self.assertGreater(kwargs["timeout_s"], 0)
        if "-o" in command:
            Path(command[command.index("-o") + 1]).write_bytes(b"fake executable")
            return subprocess.CompletedProcess(command, 0, "", "")
        if len(command) == 1:
            return subprocess.CompletedProcess(command, 0, "", "")
        kind = Path(command[0]).stem
        messages = {"address": "AddressSanitizer: heap-buffer-overflow",
                    "undefined": "runtime error: signed integer overflow",
                    "leak": "LeakSanitizer: detected memory leaks"}
        return subprocess.CompletedProcess(command, 1, "", messages[kind])

    def test_combined_probe_proves_both_runtimes(self) -> None:
        with mock.patch.object(probe, "run_bounded_process", side_effect=self.fake_run):
            result = probe.probe_sanitizer("cxx", ROOT, "address-undefined")
        self.assertTrue(result.supported)
        self.assertEqual([s["stage"] for s in result.stages], [
            "address:compile", "address:clean", "address:fault",
            "undefined:compile", "undefined:clean", "undefined:fault"])

    def test_missing_runtime_clean_crash_and_wrong_fault_are_not_support(self) -> None:
        for stage in ("compile", "clean", "fault"):
            for code, message in ((0, ""), (1, "ordinary error"), (-11, "crash")):
                if stage == "clean" and code == 0:
                    continue
                with self.subTest(stage=stage, code=code):
                    def fake(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
                        actual = "compile" if "-o" in command else "clean" if len(command) == 1 else "fault"
                        if actual == stage:
                            # compile return 0 without an artifact must also fail.
                            return subprocess.CompletedProcess(command, code, "", message)
                        return self.fake_run(command, **kwargs)
                    with mock.patch.object(probe, "run_bounded_process", side_effect=fake):
                        self.assertFalse(probe.probe_sanitizer("cxx", ROOT, "undefined").supported)

    def test_diagnostic_with_zero_exit_is_not_support(self) -> None:
        def fake(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
            result = self.fake_run(command, **kwargs)
            result.returncode = 0
            return result
        with mock.patch.object(probe, "run_bounded_process", side_effect=fake):
            self.assertFalse(probe.probe_sanitizer("cxx", ROOT, "undefined").supported)

    def test_timeout_cannot_pass_even_with_expected_diagnostic(self) -> None:
        def fake(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
            if command[-1] == "fault":
                raise subprocess.TimeoutExpired(command, 1, output=b"partial",
                                                stderr=b"runtime error: signed integer overflow")
            return self.fake_run(command, **kwargs)
        with mock.patch.object(probe, "run_bounded_process", side_effect=fake):
            result = probe.probe_sanitizer("cxx", ROOT, "undefined")
        self.assertFalse(result.supported)
        self.assertEqual(result.stages[-1]["status"], "timeout")
        self.assertEqual(result.stages[-1]["stdout"], "partial")

    def test_missing_compiler_and_complete_evidence_report(self) -> None:
        with mock.patch.object(probe, "run_bounded_process", side_effect=FileNotFoundError("missing")):
            result = probe.probe_sanitizer("missing", ROOT, "leak")
        self.assertFalse(result.supported)
        with project_temp_directory(ROOT, "probe-report-") as raw:
            report = probe.write_probe_report(result, Path(raw))
            saved = json.loads(report.read_text())
        self.assertEqual(saved["stages"][0]["status"], "launch_error")
        self.assertIn("missing", saved["stages"][0]["stderr"])

    def test_invalid_deadlines_and_mode_rejected(self) -> None:
        for timeout in (0, -1, float("nan"), float("inf")):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                probe.probe_sanitizer("cxx", ROOT, "leak", timeout_s=timeout)
        with self.assertRaises(ValueError):
            probe.probe_sanitizer("cxx", ROOT, "none")

    def test_leak_probe_requires_clean_run_and_detected_leak(self) -> None:
        with mock.patch.object(probe, "run_bounded_process", side_effect=self.fake_run):
            result = probe.probe_sanitizer("cxx", ROOT, "leak")
        self.assertTrue(result.supported)
        self.assertEqual(len(result.stages), 3)

    def test_cli_returns_failure_if_any_requested_mode_is_unavailable(self) -> None:
        with (mock.patch.object(probe, "probe_sanitizer", side_effect=[
                probe.ProbeResult("undefined", "cxx", "supported", "ok"),
                probe.ProbeResult("leak", "cxx", reason="unsupported"),
              ]),
              mock.patch.object(probe, "write_probe_report", return_value=Path("report.json")),
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(probe.main(["--sanitizer", "undefined", "--sanitizer", "leak"]), 1)

    def test_native_explicit_sanitizer_cannot_fall_back(self) -> None:
        result = probe.ProbeResult("undefined", "cxx", reason="clean failed")
        with (mock.patch.object(native, "find_compiler", return_value="cxx"),
              mock.patch.object(native, "native_runner_lock", return_value=contextlib.nullcontext()),
              mock.patch.object(native, "_run_suite") as suite,
              mock.patch.object(probe, "probe_sanitizer", return_value=result),
              mock.patch.object(probe, "write_probe_report", return_value=Path("report.json")),
              contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO())):
            self.assertEqual(native.main(["--sanitizer", "undefined", "--build-only"]), 1)
        suite.assert_not_called()

    def test_native_runs_suite_after_success_and_propagates_failure(self) -> None:
        result = probe.ProbeResult("undefined", "cxx", "supported", "ok")
        with (mock.patch.object(native, "find_compiler", return_value="cxx"),
              mock.patch.object(native, "native_runner_lock", return_value=contextlib.nullcontext()),
              mock.patch.object(native, "_run_suite", return_value=17) as suite,
              mock.patch.object(probe, "probe_sanitizer", return_value=result),
              mock.patch.object(probe, "write_probe_report", return_value=Path("report.json")),
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(native.main(["--sanitizer", "undefined"]), 17)
        suite.assert_called_once()


if __name__ == "__main__":
    unittest.main()
