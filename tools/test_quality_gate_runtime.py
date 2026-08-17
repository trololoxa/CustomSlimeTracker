#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

import run_standalone_tests
from quality_gate_runtime import (
    asan_ubsan_environment,
    project_temp_directory,
    quality_gate_environment,
    run_bounded_process,
    strongest_supported_sanitizer_flags,
)


ROOT = Path(__file__).resolve().parents[1]


class QualityGateRuntimeTests(unittest.TestCase):
    def test_project_temp_directory_is_below_ignored_build_tree(self) -> None:
        with project_temp_directory(ROOT, "runtime-policy-") as raw:
            path = Path(raw).resolve()
            path.relative_to((ROOT / "build" / "tmp").resolve())
            self.assertEqual(path.parent, (ROOT / "build" / "tmp" / "quality-gates").resolve())

    def test_project_temp_directory_restores_process_global_settings(self) -> None:
        saved_environment = {name: os.environ.get(name) for name in ("TMPDIR", "TEMP", "TMP")}
        saved_tempdir = tempfile.tempdir
        temporary = project_temp_directory(ROOT, "runtime-restore-")
        self.assertEqual(os.environ["TMPDIR"], str((ROOT / "build" / "tmp" / "quality-gates").resolve()))
        temporary.cleanup()
        self.assertEqual(
            {name: os.environ.get(name) for name in ("TMPDIR", "TEMP", "TMP")},
            saved_environment,
        )
        self.assertEqual(tempfile.tempdir, saved_tempdir)

    def test_quality_environment_supplies_all_standard_temp_variables(self) -> None:
        env = quality_gate_environment(ROOT, scope="runtime-policy-env", base={"KEEP": "yes"})
        self.assertEqual(env["KEEP"], "yes")
        self.assertEqual(env["TMPDIR"], env["TEMP"])
        self.assertEqual(env["TEMP"], env["TMP"])
        self.assertTrue(Path(env["TMPDIR"]).is_dir())
        self.assertEqual(env["PYTHONDONTWRITEBYTECODE"], "1")

    def test_asan_ubsan_environment_disables_implicit_lsan_only(self) -> None:
        env = asan_ubsan_environment(
            ROOT,
            base={
                "ASAN_OPTIONS": "symbolize=1:detect_leaks=1",
                "UBSAN_OPTIONS": "print_stacktrace=0",
            },
        )
        self.assertIn("symbolize=1", env["ASAN_OPTIONS"])
        self.assertIn("detect_leaks=0", env["ASAN_OPTIONS"])
        self.assertNotIn("detect_leaks=1", env["ASAN_OPTIONS"])
        self.assertIn("detect_leaks=0", env["LSAN_OPTIONS"])
        self.assertIn("halt_on_error=1", env["UBSAN_OPTIONS"])
        self.assertIn("print_stacktrace=1", env["UBSAN_OPTIONS"])

    def test_sanitizer_probe_falls_back_to_ubsan_when_address_runtime_is_missing(self) -> None:
        def fake_run(args: list[str], **_: object) -> subprocess.CompletedProcess[str]:
            if "-fsanitize=address,undefined" in args:
                return subprocess.CompletedProcess(args, 1)
            output = Path(args[args.index("-o") + 1])
            output.write_bytes(b"probe")
            return subprocess.CompletedProcess(args, 0)

        with mock.patch("quality_gate_runtime.subprocess.run", side_effect=fake_run):
            name, flags = strongest_supported_sanitizer_flags("synthetic-cxx", ROOT)
        self.assertEqual(name, "undefined")
        self.assertIn("-fsanitize=undefined", flags)
        self.assertNotIn("-fsanitize=address,undefined", flags)

    def test_sanitizer_probe_reports_environment_limitation_when_no_runtime_links(self) -> None:
        failed = subprocess.CompletedProcess(["synthetic-cxx"], 1)
        with mock.patch("quality_gate_runtime.subprocess.run", return_value=failed):
            name, flags = strongest_supported_sanitizer_flags("synthetic-cxx", ROOT)
        self.assertEqual(name, "none")
        self.assertEqual(flags, ())

    def test_extra_flag_cli_accepts_dash_prefixed_separate_values(self) -> None:
        normalized = run_standalone_tests._normalize_extra_cxxflag_args(
            ["--clean", "--extra-cxxflag", "-Werror", "--extra-cxxflag=-fno-exceptions"]
        )
        self.assertEqual(
            normalized,
            ["--clean", "--extra-cxxflag=-Werror", "--extra-cxxflag=-fno-exceptions"],
        )

    def test_standalone_timeout_has_stable_exit_code(self) -> None:
        expired = subprocess.TimeoutExpired(["synthetic"], timeout=0.01)
        with mock.patch.object(run_standalone_tests, "run_bounded_process", side_effect=expired):
            self.assertEqual(run_standalone_tests._run_command(["synthetic"]), 124)
        self.assertEqual(run_standalone_tests.command_failure_code(), 124)

    def test_bounded_process_reports_timeout_after_terminating_group(self) -> None:
        with self.assertRaises(subprocess.TimeoutExpired):
            run_bounded_process(
                [sys.executable, "-c", "import time; time.sleep(30)"],
                timeout_s=0.05,
            )

    def test_bounded_process_timeout_kills_descendant_process(self) -> None:
        with project_temp_directory(ROOT, "descendant-timeout-") as raw:
            marker = Path(raw) / "descendant-survived.txt"
            child = (
                "import time; from pathlib import Path; "
                f"time.sleep(0.4); Path({str(marker)!r}).write_text('survived')"
            )
            parent = (
                "import subprocess, sys, time; "
                f"subprocess.Popen([sys.executable, '-c', {child!r}]); time.sleep(30)"
            )
            with self.assertRaises(subprocess.TimeoutExpired):
                run_bounded_process(
                    [sys.executable, "-c", parent],
                    timeout_s=0.05,
                )
            time.sleep(0.6)
            self.assertFalse(marker.exists())

    def test_empty_compiler_output_is_fail_closed(self) -> None:
        with project_temp_directory(ROOT, "empty-output-") as raw:
            output = Path(raw) / "test-binary"
            output.write_bytes(b"")
            self.assertFalse(run_standalone_tests._output_is_valid(output, executable=True))
            self.assertEqual(
                run_standalone_tests.command_failure_code(),
                run_standalone_tests.INVALID_OUTPUT_RETURNCODE,
            )


if __name__ == "__main__":
    unittest.main()
