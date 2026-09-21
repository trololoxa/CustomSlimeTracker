"""Behavioral DEV-03 coverage without rebuilding the complete native matrix."""
from __future__ import annotations
import argparse
import contextlib
import io
import json
import os
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock

import check_all as checks
import gate_reporting as reports
import run_standalone_tests as native
from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[1]


class ReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = project_temp_directory(ROOT, "dev03-report-")
        self.root = Path(self.temp.__enter__())
        self.addCleanup(self.temp.__exit__, None, None, None)
        self.console = io.StringIO()
        self.redirect = contextlib.redirect_stdout(self.console)
        self.redirect.__enter__()
        self.addCleanup(self.redirect.__exit__, None, None, None)

    def report(self):
        return reports.GateReport(self.root, "check-all")

    def test_stdout_and_stderr_failure_retained(self):
        r = self.report()
        result = r.run([sys.executable, "-c", "import sys;print('stdout cause');print('stderr cause',file=sys.stderr);sys.exit(7)"],
                       cwd=self.root, timeout_s=5, env=os.environ)
        r.finish(result.returncode)
        self.assertEqual(result.returncode, 7)
        self.assertIn("stdout cause", result.stderr)
        self.assertIn("stderr cause", result.stderr)
        self.assertIn("stdout cause", self.console.getvalue())
        data = json.loads(r.path.read_text())
        self.assertEqual((data["state"], data["returncode"]), ("completed", 7))

    def test_success_noise_saved_and_excerpt_bounded(self):
        r = self.report()
        def fake(cmd, **kwargs):
            kwargs["stdout"].write(b"x" * 30000)
            return subprocess.CompletedProcess(cmd, 0)
        with mock.patch.object(reports, "run_bounded_process", side_effect=fake):
            result = r.run(["fixture"], cwd=self.root, timeout_s=1, env={})
        r.finish(result.returncode)
        self.assertEqual((r.directory / "0001.log").stat().st_size, 30000)
        self.assertNotIn("x" * 100, self.console.getvalue())
        self.assertLess(len(reports.excerpt(r.directory / "0001.log")), 6000)

    def test_actual_timeout_keeps_partial_log(self):
        r = self.report()
        result = r.run([sys.executable, "-c", "import time;print('partial',flush=True);time.sleep(10)"],
                       cwd=self.root, timeout_s=.5, env=os.environ)
        self.assertEqual(result.returncode, 124)
        self.assertIn("partial", result.stderr)
        self.assertEqual(r.data["commands"][0]["state"], "timeout")

    def test_launch_error(self):
        r = self.report()
        result = r.run([str(self.root / "missing")], cwd=self.root, timeout_s=1, env={})
        self.assertEqual(result.returncode, 127)
        self.assertIn("launch failed", result.stderr)

    def test_metadata_failure_is_unknown_and_bounded(self):
        with mock.patch.object(reports, "run_bounded_process", side_effect=subprocess.TimeoutExpired(["git"], 5)) as run:
            result = reports.source_snapshot(self.root)
        self.assertIsNone(result["commit"])
        self.assertIsNone(result["dirty"])
        self.assertEqual(run.call_count, 2)
        self.assertTrue(all(c.kwargs["timeout_s"] == 5 for c in run.call_args_list))

    def test_interruption_never_records_success(self):
        r = self.report()
        with mock.patch.object(reports, "run_bounded_process", side_effect=KeyboardInterrupt), self.assertRaises(KeyboardInterrupt):
            r.run(["fixture"], cwd=self.root, timeout_s=1, env={})
        r.finish(130, interrupted=True)
        data = json.loads(r.path.read_text())
        self.assertEqual(data["state"], "interrupted")
        self.assertEqual(data["commands"][0]["returncode"], 130)

    def test_unique_reports_keep_previous_evidence(self):
        a, b = self.report(), self.report()
        self.assertNotEqual(a.path, b.path)
        self.assertTrue(a.path.is_file())
        self.assertTrue(b.path.is_file())

    def test_reuse_ids_ignores_saved_interpreter(self):
        r = self.report()
        r.configure("focused", ["a", "b"])
        r.data.update(state="completed", returncode=1, commands=[
            {"command": ["untrusted-old-python", "tools/a.py"], "state": "completed", "returncode": 7},
            {"command": ["old-python", "tools/b.py"], "state": "completed", "returncode": 0}])
        r.save()
        self.assertEqual(reports.failed_checks(r.path, {"a": "tools/a.py", "b": "tools/b.py"}), ["a"])

    def test_invalid_reports_fail_closed(self):
        r = self.report()
        good = {"schema": reports.SCHEMA, "runner": "check-all", "scope": "focused", "state": "completed", "returncode": 1,
                "selection": ["a"], "commands": [{"command": ["python", "tools/a.py"], "state": "completed", "returncode": 1}]}
        cases = [[], {}, {**good, "returncode": 0}, {**good, "returncode": True}, {**good, "state": "running"},
                 {**good, "scope": "release"}, {**good, "selection": ["unknown"]}, {**good, "selection": ["a", "a"]},
                 {**good, "commands": []}, {**good, "commands": [None]}, {**good, "commands": good["commands"] * 2},
                 {**good, "commands": [{"command": ["python", "tools/a.py", "--evil"], "state": "completed", "returncode": 1}]}]
        for data in cases:
            with self.subTest(data=data):
                r.path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    reports.failed_checks(r.path, {"a": "tools/a.py"})


class SelectionTests(unittest.TestCase):
    def test_registry_unique_existing_files(self):
        scripts = [s for s, _ in (*checks.CONTRACT_CHECKS, *checks.TOOL_CHECKS)]
        self.assertEqual(len(scripts), len(set(scripts)))
        self.assertEqual(len(scripts), len(set(Path(s).stem for s in scripts)))
        for s in scripts:
            self.assertTrue((ROOT / s).is_file(), s)

    def test_list_has_no_execution_or_report(self):
        for module, option in ((checks, "--list-checks"), (native, "--list-tests")):
            with mock.patch.object(module, "GateReport") as report, contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(module.main([option]), 0)
            report.assert_not_called()
            self.assertTrue(out.getvalue().strip())

    def test_unknown_ids_fail_before_work(self):
        for module, option in ((checks, "--check"), (native, "--test")):
            with mock.patch.object(module, "GateReport") as report, contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as err:
                module.main([option, "not_a_test"])
            self.assertEqual(err.exception.code, 2)
            report.assert_not_called()

    def test_focused_cannot_bypass_strict_mode(self):
        for opt in ("--release", "--host-only", "--skip-native", "--skip-tool-smoke", "--skip-pio", "--clean", "--require-pio"):
            with mock.patch.object(checks, "GateReport") as report, contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                checks.main(["--check", "test_build_identity", opt])
            report.assert_not_called()

    def test_focused_deduplicates_and_continues_after_failure(self):
        with (mock.patch.object(checks, "GateReport"), mock.patch.object(checks, "run_command", side_effect=[
                checks.CheckCommandResult(7, "cause"), checks.CheckCommandResult(0)]) as run,
              mock.patch.object(checks, "run_native_tests") as full,
              mock.patch.object(checks, "run_pio_builds") as pio, contextlib.redirect_stdout(io.StringIO()) as out):
            result = checks.main(["--check", "test_build_identity", "--check", "test_check_all_policy", "--check", "test_build_identity"])
        self.assertEqual(result, 1)
        self.assertEqual([c.args[0][1] for c in run.call_args_list], ["tools/test_build_identity.py", "tools/test_check_all_policy.py"])
        full.assert_not_called(); pio.assert_not_called()
        self.assertIn("partial", out.getvalue())

    def test_default_keeps_all_gate_groups(self):
        with (mock.patch.object(checks, "GateReport"), mock.patch.object(checks, "run_native_tests") as native_run,
              mock.patch.object(checks, "run_project_contract_checks") as contracts,
              mock.patch.object(checks, "run_tool_smokes") as smokes,
              mock.patch.object(checks, "run_pio_builds", return_value=([], [], False)) as pio,
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(checks.main([]), 0)
        for item in (native_run, contracts, smokes, pio): item.assert_called_once()

    def test_full_tool_loop_schedules_each_entry(self):
        with (mock.patch.object(checks, "run_checked", return_value=False) as run,
              mock.patch.object(checks, "run_replay_gate", return_value=False),
              mock.patch.object(checks.Path, "exists", return_value=False)):
            checks.run_tool_smokes(checks.CheckSummary(), timeout_s=1)
        self.assertEqual([c.args[1][1] for c in run.call_args_list], [s for s, _ in checks.TOOL_CHECKS])

    def test_release_retains_three_clean_native_gates_and_replay(self):
        with (mock.patch.object(checks, "GateReport"), mock.patch.object(checks, "release_preflight", return_value=None),
              mock.patch.object(checks, "run_native_tests") as run, mock.patch.object(checks, "run_project_contract_checks"),
              mock.patch.object(checks, "run_tool_smokes"), mock.patch.object(checks, "run_release_logver3_gate") as replay,
              mock.patch.object(checks, "run_pio_builds", return_value=([], [], False)), contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(checks.main(["--release"]), 0)
        self.assertEqual(run.call_count, 3)
        self.assertTrue(all(c.args[1] for c in run.call_args_list))
        self.assertEqual([c.kwargs.get("sanitizer", "none") for c in run.call_args_list], ["none", "address-undefined", "leak"])
        replay.assert_called_once()

    def test_nonfinite_timeouts_rejected(self):
        for value in ("nan", "inf", "0"):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                checks.main(["--tool-timeout-s", value])

    def test_explicit_compiler_does_not_fall_back(self):
        with mock.patch.object(native.shutil, "which", return_value=None) as which, self.assertRaises(RuntimeError):
            native.find_compiler("missing")
        which.assert_called_once_with("missing")

    def test_focused_native_keeps_extra_link_dependencies(self):
        sources = [ROOT / "tests/native/test_sensor_calibration.cpp", ROOT / "tests/native/test_other.cpp"]
        args = argparse.Namespace(clean=False, tests=["test_sensor_calibration"], build_only=False, sanitizer="none")
        with (mock.patch.object(native, "configure_build_directory", return_value=ROOT / "build/native_tests/fixture"),
              mock.patch.object(native, "test_sources", return_value=sources),
              mock.patch.object(native, "compile_project_objects", return_value=([Path(str(p)+'.o') for p in native.PROJECT_SOURCES], [])),
              mock.patch.object(native, "compile_object", return_value=True) as obj,
              mock.patch.object(native, "compile_one", return_value=True) as link,
              mock.patch.object(native, "run_one", return_value=True) as run, contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(native._run_suite(args, "cxx", []), 0)
        self.assertEqual([c.args[1] for c in obj.call_args_list], list(native.TEST_EXTRA_LINK_SOURCES[sources[0].name]))
        link.assert_called_once(); run.assert_called_once()
        self.assertEqual(link.call_args.args[1], sources[0])

    def test_full_native_keeps_compile_only_and_build_only(self):
        args = argparse.Namespace(clean=False, tests=None, build_only=True, sanitizer="none")
        with (mock.patch.object(native, "configure_build_directory", return_value=ROOT / "build/native_tests/fixture"),
              mock.patch.object(native, "test_sources", return_value=[ROOT / "tests/native/test_example.cpp"]),
              mock.patch.object(native, "compile_project_objects", return_value=([Path(str(p)+'.o') for p in native.PROJECT_SOURCES], [])),
              mock.patch.object(native, "compile_object", return_value=True) as obj,
              mock.patch.object(native, "compile_one", return_value=True), mock.patch.object(native, "run_one") as run,
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(native._run_suite(args, "cxx", []), 0)
        self.assertEqual(obj.call_count, len(native.COMPILE_ONLY_SOURCES) + len(native.ARDUINO_COMPILE_ONLY_SOURCES))
        run.assert_not_called()

    def test_pio_full_output_still_reaches_size_parser(self):
        with project_temp_directory(ROOT, "dev03-pio-") as raw, contextlib.redirect_stdout(io.StringIO()):
            folder = Path(raw); report = reports.GateReport(folder, "check-all")
            output = "RAM: [==] 1.0% (used 1 bytes from 100 bytes)\nFlash: [==] 2.0% (used 2 bytes from 100 bytes)\n"
            def fake(cmd, **kwargs):
                kwargs["stdout"].write(output.encode()); return subprocess.CompletedProcess(cmd, 0)
            with mock.patch.object(checks, "_REPORT", report), mock.patch.object(reports, "run_bounded_process", side_effect=fake):
                result = checks.run_pio_build("pio", "FIXTURE", folder)
            self.assertEqual(result.returncode, 0)
            self.assertIn(output, result.output)
            self.assertEqual(len(checks.parse_size_metrics(result.output)), 2)

    def test_rerun_uses_current_registry_and_python(self):
        with (mock.patch.object(checks, "GateReport"), mock.patch.object(checks, "failed_checks", return_value=["test_build_identity"]),
              mock.patch.object(checks, "run_command", return_value=checks.CheckCommandResult(0)) as run,
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(checks.main(["--failed-from", "old.json"]), 0)
        self.assertEqual(run.call_args.args[0], [sys.executable, "tools/test_build_identity.py"])

    def test_real_failure_fix_rerun_preserves_old_report(self):
        with project_temp_directory(ROOT, "dev03-rerun-") as raw, contextlib.redirect_stdout(io.StringIO()):
            root = Path(raw); (root / "tools").mkdir()
            a, b = root / "tools/test_build_identity.py", root / "tools/test_check_all_policy.py"
            a.write_text("print('injected failure'); raise SystemExit(7)\n")
            b.write_text("print('passing fixture')\n")
            original_run = checks.run_command
            def isolated(cmd, **kwargs):
                return original_run(cmd, cwd=root, timeout_s=kwargs["timeout_s"])
            with mock.patch.object(checks, "ROOT", root), mock.patch.object(checks, "run_command", side_effect=isolated):
                self.assertEqual(checks.main(["--check", "test_build_identity", "--check", "test_check_all_policy"]), 1)
                old = next((root / "build/gate_runs").glob("*/summary.json")); saved = old.read_bytes()
                a.write_text("print('fixed')\n"); b.write_text("raise RuntimeError('must not rerun successful test')\n")
                self.assertEqual(checks.main(["--failed-from", str(old)]), 0)
            self.assertEqual(old.read_bytes(), saved)
            new = next(p for p in (root / "build/gate_runs").glob("*/summary.json") if p != old)
            data = json.loads(new.read_text())
            self.assertEqual(data["selection"], ["test_build_identity"])
            self.assertEqual(len(data["commands"]), 1)


if __name__ == "__main__":
    unittest.main()
