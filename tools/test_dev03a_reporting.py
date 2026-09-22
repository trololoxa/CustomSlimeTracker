"""DEV-03a failure injection and bounded progress checks; no firmware rebuild."""
from __future__ import annotations

import contextlib
import errno
import io
import json
import os
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock

import gate_reporting as reports
import quality_gate_runtime as runtime

ROOT = Path(__file__).resolve().parents[1]


def windows_error(code: int) -> OSError:
    error = PermissionError(errno.EACCES, "injected Windows file lock")
    error.winerror = code
    return error


class AtomicReportTests(unittest.TestCase):
    def setUp(self):
        self.directory = runtime.project_temp_directory(ROOT, "dev03a-")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.console = io.StringIO()
        redirect = contextlib.redirect_stdout(self.console)
        redirect.__enter__()
        self.addCleanup(redirect.__exit__, None, None, None)
        self.report = reports.GateReport(self.root, "native")

    def test_transient_windows_errors_preserve_old_until_replace(self):
        original = self.report.path.read_bytes()
        replace = os.replace
        calls = []
        def locked(src, dst):
            calls.append(1)
            self.assertEqual(self.report.path.read_bytes(), original)
            if len(calls) <= 3:
                raise windows_error((5, 32, 33)[len(calls) - 1])
            replace(src, dst)
        self.report.data["notes"].append("new state")
        with mock.patch.object(reports.os, "replace", side_effect=locked), mock.patch.object(reports.time, "sleep") as sleep:
            self.report.save()
        self.assertEqual(len(calls), 4)
        self.assertEqual(sleep.call_args_list, [mock.call(n) for n in reports.REPLACE_RETRY_DELAYS_S[:3]])
        self.assertEqual(json.loads(self.report.path.read_text())["notes"], ["new state"])
        self.assertIn("recovered after 3", self.console.getvalue())

    def test_permanent_lock_is_bounded_and_preserves_old_and_candidate(self):
        original = self.report.path.read_bytes()
        self.report.data["notes"].append("candidate")
        with (mock.patch.object(reports.os, "replace", side_effect=windows_error(5)) as replace,
              mock.patch.object(reports.time, "sleep") as sleep,
              self.assertRaises(PermissionError)):
            self.report.save()
        self.assertEqual(replace.call_count, 7)
        self.assertEqual(sleep.call_args_list, [mock.call(n) for n in reports.REPLACE_RETRY_DELAYS_S])
        self.assertLess(sum(reports.REPLACE_RETRY_DELAYS_S), 1)
        self.assertEqual(self.report.path.read_bytes(), original)
        self.assertEqual(json.loads(self.report.path.with_suffix(".tmp").read_text())["notes"], ["candidate"])

    def test_other_errors_are_not_retried(self):
        for error in (PermissionError(errno.EACCES, "POSIX permission"), OSError(errno.ENOSPC, "disk full"), windows_error(87)):
            with (self.subTest(error=error), mock.patch.object(reports.os, "replace", side_effect=error) as replace,
                  mock.patch.object(reports.time, "sleep") as sleep, self.assertRaises(OSError)):
                self.report.save()
            self.assertEqual(replace.call_count, 1)
            sleep.assert_not_called()

    def test_write_failure_does_not_replace_old_report(self):
        original = self.report.path.read_bytes()
        with (mock.patch.object(Path, "write_text", side_effect=OSError(errno.ENOSPC, "disk full")),
              mock.patch.object(reports.os, "replace") as replace, self.assertRaises(OSError)):
            self.report.save()
        replace.assert_not_called()
        self.assertEqual(self.report.path.read_bytes(), original)

    def test_locked_precommand_report_never_launches_command(self):
        with (mock.patch.object(reports.os, "replace", side_effect=windows_error(32)),
              mock.patch.object(reports.time, "sleep"), mock.patch.object(reports, "run_bounded_process") as run,
              self.assertRaises(PermissionError)):
            self.report.run(["fixture"], cwd=self.root, timeout_s=1, env={})
        run.assert_not_called()
        self.assertEqual(json.loads(self.report.path.read_text())["state"], "running")

    def test_transient_final_write_never_reruns_command(self):
        replace = os.replace
        calls = 0
        def locked(src, dst):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise windows_error(5)
            replace(src, dst)
        with (mock.patch.object(reports.os, "replace", side_effect=locked), mock.patch.object(reports.time, "sleep"),
              mock.patch.object(reports, "run_bounded_process", return_value=subprocess.CompletedProcess(["fixture"], 9)) as run):
            result = self.report.run(["fixture"], cwd=self.root, timeout_s=1, env={})
        self.assertEqual(result.returncode, 9)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(json.loads(self.report.path.read_text())["commands"][0]["returncode"], 9)

    @unittest.skipUnless(os.name == "nt", "real Windows sharing-lock test")
    def test_real_windows_sharing_lock(self):
        import ctypes
        from ctypes import wintypes
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                                      wintypes.LPVOID, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
        kernel.CreateFileW.restype = wintypes.HANDLE
        kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel.CloseHandle.restype = wintypes.BOOL
        # Allow readers/writers, deny delete/rename until the simulated reader closes.
        handle = kernel.CreateFileW(str(self.report.path), 0x80000000, 3, None, 3, 0x80, None)
        self.assertNotEqual(handle, ctypes.c_void_p(-1).value)
        def release(_):
            nonlocal handle
            self.assertTrue(kernel.CloseHandle(handle))
            handle = None
        try:
            self.report.data["notes"].append("unlocked")
            with mock.patch.object(reports.time, "sleep", side_effect=release) as sleep:
                self.report.save()
            self.assertEqual(sleep.call_count, 1)
        finally:
            if handle is not None:
                kernel.CloseHandle(handle)
        self.assertEqual(json.loads(self.report.path.read_text())["notes"], ["unlocked"])


class ProgressTests(unittest.TestCase):
    def test_absolute_deadline_and_collected_output(self):
        clock = [0.0]
        process = mock.Mock(returncode=None)
        timeouts, updates = [], []
        def wait(*, timeout):
            timeouts.append(timeout)
            clock[0] += timeout
            raise subprocess.TimeoutExpired(["fixture"], timeout)
        process.communicate.side_effect = wait
        with (mock.patch.object(runtime.subprocess, "Popen", return_value=process) as spawn,
              mock.patch.object(runtime.time, "monotonic", side_effect=lambda: clock[0]),
              mock.patch.object(runtime, "_kill_and_collect", return_value=("partial", "error")) as kill,
              self.assertRaises(subprocess.TimeoutExpired) as exc):
            runtime.run_bounded_process(["fixture"], timeout_s=25, on_progress=updates.append, progress_interval_s=10)
        self.assertEqual(timeouts, [10, 10, 5])
        self.assertEqual(updates, [10, 20])
        self.assertEqual(exc.exception.timeout, 25)
        self.assertEqual((exc.exception.output, exc.exception.stderr), ("partial", "error"))
        spawn.assert_called_once()
        kill.assert_called_once_with(process)

    def test_real_process_output_survives_multiple_waits(self):
        updates = []
        result = runtime.run_bounded_process(
            [sys.executable, "-c", "import time,sys;print('before',flush=True);time.sleep(.25);print('after');sys.exit(7)"],
            timeout_s=10, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            on_progress=updates.append, progress_interval_s=.03)
        self.assertEqual(result.returncode, 7)
        self.assertEqual(result.stdout.splitlines(), ["before", "after"])
        self.assertTrue(updates)

    def test_progress_callback_failure_and_interrupt_clean_up(self):
        for error in (RuntimeError("callback failed"), KeyboardInterrupt()):
            process = mock.Mock()
            process.communicate.side_effect = subprocess.TimeoutExpired(["fixture"], .01)
            with (self.subTest(error=error), mock.patch.object(runtime.subprocess, "Popen", return_value=process),
                  mock.patch.object(runtime, "_kill_and_collect") as kill, self.assertRaises(type(error))):
                runtime.run_bounded_process(["fixture"], timeout_s=10,
                                            on_progress=mock.Mock(side_effect=error), progress_interval_s=.01)
            kill.assert_called_once_with(process)

    def test_invalid_progress_deadline_rejected_before_spawn(self):
        for value in (0, -1, float("nan"), float("inf")):
            with (self.subTest(value=value), mock.patch.object(runtime.subprocess, "Popen") as spawn,
                  self.assertRaises(ValueError)):
                runtime.run_bounded_process(["fixture"], timeout_s=1, on_progress=lambda _: None,
                                            progress_interval_s=value)
            spawn.assert_not_called()

    def test_real_timeout_still_kills_descendant_with_progress(self):
        # Existing real descendant-kill regression, explicitly through the new polling path.
        from test_quality_gate_runtime import QualityGateRuntimeTests
        actual = runtime.run_bounded_process
        def polling(*args, **kwargs):
            return actual(*args, **kwargs, on_progress=lambda _: None, progress_interval_s=.01)
        with mock.patch("test_quality_gate_runtime.run_bounded_process", side_effect=polling):
            QualityGateRuntimeTests("test_bounded_process_timeout_kills_descendant_process").test_bounded_process_timeout_kills_descendant_process()

    def test_report_notices_throttled_and_fast_native_stays_quiet(self):
        with runtime.project_temp_directory(ROOT, "dev03a-progress-") as raw, contextlib.redirect_stdout(io.StringIO()) as console:
            root = Path(raw)
            r = reports.GateReport(root, "native")
            clock = [0.0]
            r._last_notice_at = 0
            def wait(cmd, **kwargs):
                for now in (10, 20, 30, 40, 50, 60):
                    clock[0] = now
                    kwargs["on_progress"](now)
                return subprocess.CompletedProcess(cmd, 0)
            with mock.patch.object(reports.time, "monotonic", side_effect=lambda: clock[0]), mock.patch.object(reports, "run_bounded_process", side_effect=wait):
                r.run(["cxx", "source.cpp"], cwd=root, timeout_s=100, env={})
            text = console.getvalue()
            self.assertEqual(text.count("# WAIT"), 2)
            self.assertNotIn("# RUN", text)
            self.assertIn("source.cpp", text)
            console.truncate(0)
            console.seek(0)
            with mock.patch.object(reports, "run_bounded_process", return_value=subprocess.CompletedProcess(["cxx"], 0)):
                r._last_notice_at = reports.time.monotonic()
                r.run(["cxx"], cwd=root, timeout_s=1, env={})
            self.assertEqual(console.getvalue(), "")

    def test_aggregate_prints_stage_before_child_launch(self):
        with runtime.project_temp_directory(ROOT, "dev03a-stage-") as raw, contextlib.redirect_stdout(io.StringIO()) as console:
            root = Path(raw)
            r = reports.GateReport(root, "check-all")
            def run(cmd, **kwargs):
                self.assertIn("# RUN 0001.log: pio.exe BOARD", console.getvalue())
                return subprocess.CompletedProcess(cmd, 0)
            with mock.patch.object(reports, "run_bounded_process", side_effect=run):
                r.run(["pio.exe", "run", "-e", "BOARD"], cwd=root, timeout_s=1, env={})


if __name__ == "__main__":
    unittest.main()
