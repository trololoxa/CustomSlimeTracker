#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import unittest
from pathlib import Path
from unittest import mock

import check_all
from quality_gate_runtime import project_temp_directory


class CheckAllAggregationPolicyTests(unittest.TestCase):
    def test_failed_command_is_recorded_without_exception(self) -> None:
        summary = check_all.CheckSummary()
        completed = subprocess.CompletedProcess([], 7)
        with mock.patch.object(check_all, "run_bounded_process", return_value=completed):
            ok = check_all.run_checked(summary, ["false-test"], "synthetic failure")
        self.assertFalse(ok)
        self.assertEqual(summary.failures, ["synthetic failure (exit=7)"])

    def test_require_pio_reports_failure_instead_of_raising(self) -> None:
        with mock.patch.object(check_all, "pio_executable", return_value=None):
            failures, warnings, skipped = check_all.run_pio_builds(None, True)
        self.assertTrue(skipped)
        self.assertFalse(warnings)
        self.assertEqual(len(failures), 1)
        self.assertIn("PlatformIO executable not found", failures[0])

    def test_timeout_is_aggregated_with_stable_exit_code(self) -> None:
        summary = check_all.CheckSummary()
        expired = subprocess.TimeoutExpired(["slow-test"], timeout=0.01)
        with mock.patch.object(check_all, "run_bounded_process", side_effect=expired):
            ok = check_all.run_checked(
                summary,
                ["slow-test"],
                "bounded command",
                timeout_s=0.01,
            )
        self.assertFalse(ok)
        self.assertEqual(summary.failures, ["bounded command (exit=124)"])

    def test_strict_modes_reject_coverage_reducing_options(self) -> None:
        common = dict(
            require_pio=False,
            pio_envs=None,
            skip_native=False,
            skip_pio=False,
            skip_tool_smoke=False,
            tool_timeout_s=1.0,
            native_suite_timeout_s=1.0,
            native_command_timeout_s=1.0,
            pio_timeout_s=1.0,
        )
        host = argparse.Namespace(host_only=True, release=False, **common)
        self.assertEqual(check_all.mode_errors(host), [])
        check_all.apply_mode(host)
        self.assertTrue(host.skip_pio)

        release = argparse.Namespace(host_only=False, release=True, **{**common, "skip_native": True})
        self.assertTrue(any("--release forbids" in error for error in check_all.mode_errors(release)))

    def test_release_preflight_fails_closed_for_dirty_source(self) -> None:
        dirty = {
            "git_available": True,
            "commit": "a" * 40,
            "head_short": "aaaaaaaa",
            "dirty": True,
            "worktree_fingerprint": "12345678",
            "identity": "deadbeef+12345678-dirty",
        }
        with mock.patch.object(check_all, "source_manifest", return_value=dirty):
            error = check_all.release_preflight()
        self.assertIsNotNone(error)
        self.assertIn("dirty", error or "")

    def test_clean_release_stays_blocked_until_strict_logver3_exists(self) -> None:
        clean = {
            "git_available": True,
            "commit": "a" * 40,
            "head_short": "aaaaaaaa",
            "dirty": False,
            "worktree_fingerprint": "12345678",
            "identity": "aaaaaaaa",
        }
        with mock.patch.object(check_all, "source_manifest", return_value=clean):
            error = check_all.release_preflight()
        self.assertIsNotNone(error)
        self.assertIn("strict LOGVER3", error or "")

    def test_release_target_build_cleans_before_compiling(self) -> None:
        completed = [
            subprocess.CompletedProcess([], 0, stdout="clean ok\n"),
            subprocess.CompletedProcess([], 0, stdout="build ok\n"),
        ]
        with project_temp_directory(check_all.ROOT, "pio-clean-policy-") as raw:
            with (
                mock.patch.object(check_all, "run_bounded_process", side_effect=completed) as run,
                mock.patch.object(check_all, "pio_artifacts", return_value=[]),
            ):
                result = check_all.run_pio_build(
                    "pio",
                    "TEST_ENV",
                    Path(raw),
                    clean_first=True,
                )
        self.assertTrue(result.ok)
        self.assertEqual(run.call_count, 2)
        self.assertEqual(run.call_args_list[0].args[0], ["pio", "run", "-e", "TEST_ENV", "-t", "clean"])
        self.assertEqual(run.call_args_list[1].args[0], ["pio", "run", "-e", "TEST_ENV"])

    def test_release_target_build_rejects_artifact_left_by_clean(self) -> None:
        completed = subprocess.CompletedProcess([], 0, stdout="clean claimed success\n")
        stale = check_all.ROOT / ".pio" / "build" / "TEST_ENV" / "firmware.bin"
        with project_temp_directory(check_all.ROOT, "pio-stale-policy-") as raw:
            with (
                mock.patch.object(check_all, "run_bounded_process", return_value=completed) as run,
                mock.patch.object(check_all, "pio_artifacts", return_value=[stale]),
            ):
                result = check_all.run_pio_build(
                    "pio",
                    "TEST_ENV",
                    Path(raw),
                    clean_first=True,
                )
        self.assertEqual(result.returncode, check_all.STALE_PIO_ARTIFACT_RETURNCODE)
        self.assertEqual(run.call_count, 1)


if __name__ == "__main__":
    unittest.main()
