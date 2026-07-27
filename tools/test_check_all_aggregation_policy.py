#!/usr/bin/env python3
from __future__ import annotations

import types
import unittest
from unittest import mock

import check_all


class CheckAllAggregationPolicyTests(unittest.TestCase):
    def test_failed_command_is_recorded_without_exception(self) -> None:
        summary = check_all.CheckSummary()
        completed = types.SimpleNamespace(returncode=7)
        with mock.patch.object(check_all.subprocess, "run", return_value=completed):
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


if __name__ == "__main__":
    unittest.main()
