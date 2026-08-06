#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import subprocess
import unittest
from unittest import mock

import run_standalone_tests
from quality_gate_runtime import project_temp_directory


ROOT = pathlib.Path(__file__).resolve().parents[1]


class RunStandaloneTestsPolicyTests(unittest.TestCase):
    def test_extra_cxxflags_are_forwarded_to_link_step(self) -> None:
        extra = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        with (
            mock.patch.object(run_standalone_tests, "run_bounded_process") as run,
            mock.patch.object(run_standalone_tests, "_output_is_valid", return_value=True),
        ):
            run.return_value = subprocess.CompletedProcess([], 0)
            run_standalone_tests.compile_one(
                "clang++",
                pathlib.Path("tests/native/test_example.cpp"),
                pathlib.Path("build/native_tests/test_example"),
                [pathlib.Path("build/native_tests/obj/project.o")],
                extra,
            )

        self.assertEqual(run.call_count, 2)
        link_cmd = run.call_args_list[1].args[0]
        for flag in extra:
            self.assertIn(flag, link_cmd)
        for call in run.call_args_list:
            self.assertEqual(call.kwargs["timeout_s"], run_standalone_tests.DEFAULT_COMMAND_TIMEOUT_S)

    def test_each_invocation_gets_a_private_build_directory(self) -> None:
        with project_temp_directory(ROOT, "native-build-policy-") as raw:
            test_root = pathlib.Path(raw)
            with (
                mock.patch.object(run_standalone_tests, "BUILD_ROOT", test_root),
                mock.patch.object(run_standalone_tests, "BUILD_DIR", test_root),
                mock.patch.object(run_standalone_tests, "OBJ_DIR", test_root / "obj"),
            ):
                first = run_standalone_tests.configure_build_directory(clean=False)
                second = run_standalone_tests.configure_build_directory(clean=False)
                third = run_standalone_tests.configure_build_directory(clean=False)

            self.assertNotEqual(first, second)
            self.assertNotEqual(second, third)
            self.assertEqual(second.parent, test_root)
            self.assertTrue((third / "obj").is_dir())
            retained = [path for path in (first, second, third) if path.exists()]
            self.assertEqual(len(retained), 2)
            self.assertIn(third, retained)
            for path in retained:
                self.assertTrue((path / "obj").is_dir())

    def test_native_runner_lock_rejects_overlapping_invocation(self) -> None:
        with project_temp_directory(ROOT, "native-lock-policy-") as raw:
            lock_path = pathlib.Path(raw) / "native-tests.lock"
            with run_standalone_tests.native_runner_lock(lock_path):
                with self.assertRaises(run_standalone_tests.NativeRunnerBusyError):
                    with run_standalone_tests.native_runner_lock(lock_path):
                        self.fail("overlapping runner unexpectedly acquired the lock")


if __name__ == "__main__":
    unittest.main()
