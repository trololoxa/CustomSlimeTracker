#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import unittest
from unittest import mock

import run_standalone_tests


class RunStandaloneTestsPolicyTests(unittest.TestCase):
    def test_extra_cxxflags_are_forwarded_to_link_step(self) -> None:
        extra = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        with mock.patch.object(run_standalone_tests.subprocess, "run") as run:
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


if __name__ == "__main__":
    unittest.main()
