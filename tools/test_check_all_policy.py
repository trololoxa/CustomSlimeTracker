#!/usr/bin/env python3
from __future__ import annotations

import unittest

from check_all_policy import is_size_only_failure, is_size_overflow, parse_size_metrics


class CheckAllPolicyTests(unittest.TestCase):
    def test_size_overflow_classification(self) -> None:
        self.assertTrue(is_size_overflow("region `irom0_0_seg' overflowed by 1234 bytes"))
        self.assertTrue(is_size_overflow("Error: The program size (123) is greater than maximum allowed (100)"))
        self.assertFalse(is_size_overflow("undefined reference to `missing_symbol'"))
        self.assertTrue(is_size_only_failure("region `dram0_0_seg' overflowed by 64 bytes"))
        self.assertFalse(is_size_only_failure("region `irom0_0_seg' overflowed by 64 bytes\nfoo.cpp:12:3: error: bad type"))
        self.assertFalse(is_size_only_failure("region `irom0_0_seg' overflowed by 64 bytes\nundefined reference to `missing_symbol'"))

    def test_size_report_parsing(self) -> None:
        output = """RAM:   [====      ]  35.6% (used 116692 bytes from 327680 bytes)\nFlash: [========= ]  92.1% (used 1200000 bytes from 1302528 bytes)\n"""
        metrics = parse_size_metrics(output)
        self.assertEqual(len(metrics), 2)
        self.assertEqual(metrics[0].kind, "RAM")
        self.assertEqual(metrics[0].used, 116692)
        self.assertAlmostEqual(metrics[1].percent, 92.1)


if __name__ == "__main__":
    unittest.main()
