#!/usr/bin/env python3
"""DEV-04 deadline dispatch, identity checks and navigation metadata contracts."""
from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock

import check_all as checks
import dev_source_check as identity
import export_native_compdb as compdb
import run_standalone_tests as native
from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[1]


class DeadlineTests(unittest.TestCase):
    def exercise(self, args: list[str]) -> tuple[dict, list[float]]:
        seen = []
        metadata = {}
        def suite(options, cxx, flags):
            metadata.update(native._REPORT.data['metadata'])
            native.compile_one(cxx, ROOT / 'tests/native/test_core_math_ahrs.cpp',
                               ROOT / 'build/dev04-test-exe', [], flags)
            native.run_one(ROOT / 'build/dev04-test-exe')
            return 0
        def command(cmd, **kwargs):
            seen.append(kwargs['timeout_s'])
            return subprocess.CompletedProcess(cmd, 0, '', '')
        with (contextlib.redirect_stdout(io.StringIO()),
              mock.patch.object(native, '_run_suite', side_effect=suite),
              mock.patch.object(native, 'find_compiler', return_value=sys.executable),
              mock.patch.object(native, '_output_is_valid', return_value=True),
              mock.patch('gate_reporting.GateReport.run', side_effect=command),
              mock.patch('sanitizer_probe.probe_sanitizer') as probe,
              mock.patch('sanitizer_probe.write_probe_report', return_value=Path('probe.json'))):
            probe.return_value.supported = True
            probe.return_value.status = 'supported'
            probe.return_value.reason = 'fixture'
            before = (native._COMMAND_TIMEOUT_S, native._BUILD_TIMEOUT_S)
            self.assertEqual(native.main(args), 0)
            self.assertEqual(before, (native._COMMAND_TIMEOUT_S, native._BUILD_TIMEOUT_S))
            if '--sanitizer' in args:
                self.assertEqual(probe.call_args.kwargs['timeout_s'], min(metadata['build_timeout_s'], 30.0))
        return metadata, seen

    def test_defaults_and_stage_precedence(self):
        for args, expected in [([], [180, 180, 180]),
                (['--sanitizer', 'address-undefined'], [600, 600, 180]),
                (['--sanitizer', 'leak'], [600, 600, 180]),
                (['--sanitizer', 'address-undefined', '--test-timeout-s', '1'], [600, 600, 1]),
                (['--sanitizer', 'address-undefined', '--timeout-s', '7'], [7, 7, 7]),
                (['--build-timeout-s', '9', '--timeout-s', '7', '--test-timeout-s', '3'], [9, 9, 3]),
                (['--timeout-s', '7', '--build-timeout-s', '9'], [9, 9, 7])]:
            with self.subTest(args=args):
                metadata, seen = self.exercise(args)
                self.assertEqual(seen, expected)
                self.assertEqual(metadata['build_timeout_s'], expected[0])
                self.assertEqual(metadata['test_timeout_s'], expected[-1])

    def test_invalid_deadlines_rejected_before_build(self):
        for flag in ('--timeout-s', '--build-timeout-s', '--test-timeout-s'):
            for value in ('0', '-1', 'nan', 'inf'):
                with (self.subTest(flag=flag, value=value), mock.patch.object(native, '_run_suite') as run,
                      contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit)):
                    native.main([f'{flag}={value}'])
                run.assert_not_called()

    def test_real_test_timeout_is_not_extended_by_build_budget(self):
        with (mock.patch.object(native, '_REPORT', None),
              mock.patch.object(native, '_COMMAND_TIMEOUT_S', .1),
              mock.patch.object(native, '_BUILD_TIMEOUT_S', 600),
              contextlib.redirect_stderr(io.StringIO())):
            self.assertEqual(native._run_command([sys.executable, '-c', 'import time; time.sleep(10)']), 124)

    def test_check_all_forwarding_preserves_outer_deadline(self):
        with mock.patch.object(checks, 'run_checked') as run:
            checks.run_native_tests(checks.CheckSummary(), False, suite_timeout_s=1000,
                                    command_timeout_s=None, sanitizer='address-undefined')
            cmd = run.call_args.args[1]
            self.assertNotIn('--timeout-s', cmd)
            checks.run_native_tests(checks.CheckSummary(), False, suite_timeout_s=1000,
                                    command_timeout_s=7, build_timeout_s=11, test_timeout_s=3)
            cmd = run.call_args.args[1]
            for option, expected in [('--timeout-s', '7'), ('--build-timeout-s', '11'), ('--test-timeout-s', '3')]:
                self.assertEqual(cmd[cmd.index(option) + 1], expected)
            self.assertEqual(run.call_args.kwargs['timeout_s'], 1000)

    def test_aggregate_rejects_invalid_stage_options(self):
        for flag in ('--native-build-timeout-s', '--native-test-timeout-s', '--native-command-timeout-s'):
            for value in ('0', '-1', 'nan', 'inf'):
                with (self.subTest(flag=flag, value=value), contextlib.redirect_stderr(io.StringIO()),
                      contextlib.redirect_stdout(io.StringIO()), mock.patch.object(checks, 'run_native_tests') as run,
                      self.assertRaises(SystemExit)):
                    checks.main([f'{flag}={value}'])
                run.assert_not_called()


class IdentityTests(unittest.TestCase):
    def test_clean_mismatch_dirty_and_ignored(self):
        with project_temp_directory(ROOT, 'dev04-identity-') as raw:
            root = Path(raw)
            def git(*args):
                result = subprocess.run(['git', '-C', str(root), *args], capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stderr)
                return result.stdout.strip()
            git('init', '-q')
            (root / '.gitignore').write_text('build/\n')
            git('add', '.gitignore')
            git('-c', 'user.name=Fixture', '-c', 'user.email=fixture@local', '-c', 'commit.gpgsign=false',
                '-c', f'core.hooksPath={root / "empty-hooks"}', 'commit', '-qm', 'fixture')
            head = git('rev-parse', 'HEAD')
            self.assertEqual(identity.check(root, head)[0], 0)
            self.assertEqual(identity.check(root, '0' * 40)[0], 1)
            (root / 'build').mkdir()
            (root / 'build/ignored.txt').write_text('ignored')
            self.assertEqual(identity.check(root, head)[0], 0)
            (root / 'changed.txt').write_text('untracked')
            self.assertEqual(identity.check(root, head)[0], 1)
            self.assertTrue((root / 'changed.txt').exists())

    def test_invalid_identity_and_git_failure(self):
        with mock.patch.object(identity, 'run_bounded_process', side_effect=OSError):
            self.assertEqual(identity.check(ROOT, None)[0], 1)
            self.assertEqual(identity.check(ROOT, '--malicious')[0], 2)


class DatabaseTests(unittest.TestCase):
    def data(self):
        argv = ['/compiler path/g++', '-DNAME=a b', '-Isrc', 'tests/native/test_core_math_ahrs.cpp', '-c', '-o', '/out path/a.o']
        return {'schema': 'tracker-gate-run-v1', 'runner': 'native', 'scope': 'focused',
                'state': 'completed', 'returncode': 0,
                'commands': [{'command': argv, 'cwd': str(ROOT), 'state': 'completed', 'returncode': 0}]}

    def test_argv_preserved_without_execution(self):
        data = self.data()
        output = compdb.entries(data)
        self.assertEqual(output[0]['arguments'], data['commands'][0]['command'])
        self.assertTrue(Path(output[0]['file']).is_absolute())
        data['commands'].append({'command': ['fake', '-o', 'binary'], 'returncode': 0})
        self.assertEqual(len(compdb.entries(data)), 1)

    def test_invalid_and_foreign_reports_rejected(self):
        for key, value in [('state', 'running'), ('returncode', 1), ('runner', 'check_all'), ('scope', 'release')]:
            data = self.data(); data[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                compdb.entries(data)
        data = self.data(); data['commands'][0]['cwd'] = '/another-checkout'
        with self.assertRaises(ValueError): compdb.entries(data)
        data = self.data(); data['commands'][0]['returncode'] = 124
        with self.assertRaises(ValueError): compdb.entries(data)
        data = self.data(); data['commands'] = []
        with self.assertRaises(ValueError): compdb.entries(data)

    def test_export_and_failure_preserve_previous_database(self):
        with project_temp_directory(ROOT, 'dev04-compdb-') as raw:
            root = Path(raw)
            source = root / 'test.cpp'; source.write_text('int x;')
            data = self.data()
            data['commands'][0]['cwd'] = str(root)
            data['commands'][0]['command'][3] = str(source)
            report = root / 'summary.json'; report.write_text(json.dumps(data))
            with (mock.patch.object(compdb, 'ROOT', root), contextlib.redirect_stdout(io.StringIO()),
                  mock.patch.object(sys, 'argv', ['export', '--report', str(report)])):
                self.assertEqual(compdb.main(), 0)
                target = root / 'build/clangd-native/compile_commands.json'
                before = target.read_bytes()
                data['returncode'] = 1; report.write_text(json.dumps(data))
                self.assertEqual(compdb.main(), 1)
                self.assertEqual(target.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
