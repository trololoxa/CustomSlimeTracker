#!/usr/bin/env python3
"""DEV-05b: real local Git/files/processes, fake expensive gate stages."""
from __future__ import annotations

import contextlib
import copy
import hashlib
import io
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock

import check_all
import dev_verify as verify
from quality_gate_runtime import project_temp_directory, run_bounded_process

ROOT = Path(__file__).resolve().parents[1]


def record(argv, code=0, log='0001.log'):
    return {'command': argv, 'returncode': code, 'state': 'completed', 'log': log}


def native_report(mode='address-undefined', code=0):
    return {'schema': 'tracker-gate-run-v1', 'runner': 'native', 'scope': 'full-native',
            'state': 'completed', 'returncode': code, 'selection': ['test_one'],
            'metadata': {'sanitizer': mode}, 'commands': [record(['/tmp/test_one'], code)]}


def windows_report():
    commands = [record(['python', 'tools/run_standalone_tests.py'])]
    commands += [record(['python', path]) for path, _ in (*check_all.CONTRACT_CHECKS, *check_all.TOOL_CHECKS)]
    commands += [record(['python', 'tools/replay/replay_machine_log.py', *args]) for args in
                 (['--min-duration-s', '60'], [], ['--require-magr'])]
    commands += [record(['python', 'tools/replay/compare_replay_metrics.py']),
                 record(['python', 'tools/replay/replay_machine_log.py', '--min-duration-s', '600'])]
    commands += [record(['pio', 'run', '-e', name]) for name in
                 (*check_all.REQUIRED_PIO_ENVS, check_all.DEBUG_LINKCHECK_ENV, check_all.DEBUG_ENV)]
    return {'schema': 'tracker-gate-run-v1', 'runner': 'check-all', 'scope': 'default',
            'state': 'completed', 'returncode': 0, 'commands': commands}


class WorkspaceTest(unittest.TestCase):
    def setUp(self):
        self.temp = project_temp_directory(ROOT, 'dev05b-')
        self.base = Path(self.temp.__enter__())
        self.addCleanup(self.temp.__exit__, None, None, None)

    def repository(self, name='source with spaces'):
        root = self.base / name
        root.mkdir()
        verify.git(root, 'init')
        verify.git(root, 'config', 'user.email', 'fixture@example.invalid')
        verify.git(root, 'config', 'user.name', 'Fixture')
        verify.git(root, 'config', 'core.autocrlf', 'false')
        (root / '.gitignore').write_text('build/\ntests/fixtures/local*\n', encoding='utf-8')
        (root / 'tests/native').mkdir(parents=True)
        (root / 'tests/native/test_one.cpp').write_text('// fixture\n', encoding='utf-8')
        self.commit(root)
        return root

    def commit(self, root):
        verify.git(root, 'add', '--all')
        verify.git(root, 'commit', '-m', 'fixture')
        return verify.clean_head(root)

    def clone(self, root):
        dest = self.base / 'linux copy'
        verify.git(self.base, '-c', 'core.autocrlf=false', 'clone', '--no-hardlinks', str(root), str(dest))
        return dest

    def test_real_fast_forward_and_source_proof(self):
        source = self.repository('исходник with spaces')
        dest = self.clone(source)
        (source / 'next.txt').write_text('next\n', encoding='utf-8')
        head = self.commit(source)
        result = verify.sync_linux(dest, source, head)
        self.assertEqual(result['head'], head)
        verify.equal_inputs(verify.inventory(source), result)

    def test_dirty_and_diverged_checkouts_are_preserved(self):
        source = self.repository()
        dest = self.clone(source)
        (dest / 'unknown.txt').write_text('keep me', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'dirty'):
            verify.sync_linux(dest, source, verify.clean_head(source))
        self.assertEqual((dest / 'unknown.txt').read_text(), 'keep me')
        verify.git(dest, 'config', 'user.email', 'fixture@example.invalid')
        verify.git(dest, 'config', 'user.name', 'Fixture')
        old = self.commit(dest)
        with self.assertRaises(RuntimeError):
            verify.sync_linux(dest, source, verify.clean_head(source))
        self.assertEqual(verify.clean_head(dest), old)
        (source / 'unknown.txt').write_text('dirty source', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'dirty'):
            verify.inventory(source)

    def test_wrong_origin_and_ignored_collision_do_not_overwrite(self):
        source = self.repository()
        dest = self.clone(source)
        head = verify.clean_head(source)
        verify.git(dest, 'remote', 'set-url', 'origin', str(self.base / 'other'))
        with self.assertRaisesRegex(ValueError, 'origin'):
            verify.sync_linux(dest, source, head)
        verify.git(dest, 'remote', 'set-url', 'origin', str(source))
        (dest / 'tests/fixtures').mkdir()
        local = dest / 'tests/fixtures/local.txt'
        local.write_text('local content', encoding='utf-8')
        (source / 'tests/fixtures').mkdir()
        (source / 'tests/fixtures/local.txt').write_text('tracked content', encoding='utf-8')
        verify.git(source, 'add', '-f', 'tests/fixtures/local.txt')
        new = self.commit(source)
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            verify.sync_linux(dest, source, new)
        self.assertEqual(local.read_text(), 'local content')
        self.assertEqual(verify.clean_head(dest), head)

    def test_ignored_parent_file_cannot_be_replaced_by_directory(self):
        source = self.repository()
        with (source / '.gitignore').open('a') as output:
            output.write('newdir\n')
        self.commit(source)
        dest = self.clone(source)
        (dest / 'newdir').write_text('keep local parent')
        (source / 'newdir').mkdir()
        (source / 'newdir/child.txt').write_text('new tracked file')
        verify.git(source, 'add', '-f', 'newdir/child.txt')
        head = self.commit(source)
        with self.assertRaisesRegex(ValueError, 'parent path'):
            verify.sync_linux(dest, source, head)
        self.assertEqual((dest / 'newdir').read_text(), 'keep local parent')

    def test_ignored_inputs_text_eol_and_binary_identity(self):
        source = self.repository()
        dest = self.clone(source)
        for root, eol in ((source, b'\r\n'), (dest, b'\n')):
            (root / 'tests/fixtures').mkdir()
            (root / 'tests/fixtures/local.txt').write_bytes(b'line' + eol)
        verify.equal_inputs(verify.inventory(source), verify.inventory(dest))
        (source / 'tests/fixtures/local.bin').write_bytes(b'line\r\n')
        (dest / 'tests/fixtures/local.bin').write_bytes(b'line\n')
        with self.assertRaisesRegex(ValueError, 'mismatch'):
            verify.equal_inputs(verify.inventory(source), verify.inventory(dest))

    def test_symlink_input_rejected(self):
        root = self.repository()
        (root / 'tests/fixtures').mkdir()
        try:
            (root / 'tests/fixtures/local-link').symlink_to(root / '.gitignore')
        except OSError:
            self.skipTest('symlink creation unavailable on this host')
        with self.assertRaisesRegex(ValueError, 'symlink'):
            verify.inventory(root)

    def test_native_coverage_is_not_just_an_exit_code(self):
        root = self.repository()
        data = native_report()
        self.assertTrue(verify.native_coverage(data, root, 'address-undefined')['complete'])
        changes = [dict(scope='focused'), dict(selection=[]), dict(commands=[]),
                   dict(commands=data['commands'] * 2), dict(state='interrupted'),
                   dict(metadata={'sanitizer': 'none'}), dict(commands=[record(['/tmp/test_one'], 1)])]
        for change in changes:
            with self.subTest(change=change):
                bad = {**data, **change}
                self.assertFalse(verify.native_coverage(bad, root, 'address-undefined')['complete'])

    def test_full_gate_missing_stage_and_known_failures(self):
        data = windows_report()
        self.assertTrue(verify.windows_coverage(data)['complete'])
        for index in (0, 1, -1, -6):
            bad = copy.deepcopy(data)
            bad['commands'].pop(index)
            self.assertFalse(verify.windows_coverage(bad)['complete'])
        bad = copy.deepcopy(data)
        bad['commands'][-6] = copy.deepcopy(bad['commands'][-7])
        self.assertFalse(verify.windows_coverage(bad)['complete'])
        failures = ['0026b', '0027c', 'pre_0024ac']
        for row in data['commands']:
            if any(part in ' '.join(row['command']) for part in failures):
                row['returncode'] = 1
        data['returncode'] = 1
        coverage = verify.windows_coverage(data)
        self.assertTrue(coverage['complete'])
        self.assertEqual(len(coverage['failed_checks']), 3)
        root = self.repository()
        with contextlib.redirect_stdout(io.StringIO()):
            flow = verify.Workflow(root, 'g++')
            flow.stages['windows-full'] = {'exit': 1, 'coverage': coverage}
            self.assertEqual(flow.finish(), 1)
        data['returncode'] = 0
        self.assertFalse(verify.windows_coverage(data)['complete'])

    def test_nested_evidence_outputs_and_provenance(self):
        root = self.repository()
        run = root / 'build/child'
        run.mkdir(parents=True)
        verify.write_json(run / 'metrics.json', {'metric': 1})
        verify.write_json(run / 'probe.json', {'probe': True})
        verify.write_json(run / 'summary.json', {'commands': [record(['python', '--output', str(run / 'metrics.json')])]})
        (run / '0001.log').write_text(f'report={run / "probe.json"}\n# report={run / "summary.json"}\n', encoding='utf-8')
        evidence = verify.Evidence(root, root / 'build/copies')
        evidence.scan(f'# report={run / "summary.json"}')
        evidence.save()
        self.assertEqual(len(evidence.records), 4)
        for item in evidence.records:
            self.assertEqual(hashlib.sha256((evidence.destination / item['copy']).read_bytes()).hexdigest(), item['sha256'])
        with self.assertRaisesRegex(ValueError, 'unsafe'):
            evidence.file(root / '.gitignore')
        (run / '0001.log').unlink()
        fresh = verify.Evidence(root, root / 'build/new-copies')
        with self.assertRaisesRegex(ValueError, 'command log'):
            fresh.file(run / 'summary.json')

    def test_transfer_checks_hash_completeness_and_traversal(self):
        dest = self.base / 'linux-origin'
        dest.mkdir()
        data = dest / 'proof.json'
        verify.write_json(data, {'proof': True})
        item = {'copy': 'proof.json', 'sha256': hashlib.sha256(data.read_bytes()).hexdigest()}
        def manifest(items):
            verify.write_json(dest / 'transfer.json', {'complete': True, 'files': items})
        manifest([item])
        verify.verify_transfer(dest)
        for items in ([item, item], [{**item, 'copy': '../outside.json'}], [{**item, 'sha256': '0' * 64}]):
            manifest(items)
            with self.assertRaises(ValueError):
                verify.verify_transfer(dest)
        manifest([item])
        (dest / 'extra.log').write_text('extra')
        with self.assertRaisesRegex(ValueError, 'all copied'):
            verify.verify_transfer(dest)

    def test_linux_continues_after_sanitizer_failure_and_checks_source(self):
        root = self.repository()
        baseline = self.base / 'baseline.json'
        verify.write_json(baseline, verify.inventory(root))
        seen = []
        def gate(flow, label, argv, timeout, sanitizer):
            seen.append((label, argv))
            flow.stages[label] = {'exit': 1 if label == 'address-undefined' else 0}
        with (contextlib.redirect_stdout(io.StringIO()),
              mock.patch.object(verify.Workflow, 'command', return_value=(0, [])),
              mock.patch.object(verify.Workflow, 'gate', gate)):
            code, folder = verify.run_linux(root, baseline, 60)
        self.assertEqual(code, 1)
        self.assertEqual([s[0] for s in seen], ['address-undefined', 'leak'])
        self.assertTrue((folder / 'source-after.json').is_file())
        for _, argv in seen:
            self.assertIn('--build-timeout-s', argv)
            self.assertNotIn('--test', argv)
        def mutate(flow, *args):
            (root / 'new.txt').write_text('modified during run')
        with (contextlib.redirect_stdout(io.StringIO()),
              mock.patch.object(verify.Workflow, 'command', return_value=(0, [])),
              mock.patch.object(verify.Workflow, 'gate', mutate)):
            code, folder = verify.run_linux(root, baseline, 60)
        self.assertNotEqual(code, 0)
        self.assertIn('dirty checkout', verify.read_json(folder / 'verification.json')['stages']['blocker']['reason'])

    def test_stage_export_uses_only_current_sanitizer_report(self):
        root = self.repository()
        calls = []
        with contextlib.redirect_stdout(io.StringIO()):
            flow = verify.Workflow(root, 'g++')
        def command(label, argv, timeout):
            calls.append((label, argv))
            flow.stages[label] = {'exit': 0}
            if label.endswith('-accuracy-export'):
                return 0, []
            path = root / 'build' / label / 'summary.json'
            verify.write_json(path, native_report(label))
            return 0, [path, path]  # Runners print the same path at start and finish.
        with mock.patch.object(flow, 'command', side_effect=command):
            for mode in ('address-undefined', 'leak'):
                flow.gate(mode, ['fixture'], 60, mode)
        exports = [(label, argv[argv.index('--report') + 1]) for label, argv in calls if '--report' in argv]
        self.assertEqual(len(exports), 2)
        self.assertIn('/leak/', exports[1][1].replace('\\', '/'))

    def test_ambiguous_summary_and_missing_doctor_are_not_pass(self):
        root = self.repository()
        with contextlib.redirect_stdout(io.StringIO()):
            flow = verify.Workflow(root, 'g++')
            code, _ = flow.command('linux-doctor', [sys.executable, '-c', 'print("no report")'], 10)
        self.assertEqual(code, 2)
        paths = [root / 'build' / name / 'summary.json' for name in ('one', 'two')]
        for path in paths:
            verify.write_json(path, native_report())
        def command(*args):
            flow.stages['asan'] = {'exit': 0}
            return 0, paths
        with mock.patch.object(flow, 'command', side_effect=command):
            flow.gate('asan', ['fixture'], 60, 'address-undefined')
        self.assertFalse(flow.stages['asan']['coverage']['complete'])

    def test_windows_failure_still_runs_linux_and_packages_evidence(self):
        root = self.repository()
        labels = []
        args = mock.Mock(cxx='g++', pio_bin='pio', wsl_root='/home/ubuntu/src/tracker',
                         wsl_distro='Ubuntu', wsl_user='ubuntu', stage_timeout_s=60)
        def command(flow, label, argv, timeout):
            labels.append(label)
            flow.stages[label] = {'exit': 0}
            if label == 'wsl-suites':
                verify.write_json(flow.folder / 'linux-origin/verification.json',
                                  {'exit': 0, 'stages': {'address-undefined': {'exit': 0}, 'leak': {'exit': 0}}})
            return 0, []
        def gate(flow, label, *args):
            labels.append(label)
            flow.stages[label] = {'exit': 1, 'coverage': {'complete': True}}
        actual_process = verify.run_bounded_process
        def process(argv, **kwargs):
            if argv[0] == 'wsl.exe':
                return subprocess.CompletedProcess(argv, 0, b'/mnt/h/tracker', b'')
            return actual_process(argv, **kwargs)
        with (contextlib.redirect_stdout(io.StringIO()), mock.patch.object(verify, 'ROOT', root),
              mock.patch.object(verify, 'run_bounded_process', side_effect=process),
              mock.patch.object(verify, 'verify_transfer') as transfer,
              mock.patch.object(verify.Workflow, 'command', command),
              mock.patch.object(verify.Workflow, 'gate', gate)):
            self.assertEqual(verify.run_windows(args), 1)
        self.assertEqual(labels, ['wsl-sync', 'windows-doctor', 'windows-full', 'wsl-suites'])
        transfer.assert_called_once()
        self.assertEqual(len(list((root / 'build/gate_runs').glob('dev-verify-*.zip'))), 1)

    def test_wsl_argv_preserves_spaces_and_plan_needs_no_tools(self):
        args = mock.Mock(wsl_distro='Ubuntu Test', wsl_user='ubuntu')
        argv = verify.wsl(args, '/home/ubuntu/path with spaces/python', 'a;literal')
        self.assertEqual(argv[-2:], ['/home/ubuntu/path with spaces/python', 'a;literal'])
        env = {**os.environ, 'PYTHONIOENCODING': 'cp1251'}
        result = run_bounded_process([sys.executable, '-c',
            'import sys; sys.path.insert(0,"tools"); import dev_verify; dev_verify.main(["plan"]); print("Unicode → ✓")'],
            cwd=ROOT, env=env, timeout_s=10, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Unicode → ✓', result.stdout.decode('utf-8'))


if __name__ == '__main__':
    unittest.main()
