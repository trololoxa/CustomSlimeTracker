"""Negative tests for growth, ownership and shared budget checks; no compilation."""
from __future__ import annotations
import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / 'tools'),
               str(Path(__file__).resolve().parents[1] / 'support')]
import contextlib
import io
import json
import os
import subprocess
import unittest
from unittest import mock
import validate_documentation as structure
import validate_test_structure as test_structure
from suite_inventory import check_registry, native_sources, native_test_text
import contract_checks
from contract_checks import compiler, parse_stack_usage, require_limit
from gate_reporting import canonical_check_ids
from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]


class MaintenanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = project_temp_directory(ROOT, 'maintenance-')
        self.root = Path(self.temp.__enter__())
        self.addCleanup(self.temp.__exit__, None, None, None)
        for name in structure.REQUIRED_DOCS:
            self.write('docs/' + name, '# Current contract\n')
        for name in ('README.md', 'AGENTS.md', '.gitignore', '.gitattributes'):
            self.write(name, '')
        self.write('docs/README.md', '\n'.join(f'[{n}]({n})' for n in structure.REQUIRED_DOCS if n != 'README.md'))
        self.write('tools/maintenance_limits.json', (ROOT/'tools/maintenance_limits.json').read_text())
        self.write('tests/tooling/test_behavior.py', 'def test_behavior():\n    pass\n')
        self.write('tools/check_all.py', 'TOOL_CHECKS = (("tests/tooling/test_behavior.py", "behavior"),)\n')
        self.write('tests/native/test_behavior.cpp', '// fixture\n')

    def write(self, path, text):
        p = self.root / path
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding='utf-8')
        return p

    def errors(self):
        return '\n'.join(structure.validate(self.root) + test_structure.validate(self.root))

    def test_current_contract_fixture_and_real_repository(self):
        self.assertEqual(self.errors(), '')
        self.assertEqual(structure.validate(ROOT) + test_structure.validate(ROOT), [])

    def test_patch_report_and_orphan_page_fail(self):
        self.write('docs/development/dev-99_report.md', '# history\n')
        errors = self.errors()
        self.assertIn('patch/run history', errors)
        self.assertIn('missing from documentation index', errors)

    def test_missing_and_ephemeral_links_fail_but_code_examples_do_not(self):
        self.write('docs/status.md', '[missing](absent.md)\n')
        self.assertIn('broken/escaping', self.errors())
        self.write('build/proof.json', '{}')
        self.write('docs/status.md', '[local run](../build/proof.json)\n')
        self.assertIn('ephemeral artifact', self.errors())
        self.write('docs/status.md', '```text\n[example](absent.md)\n```\n')
        self.assertEqual(self.errors(), '')

    def test_document_page_and_total_budgets(self):
        self.write('docs/status.md', '# line\n' * 451)
        self.assertIn('document size budget', self.errors())
        limits = json.loads((self.root/'tools/maintenance_limits.json').read_text())
        limits['documentation']['total_lines'] = 1
        self.write('tools/maintenance_limits.json', json.dumps(limits))
        self.assertIn('documentation growth budget', self.errors())

    def test_registration_and_duplicate_execution_fail(self):
        self.write('tests/tooling/test_new.py', '# unregistered\n')
        self.assertIn('discovery mismatch', self.errors())
        self.write('tools/check_all.py', 'TOOL_CHECKS = (("tests/tooling/test_behavior.py", "a"), ("tests/tooling/test_behavior.py", "b"))')
        self.assertIn('duplicate check ID', self.errors())

    def test_test_location_names_and_helper_copies(self):
        self.write('tools/test_unowned.py', '# wrong owner\n')
        self.write('tests/elsewhere/test_unowned.py', '# missing registry\n')
        self.write('tests/contracts/test_0099_policy.py', 'def compiler():\n    return "g++"\n')
        errors = self.errors()
        self.assertIn('not tools/', errors)
        self.assertIn('unowned Python test location', errors)
        self.assertIn('patch-named test', errors)
        self.assertIn('shared helper copied', errors)

    def test_historical_report_dependency_and_python_size(self):
        self.write('tests/contracts/test_history.py', 'path = "docs/0099_report.md"\n' + '# line\n' * 400)
        self.assertIn('historical report dependency', self.errors())
        self.assertIn('Python test size budget', self.errors())

    def test_native_budget_cannot_be_bypassed_by_rename(self):
        self.write('tests/native/test_behavior.cpp', '// line\n' * 801)
        self.assertIn('native naming/size budget', self.errors())
        self.write('tests/native/test_dev99_behavior.cpp', '// tiny\n')
        self.assertIn('test_dev99_behavior', self.errors())

    def test_duplicate_selectors_fail_even_with_distinct_paths(self):
        entries = (("tests/tooling/test_behavior.py", "a"),
                   ("tests/contracts/test_behavior.py", "b"))
        self.write('tests/contracts/test_behavior.py', '# distinct file\n')
        self.write('tools/check_all.py', 'TOOL_CHECKS = ' + repr(entries))
        self.assertIn('duplicate check ID', self.errors())
        with self.assertRaisesRegex(ValueError, 'duplicate check ID'):
            check_registry(entries)

    def test_nested_native_is_not_silently_omitted(self):
        self.write('tests/native/nested/test_hidden.cpp', '// fixture\n')
        self.assertIn('unowned nested native', self.errors())
        with self.assertRaisesRegex(ValueError, 'unowned nested native'):
            native_sources(self.root/'tests/native')

    def test_support_sources_have_per_file_and_total_budgets(self):
        self.write('tests/native/cases/oversized.hpp', '// line\n' * 801)
        self.write('tests/support/oversized.py', '# line\n' * 401)
        errors = self.errors()
        self.assertIn('native naming/size budget', errors)
        self.assertIn('Python test size budget', errors)
        limits = json.loads((self.root/'tools/maintenance_limits.json').read_text())
        limits['native_tests']['files'] = 1
        limits['python_tests']['files'] = 1
        self.write('tools/maintenance_limits.json', json.dumps(limits))
        self.assertIn('native test growth budget', self.errors())
        self.assertIn('Python test growth budget', self.errors())

    def test_native_case_ownership_cycles_and_escaping_paths(self):
        entry = self.write('tests/native/test_behavior.cpp', '#include "cases/behavior.hpp"\n')
        self.write('tests/native/cases/behavior.hpp', '// evidence\n')
        expanded, headers = native_test_text(entry, self.root/'tests/native')
        self.assertIn('// evidence', expanded)
        self.assertEqual(len(headers), 1)
        self.assertEqual(self.errors(), '')
        self.write('tests/native/cases/orphan.hpp', '// not compiled\n')
        self.assertIn('unowned native case', self.errors())
        self.write('tests/native/cases/behavior.hpp', '#include "cases/behavior.hpp"\n')
        self.assertIn('cyclic native case', self.errors())
        self.write('tests/native/test_behavior.cpp', '#include "cases/../../../README.md"\n')
        self.assertIn('escapes suite', self.errors())

    def test_contract_commands_preserve_failures_environment_and_deadlines(self):
        command = ['compiler', '-O2', '-fstack-usage']
        environment = {'ASAN_OPTIONS': 'detect_leaks=1'}
        failed = subprocess.CompletedProcess(command, 7)
        with mock.patch.object(contract_checks, 'run_bounded_process', return_value=failed) as run:
            with self.assertRaises(subprocess.CalledProcessError) as error:
                contract_checks.run_contract_command(command, cwd=self.root, env=environment, timeout_s=3)
            self.assertEqual(error.exception.returncode, 7)
            run.assert_called_once_with(command, cwd=self.root, env=environment, timeout_s=3,
                                        stdout=None, text=False)
        with mock.patch.object(contract_checks, 'run_bounded_process',
                               side_effect=subprocess.TimeoutExpired(command, 3)) as run:
            with self.assertRaises(subprocess.TimeoutExpired):
                contract_checks.run_contract_command(command, timeout_s=3)
            self.assertEqual(run.call_count, 1)
        self.write('tests/contracts/test_unbounded.py', 'import subprocess\nsubprocess.run(["tool"])\n')
        self.assertIn('unbounded contract subprocess', self.errors())

    def test_shared_source_guards_fail_closed(self):
        self.assertEqual(contract_checks.isolate('begin value end', 'begin', 'end'), 'begin value ')
        self.assertEqual(contract_checks.between('begin value end', 'begin', 'end', 'case'), 'begin value ')
        self.assertEqual(contract_checks.function_body('void f() { if (ok) { pass; } }', 'void f'), ' if (ok) { pass; } ')
        for begin, end in [('absent', 'end'), ('begin', 'absent')]:
            with self.assertRaises(SystemExit):
                contract_checks.isolate('begin value end', begin, end)
        with self.assertRaises(SystemExit):
            contract_checks.function_body('void f() {', 'void f')

    def test_executable_ceiling_and_nonstandard_native_inputs(self):
        limits = json.loads((self.root/'tools/maintenance_limits.json').read_text())
        limits['native_tests']['executables'] = 0
        self.write('tools/maintenance_limits.json', json.dumps(limits))
        self.assertIn('native executable growth budget', self.errors())
        self.write('tests/native/cases/huge.inc', '// line\n' * 801)
        self.assertIn('native naming/size budget', self.errors())
        self.write('tests/native/unowned.txt', 'fixture belongs elsewhere\n')
        self.assertIn('unowned native input type', self.errors())

    def test_renaming_a_duplicate_helper_does_not_hide_it(self):
        body = ''.join(f'    value += {i}\n' for i in range(8)) + '    return value\n'
        self.write('tests/contracts/test_first.py', 'def first(value):\n' + body)
        self.write('tests/contracts/test_second.py', 'def renamed(value):\n' + body)
        self.assertIn('duplicated contract function body', self.errors())

    def test_selector_aliases_are_not_registry_entries(self):
        import check_all
        current = {Path(p).stem for p, _ in check_all.TOOL_CHECKS}
        old = 'test_dev05b_verification_workflow'
        with contextlib.redirect_stderr(io.StringIO()) as output:
            self.assertEqual(canonical_check_ids([old]), ['test_cross_platform_verification'])
        self.assertNotIn(old, current)
        self.assertIn('deprecated selector', output.getvalue())
        aliases = json.loads((ROOT / 'tools/check_aliases.json').read_text())
        native_ids = {p.stem for p in (ROOT / 'tests/native').glob('test_*.cpp')}
        self.assertTrue(set(aliases.values()) <= current | native_ids)
        self.assertFalse(set(aliases) & (current | native_ids))

    def test_stack_parser_missing_function_and_worst_frame(self):
        self.write('stack/one.su', 'file:1:1:owner()\t80\tstatic\nmalformed\n')
        self.write('stack/two.su', 'file:2:2:owner()\t112\tstatic\n')
        usage = parse_stack_usage(self.root/'stack')
        self.assertEqual(usage['owner()'], 112)
        require_limit(usage, 'owner()', 112)
        with self.assertRaises(SystemExit):
            require_limit(usage, 'owner()', 96)
        with self.assertRaises(SystemExit):
            require_limit(usage, 'absent()', 999)

    def test_missing_stack_evidence_survives_temporary_input_cleanup(self):
        source = self.write('stack/frames.su', 'unit.cpp:1:1:owner()\t112\tstatic\nmalformed\n')
        usage = parse_stack_usage(source.parent)
        with mock.patch.object(contract_checks, 'ROOT', self.root):
            with self.assertRaisesRegex(SystemExit, 'stack evidence='):
                require_limit(usage, 'missing()', 96)
        source.unlink()
        manifests = list((self.root/'build/gate_runs').glob('stack-evidence-*/inventory.json'))
        self.assertEqual(len(manifests), 1)
        data = json.loads(manifests[0].read_text())
        self.assertEqual(data['file_count'], 1)
        self.assertFalse(data['truncated'])
        self.assertIn('112', (manifests[0].parent/data['files'][0]['copy']).read_text())
        with (mock.patch.object(contract_checks, 'ROOT', self.root),
              mock.patch.object(Path, 'mkdir', side_effect=PermissionError('injected')),
              self.assertRaisesRegex(SystemExit, 'did not find function: missing.*evidence save failed')):
            require_limit(usage, 'missing()', 96)

    def test_missing_stack_evidence_limits_are_explicit(self):
        paths = tuple(self.write(f'stack/{i}.su', 'x' * 40) for i in range(3))
        usage = contract_checks.StackUsage(paths)
        for byte_limit, file_limit, expected_bytes, expected_files in [(32, 64, 32, 1), (1024, 2, 80, 2)]:
            with (mock.patch.object(contract_checks, 'ROOT', self.root),
                  mock.patch.object(contract_checks, 'STACK_EVIDENCE_BYTES', byte_limit),
                  mock.patch.object(contract_checks, 'STACK_EVIDENCE_FILES', file_limit),
                  self.assertRaises(SystemExit) as failure):
                require_limit(usage, 'missing()', 96)
            path = Path(str(failure.exception).split('stack evidence=', 1)[1])
            data = json.loads(path.read_text())
            self.assertTrue(data['truncated'])
            self.assertEqual(len(data['files']), expected_files)
            self.assertEqual(sum(item['bytes'] for item in data['files']), expected_bytes)

    def test_no_stack_evidence_on_pass_or_actual_budget_excess(self):
        self.write('stack/frame.su', 'file:1:1:owner()\t112\tstatic\n')
        usage = parse_stack_usage(self.root/'stack')
        with mock.patch.object(contract_checks.tempfile, 'mkdtemp') as create:
            require_limit(usage, 'owner()', 112)
            with self.assertRaisesRegex(SystemExit, '112 bytes, limit 96'):
                require_limit(usage, 'owner()', 96)
        create.assert_not_called()

    def test_invalid_explicit_compiler_never_falls_back(self):
        with mock.patch.dict(os.environ, {'CXX': str(self.root/'missing-compiler')}):
            with self.assertRaises(RuntimeError):
                compiler()


if __name__ == '__main__':
    unittest.main()
