#!/usr/bin/env python3
"""Test ownership, complete source inventory and size budgets; no test execution."""
from __future__ import annotations
import ast
import json
from pathlib import Path
import sys
from suite_inventory import check_registry, native_sources, native_test_text
from validate_documentation import PATCH_NAME

ROOT = Path(__file__).resolve().parents[1]


def registered_tests(root):
    tree = ast.parse((root / 'tools/check_all.py').read_text(encoding='utf-8'))
    entries = []
    tests = None
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = {t.id for t in node.targets if isinstance(t, ast.Name)}
            if names & {'TOOL_CHECKS', 'CONTRACT_CHECKS'}:
                value = ast.literal_eval(node.value)
                entries.extend(value)
                if 'TOOL_CHECKS' in names:
                    tests = [entry[0] for entry in value]
    if tests is None:
        raise ValueError('missing TOOL_CHECKS registry')
    check_registry(entries)
    return tests


def validate(root=ROOT):
    root = root.resolve()
    limits = json.loads((root / 'tools/maintenance_limits.json').read_text(encoding='utf-8'))
    errors = []
    tests = sorted((root / 'tests').glob('tooling/test_*.py')) + sorted((root / 'tests').glob('contracts/test_*.py'))
    misplaced = set((root / 'tests').rglob('test_*.py')) - set(tests)
    if misplaced:
        errors.append(f'unowned Python test location: {sorted(str(p.relative_to(root)) for p in misplaced)}')
    try:
        registered = registered_tests(root)
    except ValueError as exc:
        errors.append(str(exc))
        registered = []
    expected = {p.relative_to(root).as_posix() for p in tests}
    if len(registered) != len(set(registered)) or set(registered) != expected:
        errors.append(f'Python test discovery mismatch: missing={sorted(expected-set(registered))}, stale={sorted(set(registered)-expected)} or duplicate entries')
    if list((root / 'tools').glob('test_*.py')):
        errors.append('test suites belong in tests/, not tools/')
    total_python = sum(len(p.read_text(encoding='utf-8').splitlines()) for p in (root / 'tests').rglob('*.py'))
    python = sorted((root / 'tests').rglob('*.py'))
    helper_bodies = {}
    for p in python:
        if p.parent not in [root / 'tests' / owner for owner in ('contracts', 'tooling', 'support')]:
            errors.append(f'unowned Python support location: {p.relative_to(root)}')
        if PATCH_NAME.search(p.name):
            errors.append(f'patch-named test: {p.relative_to(root)}')
        if len(p.read_text(encoding='utf-8').splitlines()) > limits['python_tests']['file_lines']:
            errors.append(f'Python test size budget: {p.relative_to(root)}')
        tree = ast.parse(p.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if (p.parent.name == 'contracts' and isinstance(node, ast.Call)
                    and ast.unparse(node.func) in {'subprocess.run', 'subprocess.check_output',
                        'subprocess.call', 'subprocess.check_call', 'subprocess.Popen'}):
                errors.append(f'unbounded contract subprocess: {p.relative_to(root)}')
            if p.parent.name == 'contracts' and isinstance(node, ast.Constant) and isinstance(node.value, str) and node.value.startswith('docs/') and node.value.endswith('_report.md'):
                errors.append(f'historical report dependency in test: {p.relative_to(root)}')
        if p.parent.name == 'contracts':
            duplicated = {n.name for n in tree.body if isinstance(n, ast.FunctionDef)} & {'compiler','parse_stack_usage','require_limit','require','forbid','stack_usage','isolate','between','function_body','compile_run'}
            for node in tree.body:
                if isinstance(node, ast.FunctionDef) and node.end_lineno - node.lineno >= 7:
                    body = ast.dump(ast.Module(body=node.body, type_ignores=[]), include_attributes=False)
                    previous = helper_bodies.setdefault(body, (p.name, node.name))
                    if previous[0] != p.name:
                        errors.append(f'duplicated contract function body: {previous} and {(p.name, node.name)}')
            if duplicated:
                errors.append(f'shared helper copied into {p.name}: {sorted(duplicated)}')
    if len(python) > limits['python_tests']['files'] or total_python > limits['python_tests']['total_lines']:
        errors.append(f'Python test growth budget exceeded: files={len(python)} lines={total_python}')
    try:
        entries = native_sources(root / 'tests/native')
        if len(entries) > limits['native_tests']['executables']:
            errors.append(f'native executable growth budget: {len(entries)}')
        owned = set()
        for entry in entries:
            _, headers = native_test_text(entry, root / 'tests/native')
            owned.update(headers)
        cases = {p.resolve() for p in (root / 'tests/native/cases').rglob('*') if p.is_file()}
        if cases - owned:
            errors.append('unowned native case headers: ' + ', '.join(str(p.relative_to(root)) for p in sorted(cases - owned)))
    except (ValueError, OSError) as exc:
        errors.append(str(exc))
    native = sorted(p for p in (root / 'tests/native').rglob('*') if p.is_file())
    total_native = 0
    for p in native:
        if p.suffix not in ('.cpp', '.hpp', '.h', '.cc', '.cxx', '.c', '.inc', '.inl', '.tpp', '.S'):
            errors.append(f'unowned native input type: {p.relative_to(root)}; data fixtures belong in tests/fixtures')
            continue
        count = len(p.read_text(encoding='utf-8').splitlines()); total_native += count
        ceiling = limits['native_tests']['file_lines']
        if PATCH_NAME.search(p.name) or count > ceiling:
            errors.append(f'native naming/size budget: {p.relative_to(root)}: {count} / {ceiling}')
    if len(native) > limits['native_tests']['files'] or total_native > limits['native_tests']['total_lines']:
        errors.append(f'native test growth budget exceeded: files={len(native)} lines={total_native}')
    return errors


def main():
    errors = validate()
    for error in errors:
        print(error, file=sys.stderr)
    print(f'# test structure: {"FAIL" if errors else "PASS"}')
    return int(bool(errors))


if __name__ == '__main__':
    raise SystemExit(main())
