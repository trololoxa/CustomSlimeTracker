#!/usr/bin/env python3
"""One local Windows/WSL verification workflow, using the existing gate runners.

Requires committed clean checkouts. Never commits, resets, installs, escalates,
changes test budgets, or turns a failed firmware policy into a pass.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import zipfile

from gate_reporting import GateReport, replace_report
from quality_gate_runtime import quality_gate_environment, run_bounded_process
from run_standalone_tests import native_runner_lock

ROOT = Path(__file__).resolve().parents[1]
INPUT_DIRS = ('src', 'include', 'lib', 'test', 'tests', 'tools', 'partitions')
TEXT_SUFFIXES = {'.cpp', '.hpp', '.h', '.c', '.s', '.py', '.md', '.txt', '.ini',
                 '.json', '.csv', '.log', '.yaml', '.yml', '.toml', '.sh', '.ps1'}
MAX_FILE_BYTES = 64 * 1024 * 1024


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    replace_report(temporary, path)


def read_json(path: Path):
    if path.stat().st_size > MAX_FILE_BYTES:
        raise ValueError(f'oversized JSON: {path}')
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'expected JSON object: {path}')
    return data


def git(root: Path, *args: str) -> str:
    result = run_bounded_process(['git', '-C', str(root), *args], timeout_s=60,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        raise RuntimeError(f'git {args[0]} failed: {result.stderr.decode("utf-8", errors="replace").strip()}')
    return result.stdout.decode('utf-8')


def clean_head(root: Path) -> str:
    if Path(git(root, 'rev-parse', '--show-toplevel').strip()).resolve() != root.resolve():
        raise ValueError('expected repository root, not a nested directory')
    head = git(root, 'rev-parse', 'HEAD').strip()
    if not re.fullmatch(r'[0-9a-f]{40}|[0-9a-f]{64}', head):
        raise ValueError('invalid HEAD')
    if git(root, 'status', '--porcelain=v1', '--untracked-files=all').strip():
        raise ValueError(f'dirty checkout: {root}; review/commit intended changes first; nothing discarded')
    return head


def inventory(root: Path) -> dict:
    root = root.resolve()
    head = clean_head(root)
    entries = git(root, 'ls-files', '--stage', '-z').split('\0')
    names = set()
    for entry in filter(None, entries):
        metadata, name = entry.split('\t', 1)
        if metadata.split()[0] in ('120000', '160000'):
            raise ValueError(f'symlink/submodule needs explicit input handling: {name}')
        names.add(name)
    for folder in INPUT_DIRS:
        for path in (root / folder).rglob('*'):
            if '__pycache__' not in path.parts and path.suffix not in ('.pyc', '.pyo'):
                if path.is_symlink():
                    raise ValueError(f'symlink input: {path}')
                if path.is_file():
                    names.add(path.relative_to(root).as_posix())
    files = {}
    for name in sorted(names):
        path = root / name
        if not path.resolve().is_relative_to(root) or path.is_symlink():
            raise ValueError(f'input escapes repository: {name}')
        if path.stat().st_size > MAX_FILE_BYTES:
            raise ValueError(f'input exceeds inventory limit: {name}')
        raw = path.read_bytes()
        normalized = raw
        kind = 'binary'
        if path.suffix.lower() in TEXT_SUFFIXES or path.name in ('.gitignore', '.gitattributes'):
            try:
                raw.decode('utf-8')
                if b'\0' not in raw:
                    normalized = raw.replace(b'\r\n', b'\n')
                    kind = 'utf8-text'
            except UnicodeDecodeError:
                pass
        files[name] = {'sha256': hashlib.sha256(normalized).hexdigest(),
                       'raw_sha256': hashlib.sha256(raw).hexdigest(), 'kind': kind}
    digest = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    return {'head': head, 'root': str(root), 'files': files, 'manifest_sha256': digest,
            'scope': 'tracked files and ignored inputs in named source directories; '
                     'no external SDK/toolchain certification'}


def equal_inputs(before: dict, after: dict) -> None:
    differences = [name for name in sorted(set(before['files']) | set(after['files']))
        if before['files'].get(name, {}).get('sha256') != after['files'].get(name, {}).get('sha256')]
    if before['head'] != after['head'] or differences:
        raise ValueError(f'source identity mismatch; changed inputs: {differences[:12]}')


def sync_linux(root: Path, source: Path, head: str) -> dict:
    if not re.fullmatch(r'[0-9a-f]{40}|[0-9a-f]{64}', head):
        raise ValueError('invalid source commit')
    old = inventory(root)['head']  # Reject existing input symlinks before mutation.
    if clean_head(source) != head:
        raise ValueError('Windows source changed before sync')
    origin = git(root, 'remote', 'get-url', 'origin').strip()
    if not Path(origin).is_absolute() or Path(origin).resolve() != source.resolve():
        raise ValueError('unexpected WSL origin; remote configuration was not changed')
    git(root, 'fetch', '--no-tags', 'origin', head)
    git(root, 'merge-base', '--is-ancestor', old, head)  # Divergence is a blocker, never reset.
    old_names = set(git(root, 'ls-files', '-z').split('\0'))
    for name in filter(None, git(root, 'ls-tree', '-r', '--name-only', '-z', head).split('\0')):
        if name not in old_names:
            path = root / name
            if path.exists() or path.is_symlink():
                raise ValueError(f'fast-forward would overwrite an ignored/local path: {name}')
            for parent in path.parents:
                if parent == root:
                    break
                if parent.is_symlink() or (parent.exists() and not parent.is_dir()):
                    raise ValueError(f'fast-forward would replace a local parent path: {parent}')
    git(root, 'merge', '--ff-only', '--no-edit', head)
    if clean_head(root) != head:
        raise ValueError('fast-forward did not reach requested commit')
    expected, actual = inventory(source), inventory(root)
    equal_inputs(expected, actual)
    return actual


def environment(root: Path, cxx: str) -> dict:
    env = quality_gate_environment(root, scope='dev-verify')
    env.update(PYTHONNOUSERSITE='1', PYTHONIOENCODING='utf-8', CXX=cxx)
    if os.name == 'nt':
        python_dir = Path(sys.executable).parent
        paths = [python_dir, python_dir / 'Library/bin', python_dir / 'Scripts', Path(cxx).parent]
        env['PATH'] = os.pathsep.join(map(str, paths)) + os.pathsep + env.get('PATH', '')
    return env


class Evidence:
    """Copy only reports/logs/manifests reached from this run, with provenance."""
    def __init__(self, root: Path, destination: Path):
        self.root, self.destination = root.resolve(), destination
        self.seen: set[Path] = set()
        self.records: list[dict] = []
        self.total_bytes = 0

    def file(self, path: Path) -> None:
        if not path.is_absolute():
            path = self.root / path
        if path.is_symlink() or not path.resolve().is_relative_to(self.root / 'build'):
            raise ValueError(f'unsafe evidence path: {path}')
        path = path.resolve()
        if path in self.seen:
            return
        if path.suffix not in ('.json', '.log') or path.stat().st_size > MAX_FILE_BYTES:
            raise ValueError(f'unsupported/oversized evidence: {path}')
        self.total_bytes += path.stat().st_size
        if len(self.seen) >= 10000 or self.total_bytes > 512 * 1024 * 1024:
            raise ValueError('evidence collection exceeds bounded file/byte budget')
        self.seen.add(path)
        relative = path.relative_to(self.root / 'build')
        target = self.destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        raw = path.read_bytes()
        target.write_bytes(raw)
        digest = hashlib.sha256(raw).hexdigest()
        if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            raise OSError('evidence copy hash mismatch')
        self.records.append({'origin': str(path), 'copy': relative.as_posix(), 'sha256': digest})
        if path.name == 'summary.json':
            for record in read_json(path).get('commands', []):
                log_name = record.get('log', '')
                if not re.fullmatch(r'[0-9]+\.log', log_name) or not (path.parent / log_name).is_file():
                    raise ValueError(f'missing/invalid command log in {path}: {log_name}')
                self.file(path.parent / log_name)
                argv = record.get('command', [])
                if record.get('returncode') == 0 and '--output' in argv and argv.index('--output') + 1 < len(argv):
                    output = Path(argv[argv.index('--output') + 1])
                    output = output if output.is_absolute() else self.root / output
                    if output.suffix == '.json' and output.is_file():
                        self.file(output)
            for log in sorted(path.parent.glob('*.log')):
                self.file(log)
        if path.suffix == '.log':
            self.scan(raw.decode('utf-8', errors='replace'))

    def scan(self, text: str) -> list[Path]:
        reports = []
        for line in text.splitlines():
            match = re.search(r'(?:\breport=|^# MANIFEST )(.+\.(?:json))$', line.strip())
            if match:
                path = Path(match.group(1))
                if not path.is_absolute():
                    path = self.root / path
                # Existing files only; failures can mention reports never created.
                if path.is_file():
                    self.file(path)
                    reports.append(path)
        return reports

    def save(self) -> None:
        write_json(self.destination / 'provenance.json', {'root': str(self.root), 'files': self.records})


def native_coverage(data: dict, root: Path, sanitizer: str) -> dict:
    expected = sorted(p.stem for p in (root / 'tests/native').glob('test_*.cpp'))
    commands = data.get('commands', [])
    executions = [r for r in commands if len(r.get('command', [])) == 1]
    actual = [Path(r['command'][0].replace('\\', '/')).name.removesuffix('.exe') for r in executions]
    complete = (bool(expected) and data.get('schema') == 'tracker-gate-run-v1'
        and type(data.get('returncode')) is int and data.get('runner') == 'native' and data.get('scope') == 'full-native'
        and sorted(data.get('selection', [])) == expected and sorted(actual) == expected
        and data.get('metadata', {}).get('sanitizer') == sanitizer
        and data.get('state') == 'completed'
        and all(r.get('state') == 'completed' for r in commands)
        and (data.get('returncode') != 0 or all(r.get('returncode') == 0 for r in commands)))
    return {'complete': complete, 'expected': len(expected), 'executed': len(actual),
            'passed': sum(r.get('returncode') == 0 for r in executions),
            'exit': data.get('returncode')}


def windows_coverage(data: dict) -> dict:
    import check_all
    commands = data.get('commands', [])
    expected = [p for p, _ in (*check_all.CONTRACT_CHECKS, *check_all.TOOL_CHECKS)]
    scripts = [r['command'][1] for r in commands if len(r.get('command', [])) > 1
               and r['command'][1] in expected]
    profiles = [r for r in commands if r.get('command', [])[1:3] == ['run', '-e']]
    replay = [r for r in commands if len(r.get('command', [])) > 1 and r['command'][1].startswith('tools/replay/')]
    replay_roles = []
    for row in replay:
        argv = row['command']
        if argv[1] == 'tools/replay/compare_replay_metrics.py':
            replay_roles.append('compare')
        elif argv[1] != 'tools/replay/replay_machine_log.py':
            replay_roles.append('unknown')
        elif '--require-magr' in argv:
            replay_roles.append('magr')
        elif '--min-duration-s' in argv and argv.index('--min-duration-s') + 1 < len(argv):
            replay_roles.append(argv[argv.index('--min-duration-s') + 1])
        else:
            replay_roles.append('metrics')
    native = [r for r in commands if len(r.get('command', [])) > 1 and r['command'][1] == 'tools/run_standalone_tests.py']
    complete = (data.get('schema') == 'tracker-gate-run-v1' and type(data.get('returncode')) is int
        and data.get('scope') == 'default' and data.get('runner') == 'check-all'
        and scripts == expected and len(native) == 1 and replay_roles == ['60', 'metrics', 'magr', 'compare', '600']
        and [r['command'][3] for r in profiles] == [*check_all.REQUIRED_PIO_ENVS, check_all.DEBUG_LINKCHECK_ENV, check_all.DEBUG_ENV]
        and data.get('state') == 'completed' and all(r.get('state') == 'completed' for r in commands)
        and (data.get('returncode') != 0 or all(r.get('returncode') == 0 for r in commands)))
    return {'complete': complete, 'exit': data.get('returncode'),
            'profiles': [{'name': r['command'][3], 'exit': r['returncode']} for r in profiles],
            'failed_checks': [{'command': r['command'], 'exit': r['returncode'], 'log': r['log']}
                              for r in commands if r.get('returncode') != 0]}


class Workflow:
    def __init__(self, root: Path, cxx: str):
        self.root, self.env = root, environment(root, cxx)
        self.report = GateReport(root, 'dev-verify')
        self.report.configure('local verification orchestration', [])
        self.folder = self.report.directory
        self.evidence = Evidence(root, self.folder / 'origin')
        self.stages: dict = {}

    def command(self, label: str, argv: list[str], timeout: float) -> tuple[int, list[Path]]:
        print(f'# STAGE {label}', flush=True)
        result = self.report.run(argv, cwd=self.root, timeout_s=timeout, env=self.env)
        log = self.folder / self.report.data['commands'][-1]['log']
        paths = self.evidence.scan(log.read_text(encoding='utf-8', errors='replace'))
        self.stages[label] = {'exit': result.returncode, 'log': log.name}
        self.evidence.save()
        code = result.returncode
        if label in ('windows-doctor', 'linux-doctor'):
            profile = 'firmware' if label == 'windows-doctor' else 'sanitized'
            doctors = [read_json(p) for p in dict.fromkeys(paths)
                       if read_json(p).get('schema') == 'tracker-doctor-v1']
            valid = (len(doctors) == 1 and doctors[0].get('profile') == profile
                     and doctors[0].get('failed') is (code != 0))
            if not valid:
                self.stages[label]['coverage'] = {'complete': False, 'reason': 'missing/inconsistent doctor report'}
                code = 2
        return code, paths

    def gate(self, label: str, argv: list[str], timeout: float, sanitizer: str | None = None) -> None:
        code, paths = self.command(label, argv, timeout)
        runner = 'native' if sanitizer else 'check-all'
        summaries = list(dict.fromkeys(p for p in paths if p.name == 'summary.json' and read_json(p).get('runner') == runner))
        if len(summaries) != 1:
            self.stages[label]['coverage'] = {'complete': False, 'reason': 'missing/ambiguous summary'}
            return
        data = read_json(summaries[-1])
        coverage = native_coverage(data, self.root, sanitizer) if sanitizer else windows_coverage(data)
        coverage['complete'] &= data.get('returncode') == code
        self.stages[label]['coverage'] = coverage
        native_reports = [summaries[-1]] if sanitizer else []
        if not sanitizer:
            child_commands = [r for r in data['commands'] if r.get('command', [])[1:2] == ['tools/run_standalone_tests.py']]
            if len(child_commands) == 1 and re.fullmatch(r'[0-9]+\.log', child_commands[0]['log']):
                child_log = summaries[-1].parent / child_commands[0]['log']
                native_reports = list(dict.fromkeys(p for p in self.evidence.scan(child_log.read_text(encoding='utf-8', errors='replace'))
                    if p.name == 'summary.json' and read_json(p).get('runner') == 'native'))
            self.stages[label]['native'] = [native_coverage(read_json(p), self.root, 'none') for p in native_reports]
            coverage['complete'] &= len(native_reports) == 1 and self.stages[label]['native'][0]['complete']
        for p in native_reports:
            if read_json(p).get('returncode') == 0:
                # Export packages evidence; known OPEN contracts are not a successful roadmap gate.
                self.command(label + '-accuracy-export', [sys.executable, 'tools/replay/algorithm_accuracy.py',
                    'export', '--report', str(p), '--label', label, '--output', str(self.folder / (label + '-accuracy.json'))], 60)

    def finish(self, error: str | None = None, *, interrupted: bool = False) -> int:
        if error:
            self.stages['blocker'] = {'exit': 2, 'reason': error}
        code = int(any(s.get('exit') != 0 or not s.get('coverage', {}).get('complete', True)
                       for s in self.stages.values()))
        if interrupted:
            code = 130
        self.evidence.save()
        self.report.finish(code, interrupted=interrupted)
        data = {'schema': 'tracker-local-verification-v1', 'stages': self.stages,
                'exit': code, 'source': self.report.data.get('source'),
                'limitations': ['Not release/HIL acceptance; no automatic pending removal',
                                'Known stack policies remain failures; firmware work deferred',
                                'No hostile-source or external SDK content certification']}
        write_json(self.folder / 'verification.json', data)
        lines = ['# Local verification', '', f'Overall exit: **{code}**. Scope: local host/build evidence.',
                 f'Source commit: `{(data["source"] or {}).get("commit")}`.', '',
                 '| Stage | Exit | Coverage complete |', '| --- | --- | --- |']
        rows = [(name, stage) for name, stage in self.stages.items()]
        rows += [(f'wsl/{name}', stage) for name, stage in self.stages.get('wsl-suites', {}).get('details', {}).items()]
        for name, stage in rows:
            complete = stage.get('coverage', {}).get('complete', 'n/a')
            lines.append(f'| {name} | {stage.get("exit")} | {complete} |')
            print(f'# RESULT {name}: exit={stage.get("exit")} coverage={complete}', flush=True)
        lines += ['', 'See verification.json and raw logs. This file does not close hardware, release,',
                  'Server, real accuracy or known OPEN AHRS/mag requirements.',
                  'Update only documentation claims established by these reports; preserve historical failures.', '']
        (self.folder / 'verification.md').write_text('\n'.join(lines), encoding='utf-8')
        print(f'# verification={self.folder / "verification.json"}', flush=True)
        return code


def run_linux(root: Path, baseline: Path, timeout: float) -> tuple[int, Path]:
    with native_runner_lock(root / 'build/dev-verify.lock'):
        flow = Workflow(root, '/usr/bin/g++')
        try:
            before = inventory(root)
            equal_inputs(read_json(baseline), before)
            write_json(flow.folder / 'source-before.json', before)
            code, _ = flow.command('linux-doctor', [sys.executable, 'tools/doctor.py', '--profile', 'sanitized', '--cxx', '/usr/bin/g++'], 300)
            if code:
                raise ValueError('Linux sanitizer doctor failed; no unsanitized fallback')
            for mode in ('address-undefined', 'leak'):
                flow.gate(mode, [sys.executable, 'tools/run_standalone_tests.py', '--cxx', '/usr/bin/g++',
                    '--sanitizer', mode, '--build-timeout-s', '600', '--test-timeout-s', '180'], timeout, mode)
            after = inventory(root)
            write_json(flow.folder / 'source-after.json', after)
            equal_inputs(before, after)
            code = flow.finish()
        except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
            code = flow.finish(str(exc))
        except KeyboardInterrupt:
            code = flow.finish('interrupted; no successful completion inferred', interrupted=True)
        return code, flow.folder


def wsl(args, *command: str) -> list[str]:
    return ['wsl.exe', '--distribution', args.wsl_distro, '--user', args.wsl_user, '--exec', *command]


def run_windows(args) -> int:
    with native_runner_lock(ROOT / 'build/dev-verify.lock'):
        flow = Workflow(ROOT, args.cxx)
        try:
            before = inventory(ROOT)
            write_json(flow.folder / 'source-before.json', before)
            # Use wslpath rather than guessing drive mount locations or shell quoting.
            converted = run_bounded_process(wsl(args, 'wslpath', '-a', '-u', str(ROOT)),
                timeout_s=30, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if converted.returncode:
                raise RuntimeError(f'WSL unavailable: {converted.stderr.decode("utf-8", errors="replace").strip()}')
            source = converted.stdout.decode('utf-8').strip()
            script = source + '/tools/dev_verify.py'
            sync = wsl(args, 'python3', script, '_sync', '--root', args.wsl_root,
                       '--source', source, '--head', before['head'])
            sync_code, _ = flow.command('wsl-sync', sync, 300)
            doctor, _ = flow.command('windows-doctor', [sys.executable, 'tools/doctor.py', '--profile', 'firmware',
                '--cxx', args.cxx, '--pio-bin', args.pio_bin], 300)
            if doctor == 0:
                flow.gate('windows-full', [sys.executable, 'tools/check_all.py', '--clean', '--require-pio',
                    '--pio-bin', args.pio_bin], args.stage_timeout_s)
            else:
                flow.stages['windows-full'] = {'exit': None, 'reason': 'blocked by Windows doctor'}
            if sync_code == 0:
                destination = source + '/' + flow.folder.relative_to(ROOT).as_posix() + '/linux-origin'
                baseline = source + '/' + (flow.folder / 'source-before.json').relative_to(ROOT).as_posix()
                code, _ = flow.command('wsl-suites', wsl(args, args.wsl_root + '/.venv-dev-linux/bin/python',
                    args.wsl_root + '/tools/dev_verify.py', '_linux', '--root', args.wsl_root,
                    '--baseline', baseline, '--destination', destination,
                    '--stage-timeout-s', str(args.stage_timeout_s)), 2 * args.stage_timeout_s + 1200)
                linux = flow.folder / 'linux-origin/verification.json'
                if not linux.is_file() or read_json(linux).get('exit') != code:
                    raise ValueError('Linux result/transfer incomplete; no PASS inferred')
                verify_transfer(linux.parent)
                flow.stages['wsl-suites']['details'] = read_json(linux)['stages']
            else:
                flow.stages['wsl-suites'] = {'exit': None, 'reason': 'blocked by sync/source mismatch'}
            after = inventory(ROOT)
            write_json(flow.folder / 'source-after.json', after)
            equal_inputs(before, after)
            code = flow.finish()
        except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
            code = flow.finish(str(exc))
        except KeyboardInterrupt:
            code = flow.finish('interrupted; Linux workers may still hold their lock until their deadline', interrupted=True)
        archive = flow.folder.with_suffix('.zip')
        with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as output:
            for path in sorted(flow.folder.rglob('*')):
                if path.is_file():
                    output.write(path, flow.folder.name + '/' + path.relative_to(flow.folder).as_posix())
        print(f'# archive={archive}', flush=True)
        return code


def verify_transfer(destination: Path) -> None:
    transfer = read_json(destination / 'transfer.json')
    if transfer.get('complete') is not True or not transfer.get('files'):
        raise ValueError('incomplete Linux evidence transfer')
    seen = set()
    for item in transfer['files']:
        name = item['copy']
        original = destination / name
        path = original.resolve()
        if name in seen or not path.is_relative_to(destination.resolve()) or original.is_symlink():
            raise ValueError('unsafe/duplicate transfer entry')
        seen.add(name)
        if path.stat().st_size > MAX_FILE_BYTES or hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('Linux evidence changed after transfer')
    actual = {p.relative_to(destination).as_posix() for p in destination.rglob('*') if p.is_file() and p != destination / 'transfer.json'}
    if actual != seen:
        raise ValueError('Linux transfer manifest does not cover all copied files')


def main(argv=None) -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8', errors='backslashreplace')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('plan', 'run', '_sync', '_linux'))
    parser.add_argument('--cxx', default=os.environ.get('CXX', r'C:\msys64\ucrt64\bin\g++.exe'))
    parser.add_argument('--pio-bin', default=str(Path(sys.executable).parent / 'Scripts/pio.exe'))
    parser.add_argument('--wsl-distro', default='Ubuntu')
    parser.add_argument('--wsl-user', default='ubuntu')
    parser.add_argument('--wsl-root', default='/home/ubuntu/src/SlimeTracker')
    parser.add_argument('--stage-timeout-s', type=float, default=7200)
    parser.add_argument('--root', type=Path)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--head')
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--destination', type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.stage_timeout_s) or args.stage_timeout_s <= 0:
        parser.error('stage timeout must be positive and finite')
    try:
        if args.action == 'plan':
            print('Clean committed Windows/WSL checkouts required. No changes made.\n'
                  '1. Verify source inventory and local-origin fast-forward (no reset/commit/install).\n'
                  '2. Windows doctor + full check_all --clean --require-pio.\n'
                  '3. WSL doctor + full ASan/UBSan, then full LSan (even if Windows gate fails).\n'
                  '4. Verify unchanged inputs, collect hashed evidence + summary + ZIP.\n'
                  'Failures stay failures; no automatic reruns or documentation status editing.')
            return 0
        if args.action == 'run':
            if os.name != 'nt':
                raise ValueError('run is the Windows coordinator; use plan for read-only inspection')
            return run_windows(args)
        if os.name == 'nt' or not args.root or not args.root.is_absolute():
            raise ValueError('internal worker requires Linux and an absolute repository root')
        if args.action == '_sync':
            if not args.source or not args.source.is_absolute() or not args.head:
                raise ValueError('sync needs source and exact commit')
            with native_runner_lock(args.root / 'build/dev-verify.lock'):
                data = sync_linux(args.root, args.source, args.head)
            print(f'SYNC verified head={data["head"]} files={len(data["files"])}')
            return 0
        if not args.baseline or not args.destination or not args.destination.is_absolute():
            raise ValueError('Linux worker needs baseline and absolute evidence destination')
        if args.destination.parent.resolve() != args.baseline.parent.resolve() or args.destination.name != 'linux-origin':
            raise ValueError('Linux evidence must be next to its Windows baseline')
        if args.destination.exists():
            raise ValueError('refusing to overwrite an existing Linux evidence directory')
        code, folder = run_linux(args.root, args.baseline, args.stage_timeout_s)
        args.destination.mkdir(parents=True)
        manifest = []
        for path in sorted(folder.rglob('*')):
            if path.is_file():
                raw = path.read_bytes()
                target = args.destination / path.relative_to(folder)
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(raw)
                digest = hashlib.sha256(raw).hexdigest()
                if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
                    raise OSError('Linux transfer hash mismatch')
                manifest.append({'origin': str(path), 'copy': target.relative_to(args.destination).as_posix(), 'sha256': digest})
        write_json(args.destination / 'transfer.json', {'files': manifest, 'complete': True})
        return code
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f'BLOCKED dev verification: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
