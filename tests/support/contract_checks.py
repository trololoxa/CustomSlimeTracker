"""Shared static-contract and host stack checks; no firmware budgets live here."""
import hashlib
import json
import tempfile
from pathlib import Path
from run_standalone_tests import find_compiler, DEFAULT_COMMAND_TIMEOUT_S, DEFAULT_SANITIZER_BUILD_TIMEOUT_S
from quality_gate_runtime import asan_ubsan_environment, run_bounded_process
from suite_inventory import native_test_text

ROOT = Path(__file__).resolve().parents[2]
STACK_EVIDENCE_BYTES = 8 * 1024 * 1024
STACK_EVIDENCE_FILES = 64


def require(text, needle, description='contract'):
    if needle not in text:
        raise SystemExit(f'missing {description}: {needle}')


def forbid(text, needle, description='contract'):
    if needle in text:
        raise SystemExit(f'forbidden {description}: {needle}')


def compiler():
    # Honor an explicit CXX; an unavailable requested compiler must not silently change ABI.
    return find_compiler(None)


class StackUsage(dict):
    """Parsed frames plus the input inventory, retained only for failure evidence."""
    def __init__(self, files):
        super().__init__()
        self.files = files


def parse_stack_usage(directory: Path):
    usage = StackUsage(tuple(sorted(directory.rglob('*.su'))))
    for path in usage.files:
        for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
            parts = line.split('\t')
            if len(parts) < 2:
                continue
            try:
                size = int(parts[1])
            except ValueError:
                continue
            name = parts[0].split(':', 3)[-1]
            usage[name] = max(size, usage.get(name, 0))
    return usage


def missing_stack_function(usage, symbol):
    message = f'stack policy did not find function: {symbol}'
    if isinstance(usage, StackUsage):
        # A self-cleaning compiler directory otherwise destroys the only evidence.
        # Keep bounded raw inputs only for missing-record failures, never on PASS.
        try:
            parent = ROOT / 'build/gate_runs'
            parent.mkdir(parents=True, exist_ok=True)
            directory = Path(tempfile.mkdtemp(prefix='stack-evidence-', dir=parent))
            manifest = {'missing_function': symbol, 'file_count': len(usage.files),
                        'files': [], 'truncated': False}
            remaining = STACK_EVIDENCE_BYTES
            for index, path in enumerate(usage.files):
                if index >= STACK_EVIDENCE_FILES or remaining == 0:
                    manifest['truncated'] = True
                    break
                with path.open('rb') as stream:
                    data = stream.read(remaining + 1)
                truncated = len(data) > remaining
                data = data[:remaining]
                name = f'{index:02d}.su'
                (directory / name).write_bytes(data)
                manifest['files'].append({'source': str(path), 'copy': name,
                    'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
                    'truncated': truncated})
                manifest['truncated'] |= truncated
                remaining -= len(data)
            target = directory / 'inventory.json'
            target.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
            message += f'; stack evidence={target}'
        except OSError as exc:
            message += f'; evidence save failed: {exc}'
    raise SystemExit(message)


def require_limit(usage, needle, limit):
    matches = [(name, size) for name, size in usage.items() if needle in name]
    if not matches:
        missing_stack_function(usage, needle)
    name, size = max(matches, key=lambda item: item[1])
    if size > limit:
        raise SystemExit(f'stack budget exceeded: {name} uses {size} bytes, limit {limit}')


def isolate(haystack: str, begin: str, end: str) -> str:
    start = haystack.find(begin)
    if start < 0:
        raise SystemExit(f"section start not found: {begin}")
    finish = haystack.find(end, start)
    if finish < 0:
        raise SystemExit(f"section end not found: {end}")
    return haystack[start:finish]


def between(text: str, start: str, end: str, label: str) -> str:
    first = text.find(start)
    if first < 0:
        raise SystemExit(f"missing {label} start: {start}")
    last = text.find(end, first + len(start))
    if last < 0:
        raise SystemExit(f"missing {label} end: {end}")
    return text[first:last]


def function_body(text: str, signature: str) -> str:
    start = text.index(signature)
    brace = text.index("{", start)
    depth = 0
    for i in range(brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[brace + 1:i]
    raise SystemExit(f"unterminated function: {signature}")


def stack_usage(directory, symbol):
    usage = parse_stack_usage(directory)
    matches = [size for name, size in usage.items() if symbol in name]
    if not matches:
        missing_stack_function(usage, symbol)
    return max(matches)


def run_contract_command(cmd, *, cwd=None, env=None, check=True,
                         timeout_s=DEFAULT_SANITIZER_BUILD_TIMEOUT_S, stdout=None, text=False):
    """Bound compiler/contract subprocess trees even outside check_all; never retry."""
    result = run_bounded_process(cmd, cwd=cwd, env=env, timeout_s=timeout_s,
                                 stdout=stdout, text=text)
    if check:
        result.check_returncode()
    return result


def run(cmd):
    run_contract_command(cmd, cwd=ROOT, env=asan_ubsan_environment(ROOT))


def compile_run(cxx, exe, sources, flags):
    run([
        cxx, "-std=c++20", *flags,
        "-I", str(ROOT / "src"),
        "-I", str(ROOT / "tests/native"),
        *[str(ROOT / source) for source in sources],
        str(ROOT / "tests/native/sanitizer_runtime_options.cpp"),
        "-o", str(exe),
    ])
    run_contract_command([str(exe)], cwd=ROOT, env=asan_ubsan_environment(ROOT),
                         timeout_s=DEFAULT_COMMAND_TIMEOUT_S)


def read_native_test(path):
    """Expand owned case headers for legacy source guards, not as behavior proof."""
    return native_test_text(Path(path), ROOT / 'tests/native')[0]
