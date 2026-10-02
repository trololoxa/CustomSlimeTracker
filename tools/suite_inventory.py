"""Fail-closed executable discovery shared by runners and structural validation."""
from pathlib import Path
import re


def check_registry(entries):
    registry = {}
    for script, description in entries:
        selector = Path(script).stem
        if selector in registry:
            raise ValueError(f'duplicate check ID: {selector}: {registry[selector][0]} and {script}')
        registry[selector] = (script, description)
    return registry


def native_sources(directory):
    sources = sorted(directory.glob('test_*.cpp'))
    nested = set(directory.rglob('test_*.cpp')) - set(sources)
    if nested:
        raise ValueError('unowned nested native executable: ' + ', '.join(str(p.relative_to(directory)) for p in sorted(nested)))
    return sources


def native_test_text(source, directory):
    """Read an entrypoint and owned case headers; reject escaping/cyclic includes."""
    directory = directory.resolve()
    owned = set()

    def expand(path, active):
        path = path.resolve()
        if not path.is_relative_to(directory):
            raise ValueError(f'native case include escapes suite: {path}')
        if path in active:
            raise ValueError(f'cyclic native case include: {path}')
        text = path.read_text(encoding='utf-8')
        def include(match):
            child = (directory / match[1]).resolve()
            owned.add(child)
            return expand(child, active | {path})
        return re.sub(r'^#include "(cases/[^"\n]+)"[^\n]*$', include, text, flags=re.M)

    return expand(source, set()), owned
