#!/usr/bin/env python3
"""Documentation ownership, navigation and growth checks."""
from __future__ import annotations
import json
from pathlib import Path
import re
import sys
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
PATCH_NAME = re.compile(r'(?:^|[_-])(?:dev[_-]?\d{2}[a-z]*|(?:pre[_-])?00\d{2}[a-z]*)(?:[_-]|\.)', re.I)
LINK = re.compile(r'\[[^\]]*\]\(([^)]+)\)')
REQUIRED_DOCS = ('README.md', 'status.md', 'roadmap.md', 'architecture/ownership.md',
                 'architecture/coordinate_frames.md', 'development/maintenance.md',
                 'development/testing.md', 'development/session.md', 'development/test_map.md',
                 'reference/cli.md', 'reference/configuration.md')


def local_links(text):
    # Examples/code are not navigational links.
    text = re.sub(r'(?ms)^```.*?^```[^\n]*$', '', text)
    for raw in LINK.findall(text):
        value = raw.strip().strip('<>').split(maxsplit=1)[0]
        if not value.startswith(('#', 'http://', 'https://', 'mailto:')):
            yield unquote(value.split('#', 1)[0].split('?', 1)[0])


def validate(root=ROOT):
    root = root.resolve()
    limits = json.loads((root / 'tools/maintenance_limits.json').read_text(encoding='utf-8'))
    errors = []
    docs = sorted((root / 'docs').rglob('*.md'))
    for name in REQUIRED_DOCS:
        if not (root / 'docs' / name).is_file():
            errors.append(f'missing required document: docs/{name}')
    for name in ('README.md', 'AGENTS.md', '.gitignore', '.gitattributes'):
        if not (root / name).is_file():
            errors.append(f'missing root file: {name}')
    index = root / 'docs/README.md'
    indexed = {(index.parent / p).resolve() for p in local_links(index.read_text(encoding='utf-8'))} if index.exists() else set()
    doc_lines = 0
    for doc in docs:
        relative = doc.relative_to(root / 'docs')
        if len(relative.parts) > 2 or (len(relative.parts) == 2 and relative.parts[0] not in ('architecture', 'reference', 'development', 'decisions')):
            errors.append(f'unowned documentation location: {relative}')
        if (PATCH_NAME.search(doc.name) or doc.name.endswith('_report.md')
                or re.search(r'\d{4}-\d{2}-\d{2}', doc.name)):
            errors.append(f'patch/run history in active documentation: {relative}')
        if doc != index and doc.resolve() not in indexed:
            errors.append(f'page missing from documentation index: {relative}')
        count = len(doc.read_text(encoding='utf-8').splitlines()); doc_lines += count
        ceiling = limits['documentation']['reference_lines' if relative.parts[0] == 'reference' else 'page_lines']
        if count > ceiling:
            errors.append(f'document size budget: {relative}: {count} > {ceiling}')
    for doc in [root / 'README.md', root / 'AGENTS.md', *docs]:
        if not doc.is_file():
            continue
        for target in local_links(doc.read_text(encoding='utf-8')):
            path = (doc.parent / target).resolve()
            if not path.is_relative_to(root) or not path.exists():
                errors.append(f'{doc.relative_to(root)}: broken/escaping local link: {target}')
            elif path.is_relative_to(root / 'build'):
                errors.append(f'{doc.relative_to(root)}: ephemeral artifact required by active docs: {target}')
    if len(docs) > limits['documentation']['files'] or doc_lines > limits['documentation']['total_lines']:
        errors.append(f'documentation growth budget exceeded: files={len(docs)} lines={doc_lines}')
    return errors


def main():
    errors = validate()
    for error in errors:
        print(error, file=sys.stderr)
    print(f'# maintenance structure: {"FAIL" if errors else "PASS"}; no firmware or historical-run acceptance implied')
    return int(bool(errors))


if __name__ == '__main__':
    raise SystemExit(main())
