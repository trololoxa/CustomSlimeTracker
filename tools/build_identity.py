#!/usr/bin/env python3
"""Deterministic Git/worktree identity generation for firmware builds.

The identity intentionally has no timestamp. Two builds of the same repository
state therefore produce the same generated header and do not force pointless
recompilation. Ignored files and .git internals are excluded; tracked and
non-ignored untracked files are hashed by relative path and content.
"""

from __future__ import annotations

import hashlib
import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class BuildIdentity:
    available: bool
    head: str
    worktree: str
    dirty: bool
    identity: str


def _run_git(root: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        ["git", "-C", str(root), *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


def git_available(root: Path) -> bool:
    try:
        proc = _run_git(root, "rev-parse", "--is-inside-work-tree", check=False)
    except OSError:
        return False
    return proc.returncode == 0 and proc.stdout.strip() == b"true"


def listed_project_files(root: Path) -> list[str]:
    proc = _run_git(
        root,
        "ls-files",
        "-z",
        "--cached",
        "--others",
        "--exclude-standard",
    )
    paths = [item.decode("utf-8", errors="surrogateescape") for item in proc.stdout.split(b"\0") if item]
    paths.sort()
    return paths


def compute_worktree_fingerprint(root: Path, paths: Iterable[str] | None = None) -> str:
    digest = hashlib.sha256()
    project_paths = list(paths) if paths is not None else listed_project_files(root)
    for relative in sorted(project_paths):
        encoded_path = relative.encode("utf-8", errors="surrogateescape")
        path = root / relative
        digest.update(b"P\0")
        digest.update(encoded_path)
        digest.update(b"\0")
        if path.is_symlink():
            digest.update(b"L\0")
            digest.update(os.readlink(path).encode("utf-8", errors="surrogateescape"))
        elif path.is_file():
            digest.update(b"F\0")
            with path.open("rb") as handle:
                while True:
                    block = handle.read(1024 * 1024)
                    if not block:
                        break
                    digest.update(block)
        else:
            # A tracked deletion must affect the identity even though git
            # ls-files still reports its path.
            digest.update(b"MISSING")
        digest.update(b"\0")
    return digest.hexdigest()[:8]


def collect_build_identity(root: Path) -> BuildIdentity:
    root = root.resolve()
    if not git_available(root):
        return BuildIdentity(False, "unknown", "unknown", False, "unknown")

    head_proc = _run_git(root, "rev-parse", "--short=8", "HEAD", check=False)
    head = head_proc.stdout.decode("ascii", errors="replace").strip() if head_proc.returncode == 0 else "unknown"
    status = _run_git(root, "status", "--porcelain=v1", "--untracked-files=all", check=False)
    dirty = status.returncode == 0 and bool(status.stdout.strip())
    worktree = compute_worktree_fingerprint(root)
    identity = f"{head}+{worktree}-dirty" if dirty else head
    return BuildIdentity(True, head, worktree, dirty, identity)


def _cpp_string(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


def load_firmware_feature_version() -> str:
    header = Path(__file__).resolve().parents[1] / "src" / "build_config" / "firmware_feature_version.hpp"
    match = re.search(
        r'^#define\s+TRACKER_FIRMWARE_FEATURE_VERSION\s+"([^"]+)"\s*$',
        header.read_text(encoding="utf-8"),
        flags=re.MULTILINE,
    )
    if not match:
        raise RuntimeError(f"TRACKER_FIRMWARE_FEATURE_VERSION is missing in {header}")
    return match.group(1)


FIRMWARE_FEATURE_VERSION = load_firmware_feature_version()


def render_generated_header(identity: BuildIdentity, pio_environment: str) -> str:
    firmware_version = FIRMWARE_FEATURE_VERSION
    return """#pragma once

// Generated automatically by tools/generate_build_identity.py.
// Do not edit or commit this file. It lives under the PlatformIO build tree.
#define TRACKER_BUILD_GIT_AVAILABLE {available}
#define TRACKER_BUILD_GIT_HEAD \"{head}\"
#define TRACKER_BUILD_WORKTREE_FINGERPRINT \"{worktree}\"
#define TRACKER_BUILD_GIT_DIRTY {dirty}
#define TRACKER_BUILD_IDENTITY_STRING \"{identity}\"
#define TRACKER_BUILD_PIO_ENVIRONMENT \"{pio_environment}\"
#define TRACKER_BUILD_FIRMWARE_VERSION \"{firmware_version}\"
""".format(
        available=1 if identity.available else 0,
        head=_cpp_string(identity.head),
        worktree=_cpp_string(identity.worktree),
        dirty=1 if identity.dirty else 0,
        identity=_cpp_string(identity.identity),
        pio_environment=_cpp_string(pio_environment),
        firmware_version=_cpp_string(firmware_version),
    )


def write_if_changed(path: Path, content: str) -> bool:
    encoded = content.replace("\r\n", "\n").encode("utf-8")
    if path.exists() and path.read_bytes() == encoded:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded)
    return True
