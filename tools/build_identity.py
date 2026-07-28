#!/usr/bin/env python3
"""Deterministic Git/worktree identity generation for firmware builds.

The Git/worktree identity intentionally has no timestamp. Ignored files and
.git internals are excluded; tracked and non-ignored untracked files are hashed
by relative path and content. The generated PlatformIO header additionally
contains an explicit UTC build date; SOURCE_DATE_EPOCH makes that date
reproducible when required.
"""

from __future__ import annotations

import hashlib
import os
import re
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Mapping


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


def resolve_build_date_utc(
    environment: Mapping[str, str] | None = None,
    now: datetime | None = None,
) -> str:
    env = os.environ if environment is None else environment
    source_date_epoch = env.get("SOURCE_DATE_EPOCH", "").strip()
    if source_date_epoch:
        try:
            instant = datetime.fromtimestamp(int(source_date_epoch, 10), tz=timezone.utc)
        except (ValueError, OverflowError, OSError) as exc:
            raise ValueError("SOURCE_DATE_EPOCH must be a representable integer timestamp") from exc
    else:
        instant = now if now is not None else datetime.now(timezone.utc)
        if instant.tzinfo is None:
            instant = instant.replace(tzinfo=timezone.utc)
        else:
            instant = instant.astimezone(timezone.utc)
    return instant.strftime("%Y-%m-%d")


def slimevr_firmware_version(build_date_utc: str) -> str:
    compact = build_date_utc.replace("-", "")
    if not re.fullmatch(r"[0-9]{8}", compact):
        raise ValueError("build date must use YYYY-MM-DD")
    return f"{FIRMWARE_FEATURE_VERSION}+build.{compact}"


def render_generated_header(
    identity: BuildIdentity,
    pio_environment: str,
    build_date_utc: str | None = None,
) -> str:
    firmware_version = FIRMWARE_FEATURE_VERSION
    resolved_build_date = build_date_utc or resolve_build_date_utc()
    compact_build_date = resolved_build_date.replace("-", "")
    server_firmware_version = slimevr_firmware_version(resolved_build_date)
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
#define TRACKER_BUILD_DATE_UTC \"{build_date_utc}\"
#define TRACKER_BUILD_DATE_COMPACT \"{build_date_compact}\"
#define TRACKER_BUILD_SLIMEVR_FIRMWARE_VERSION \"{slimevr_firmware_version}\"
""".format(
        available=1 if identity.available else 0,
        head=_cpp_string(identity.head),
        worktree=_cpp_string(identity.worktree),
        dirty=1 if identity.dirty else 0,
        identity=_cpp_string(identity.identity),
        pio_environment=_cpp_string(pio_environment),
        firmware_version=_cpp_string(firmware_version),
        build_date_utc=_cpp_string(resolved_build_date),
        build_date_compact=_cpp_string(compact_build_date),
        slimevr_firmware_version=_cpp_string(server_firmware_version),
    )


def write_if_changed(path: Path, content: str) -> bool:
    encoded = content.replace("\r\n", "\n").encode("utf-8")
    if path.exists() and path.read_bytes() == encoded:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded)
    return True
