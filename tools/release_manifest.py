#!/usr/bin/env python3
"""Create a traceable SHA-256 manifest for one firmware build environment."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Mapping, Sequence

from build_identity import FIRMWARE_FEATURE_VERSION, collect_build_identity


SCHEMA = "tracker-release-manifest-v1"


def _git_text(root: Path, *args: str) -> str:
    try:
        proc = subprocess.run(
            ["git", "-C", str(root), *args],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "unknown"
    value = proc.stdout.strip()
    return value if proc.returncode == 0 and value else "unknown"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def manifest_timestamp(
    environment: Mapping[str, str] | None = None,
    now: datetime | None = None,
) -> str:
    env = os.environ if environment is None else environment
    raw_epoch = env.get("SOURCE_DATE_EPOCH", "").strip()
    if raw_epoch:
        try:
            instant = datetime.fromtimestamp(int(raw_epoch, 10), tz=timezone.utc)
        except (ValueError, OverflowError, OSError) as exc:
            raise ValueError("SOURCE_DATE_EPOCH must be a representable integer timestamp") from exc
    else:
        instant = now if now is not None else datetime.now(timezone.utc)
        if instant.tzinfo is None:
            instant = instant.replace(tzinfo=timezone.utc)
        else:
            instant = instant.astimezone(timezone.utc)
    return instant.isoformat(timespec="seconds").replace("+00:00", "Z")


def source_manifest(root: Path) -> dict[str, object]:
    try:
        identity = collect_build_identity(root)
    except (OSError, subprocess.SubprocessError):
        return {
            "git_available": False,
            "commit": "unknown",
            "head_short": "unknown",
            "dirty": False,
            "worktree_fingerprint": "unknown",
            "identity": "unknown",
        }
    commit = _git_text(root, "rev-parse", "HEAD") if identity.available else "unknown"
    return {
        "git_available": identity.available,
        "commit": commit,
        "head_short": identity.head[:8],
        "dirty": identity.dirty,
        "worktree_fingerprint": identity.worktree,
        "identity": identity.identity,
    }


def require_release_source(source: Mapping[str, object]) -> None:
    commit = str(source.get("commit", "unknown"))
    if not source.get("git_available") or not re.fullmatch(r"[0-9a-fA-F]{40}", commit):
        raise ValueError("release source identity is unavailable (git=unknown)")
    for field in ("head_short", "worktree_fingerprint", "identity"):
        if source.get(field) in (None, "", "unknown"):
            raise ValueError(f"release source identity is incomplete ({field}=unknown)")
    if source.get("dirty"):
        raise ValueError(f"release source tree is dirty ({source.get('identity', 'unknown')})")


def _artifact_entry(root: Path, artifact: Path) -> dict[str, object]:
    resolved_root = root.resolve()
    resolved = artifact.resolve()
    try:
        relative = resolved.relative_to(resolved_root)
    except ValueError as exc:
        raise ValueError(f"artifact is outside the repository: {artifact}") from exc
    if not resolved.is_file():
        raise ValueError(f"artifact does not exist or is not a file: {artifact}")
    before = resolved.stat()
    if before.st_size <= 0:
        raise ValueError(f"artifact is empty: {artifact}")
    digest = sha256_file(resolved)
    after = resolved.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError(f"artifact changed while it was being hashed: {artifact}")
    return {
        "path": relative.as_posix(),
        "size_bytes": after.st_size,
        "sha256": digest,
    }


def normalize_toolchains(
    toolchains: Mapping[str, str] | None,
    *,
    require_known: bool,
) -> dict[str, str]:
    normalized: dict[str, str] = {}
    for raw_name, raw_value in (toolchains or {}).items():
        if not isinstance(raw_name, str) or not isinstance(raw_value, str):
            raise ValueError("toolchain names and values must be strings")
        name = raw_name.strip()
        value = raw_value.strip()
        if not name or not value:
            raise ValueError("toolchain names and values must not be empty")
        if name in normalized:
            raise ValueError(f"duplicate normalized toolchain name: {name}")
        if require_known and value.lower() in {"unknown", "unavailable", "n/a"}:
            raise ValueError(f"release toolchain identity is unavailable ({name}={value})")
        normalized[name] = value
    if require_known and not normalized:
        raise ValueError("release toolchain identity is required")
    return dict(sorted(normalized.items()))


def build_manifest(
    root: Path,
    environment_name: str,
    artifacts: Iterable[Path],
    *,
    toolchains: Mapping[str, str] | None = None,
    timestamp_utc: str | None = None,
    require_clean: bool = False,
) -> dict[str, object]:
    if not environment_name.strip():
        raise ValueError("build environment name is required")
    resolved_artifacts = [path.resolve() for path in artifacts]
    if len(resolved_artifacts) != len(set(resolved_artifacts)):
        raise ValueError("duplicate artifact path")
    artifact_entries = sorted(
        (_artifact_entry(root, path) for path in resolved_artifacts),
        key=lambda entry: str(entry["path"]),
    )
    if not artifact_entries:
        raise ValueError("at least one artifact is required")

    source = source_manifest(root)
    if require_clean:
        require_release_source(source)
    normalized_toolchains = normalize_toolchains(toolchains, require_known=require_clean)

    return {
        "schema": SCHEMA,
        "generated_at_utc": timestamp_utc or manifest_timestamp(),
        "source": source,
        "build": {
            "environment": environment_name,
            "firmware_feature_version": FIRMWARE_FEATURE_VERSION,
            "source_date_epoch": os.environ.get("SOURCE_DATE_EPOCH") or None,
        },
        "toolchains": normalized_toolchains,
        "host": {
            "python": platform.python_version(),
            "platform": platform.platform(),
        },
        "artifacts": artifact_entries,
    }


def write_manifest(path: Path, manifest: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    content = json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(content, encoding="utf-8", newline="\n")
    temporary.replace(path)


def parse_toolchains(values: Sequence[str]) -> dict[str, str]:
    parsed: dict[str, str] = {}
    for raw in values:
        name, separator, value = raw.partition("=")
        if not separator or not name.strip() or not value.strip():
            raise ValueError(f"toolchain must use NAME=VALUE: {raw!r}")
        key = name.strip()
        if key in parsed:
            raise ValueError(f"duplicate toolchain name: {key}")
        parsed[key] = value.strip()
    return parsed


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--environment", required=True, help="PlatformIO environment name")
    parser.add_argument("--artifact", action="append", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--toolchain", action="append", default=[], metavar="NAME=VALUE")
    parser.add_argument("--require-clean", action="store_true")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)

    root = args.root.resolve()
    artifacts = [path if path.is_absolute() else root / path for path in args.artifact]
    output = args.output if args.output.is_absolute() else root / args.output
    try:
        manifest = build_manifest(
            root,
            args.environment,
            artifacts,
            toolchains=parse_toolchains(args.toolchain),
            require_clean=args.require_clean,
        )
        write_manifest(output, manifest)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"# release manifest: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
