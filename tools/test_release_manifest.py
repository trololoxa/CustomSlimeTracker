#!/usr/bin/env python3
from __future__ import annotations

import json
import shutil
import subprocess
import unittest
from datetime import datetime, timezone
from pathlib import Path

from quality_gate_runtime import project_temp_directory
from release_manifest import (
    SCHEMA,
    build_manifest,
    manifest_timestamp,
    parse_toolchains,
    write_manifest,
)


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("git"), "git executable is required")
class ReleaseManifestTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = project_temp_directory(ROOT, "tracker-release-manifest-")
        self.root = Path(self.tmp.name)
        subprocess.run(["git", "init", "-q", str(self.root)], check=True)
        subprocess.run(
            ["git", "-C", str(self.root), "config", "user.email", "test@example.invalid"],
            check=True,
        )
        subprocess.run(
            ["git", "-C", str(self.root), "config", "user.name", "Release Manifest Test"],
            check=True,
        )
        self.artifact = self.root / "firmware.bin"
        self.artifact.write_bytes(b"firmware-bytes\x00\x01")
        subprocess.run(["git", "-C", str(self.root), "add", "firmware.bin"], check=True)
        subprocess.run(["git", "-C", str(self.root), "commit", "-q", "-m", "baseline"], check=True)

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_clean_manifest_contains_full_identity_and_artifact_hash(self) -> None:
        manifest = build_manifest(
            self.root,
            "TEST_ENV",
            [self.artifact],
            toolchains={"platformio": "Core 6.1.18"},
            timestamp_utc="2026-08-01T12:00:00Z",
            require_clean=True,
        )
        self.assertEqual(manifest["schema"], SCHEMA)
        self.assertEqual(manifest["source"]["dirty"], False)  # type: ignore[index]
        self.assertEqual(len(manifest["source"]["commit"]), 40)  # type: ignore[index]
        artifact = manifest["artifacts"][0]  # type: ignore[index]
        self.assertEqual(artifact["path"], "firmware.bin")
        self.assertEqual(len(artifact["sha256"]), 64)

    def test_dirty_source_is_rejected_for_release(self) -> None:
        self.artifact.write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "dirty"):
            build_manifest(self.root, "TEST_ENV", [self.artifact], require_clean=True)

    def test_empty_artifact_is_rejected(self) -> None:
        self.artifact.write_bytes(b"")
        with self.assertRaisesRegex(ValueError, "empty"):
            build_manifest(self.root, "TEST_ENV", [self.artifact])

    def test_release_rejects_missing_or_unknown_toolchain_identity(self) -> None:
        with self.assertRaisesRegex(ValueError, "toolchain identity is required"):
            build_manifest(self.root, "TEST_ENV", [self.artifact], require_clean=True)
        with self.assertRaisesRegex(ValueError, "platformio=unknown"):
            build_manifest(
                self.root,
                "TEST_ENV",
                [self.artifact],
                toolchains={"platformio": "unknown"},
                require_clean=True,
            )

    def test_timestamp_and_serialization_are_reproducible(self) -> None:
        self.assertEqual(manifest_timestamp({"SOURCE_DATE_EPOCH": "0"}), "1970-01-01T00:00:00Z")
        instant = datetime(2026, 8, 1, 12, 34, 56, tzinfo=timezone.utc)
        self.assertEqual(manifest_timestamp({}, now=instant), "2026-08-01T12:34:56Z")
        manifest = build_manifest(
            self.root,
            "TEST_ENV",
            [self.artifact],
            timestamp_utc="1970-01-01T00:00:00Z",
        )
        output = self.root / "build" / "manifest.json"
        write_manifest(output, manifest)
        first = output.read_bytes()
        write_manifest(output, manifest)
        self.assertEqual(output.read_bytes(), first)
        self.assertFalse(output.with_name("manifest.json.tmp").exists())
        self.assertEqual(json.loads(first)["schema"], SCHEMA)

    def test_toolchain_parser_rejects_ambiguous_values(self) -> None:
        self.assertEqual(parse_toolchains(["pio=6.1", "gcc=13.3"]), {"pio": "6.1", "gcc": "13.3"})
        with self.assertRaises(ValueError):
            parse_toolchains(["missing-separator"])
        with self.assertRaises(ValueError):
            parse_toolchains(["pio=one", "pio=two"])
        with self.assertRaisesRegex(ValueError, "duplicate artifact"):
            build_manifest(self.root, "TEST_ENV", [self.artifact, self.artifact])
        with self.assertRaisesRegex(ValueError, "environment"):
            build_manifest(self.root, "", [self.artifact])


if __name__ == "__main__":
    unittest.main()
