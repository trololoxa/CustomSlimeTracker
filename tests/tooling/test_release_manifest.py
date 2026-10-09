#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]

import json
import shutil
import subprocess
import unittest
import zipfile
from unittest import mock
from datetime import datetime, timezone

from quality_gate_runtime import project_temp_directory
from release_manifest import (
    SCHEMA,
    build_manifest,
    manifest_timestamp,
    parse_toolchains,
    write_manifest,
    write_firmware_bundle,
    sha256_file,
    main,
)


ROOT = Path(__file__).resolve().parents[2]


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

    def firmware_manifest(self):
        # Build products are ignored, like real PlatformIO products; source is clean.
        products = self.root / '.git/info/exclude'
        with products.open('a', encoding='utf-8') as handle:
            handle.write('\nbuild/\n.pio/\n')
        paths = []
        for name in ('firmware.bin', 'firmware.elf', 'partitions.bin', 'bootloader.bin'):
            path = self.root / '.pio/build/TEST_ENV' / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(name.encode() * 20)
            paths.append(path)
        manifest = build_manifest(self.root, 'TEST_ENV', paths,
                                  toolchains={'platformio': 'Core 6.1.18'}, require_clean=True)
        return manifest, paths, self.root / 'build/output.zip'

    def test_bundle_roundtrip_survives_rebuild_and_preserves_manifest_contract(self):
        manifest, paths, output = self.firmware_manifest()
        original = json.loads(json.dumps(manifest))
        restored_manifest = write_firmware_bundle(self.root, manifest, output)
        self.assertEqual(manifest, original)
        for path in paths:
            path.write_bytes(b'new-build')
        with zipfile.ZipFile(output) as archive:
            self.assertIsNone(archive.testzip())
            self.assertEqual(len(archive.namelist()), 5)
            self.assertTrue(all(n.startswith('build/firmware-bundles/') for n in archive.namelist()))
            archive.extractall(self.root)
        saved = json.loads((self.root / restored_manifest).read_text())
        self.assertEqual(saved['source'], original['source'])
        self.assertEqual(saved['toolchains'], original['toolchains'])
        for entry in saved['artifacts']:
            self.assertNotIn('.pio', entry['path'])
            path = self.root / entry['path']
            self.assertEqual(path.stat().st_size, entry['size_bytes'])
            self.assertEqual(sha256_file(path), entry['sha256'])

    def test_bundle_rejects_changed_missing_empty_or_duplicate_artifacts(self):
        manifest, paths, output = self.firmware_manifest()
        for replacement in (b'stale', b''):
            paths[0].write_bytes(replacement)
            with self.assertRaises(ValueError):
                write_firmware_bundle(self.root, manifest, output)
            self.assertFalse(output.exists())
        manifest['artifacts'].pop()
        with self.assertRaisesRegex(ValueError, 'exactly'):
            write_firmware_bundle(self.root, manifest, output)
        manifest['artifacts'].append(manifest['artifacts'][0])
        with self.assertRaisesRegex(ValueError, 'exactly'):
            write_firmware_bundle(self.root, manifest, output)

    def test_bundle_rejects_unsafe_paths_and_dirty_identity(self):
        manifest, paths, output = self.firmware_manifest()
        for environment in ('../escape', '/absolute', 'x\\y'):
            altered = json.loads(json.dumps(manifest))
            altered['build']['environment'] = environment
            with self.assertRaisesRegex(ValueError, 'unsafe'):
                write_firmware_bundle(self.root, altered, output)
        manifest['source']['dirty'] = True
        with self.assertRaisesRegex(ValueError, 'dirty'):
            write_firmware_bundle(self.root, manifest, output)
        manifest['source']['dirty'] = False
        manifest['artifacts'][0]['path'] = '../' + Path(manifest['artifacts'][0]['path']).name
        with self.assertRaisesRegex(ValueError, 'outside'):
            write_firmware_bundle(self.root, manifest, output)
        self.assertFalse(output.exists())

    def test_bundle_does_not_overwrite_existing_archive(self):
        manifest, _, output = self.firmware_manifest()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(b'old-good')
        with self.assertRaisesRegex(ValueError, 'already exists'):
            write_firmware_bundle(self.root, manifest, output)
        self.assertEqual(output.read_bytes(), b'old-good')

    def test_bundle_detects_change_during_copy_and_removes_only_temporary_output(self):
        manifest, paths, output = self.firmware_manifest()
        real_open = zipfile.ZipFile.open
        changed = False
        def mutate(archive, name, mode='r', *args, **kwargs):
            nonlocal changed
            if mode == 'w' and not changed:
                paths[0].write_bytes(b'changed-during-copy')
                changed = True
            return real_open(archive, name, mode, *args, **kwargs)
        with mock.patch.object(zipfile.ZipFile, 'open', new=mutate):
            with self.assertRaisesRegex(ValueError, 'changed while'):
                write_firmware_bundle(self.root, manifest, output)
        self.assertFalse(output.exists())
        self.assertEqual(list(output.parent.glob('tracker-bundle-*')), [])
        self.assertEqual(paths[0].read_bytes(), b'changed-during-copy')

    def test_bundle_cli_success_and_failure_are_explicit(self):
        manifest, paths, output = self.firmware_manifest()
        args = ['--root', str(self.root), '--environment', 'TEST_ENV',
                '--output', 'build/original-manifest.json', '--bundle', str(output),
                '--toolchain', 'platformio=6.1.18', '--require-clean']
        for path in paths:
            args += ['--artifact', str(path)]
        self.assertEqual(main(args), 0)
        before = output.read_bytes()
        saved_manifest = self.root / 'build/original-manifest.json'
        manifest_before = saved_manifest.read_bytes()
        paths[0].write_bytes(b'another-build')
        self.assertEqual(main(args), 1)
        self.assertEqual(output.read_bytes(), before)
        self.assertEqual(saved_manifest.read_bytes(), manifest_before)


if __name__ == "__main__":
    unittest.main()
