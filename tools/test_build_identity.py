#!/usr/bin/env python3
from __future__ import annotations

import contextlib
import io
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from build_identity import FIRMWARE_FEATURE_VERSION, collect_build_identity, render_generated_header, write_if_changed


@unittest.skipUnless(shutil.which("git"), "git executable is required")
class BuildIdentityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        subprocess.run(["git", "init", "-q", str(self.root)], check=True)
        subprocess.run(["git", "-C", str(self.root), "config", "user.email", "test@example.invalid"], check=True)
        subprocess.run(["git", "-C", str(self.root), "config", "user.name", "Build Identity Test"], check=True)
        (self.root / ".gitignore").write_text(
            "ignored/\n.pio/\n__pycache__/\n*.py[cod]\n",
            encoding="utf-8",
        )
        (self.root / "tracked.txt").write_text("one\n", encoding="utf-8")
        subprocess.run(["git", "-C", str(self.root), "add", "."], check=True)
        subprocess.run(["git", "-C", str(self.root), "commit", "-q", "-m", "baseline"], check=True)

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_clean_and_dirty_identity(self) -> None:
        clean = collect_build_identity(self.root)
        self.assertTrue(clean.available)
        self.assertFalse(clean.dirty)
        self.assertEqual(clean.identity, clean.head)

        (self.root / "tracked.txt").write_text("two\n", encoding="utf-8")
        dirty = collect_build_identity(self.root)
        self.assertTrue(dirty.dirty)
        self.assertNotEqual(dirty.worktree, clean.worktree)
        self.assertEqual(dirty.identity, f"{dirty.head}+{dirty.worktree}-dirty")

    def test_untracked_changes_identity_but_ignored_file_does_not(self) -> None:
        clean = collect_build_identity(self.root)
        ignored = self.root / "ignored" / "artifact.bin"
        ignored.parent.mkdir()
        ignored.write_bytes(b"ignored")
        still_clean = collect_build_identity(self.root)
        self.assertFalse(still_clean.dirty)
        self.assertEqual(still_clean.worktree, clean.worktree)

        (self.root / "new_source.cpp").write_text("int x = 1;\n", encoding="utf-8")
        untracked = collect_build_identity(self.root)
        self.assertTrue(untracked.dirty)
        self.assertNotEqual(untracked.worktree, clean.worktree)

    def test_generated_header_is_stable(self) -> None:
        identity = collect_build_identity(self.root)
        content = render_generated_header(identity, "TEST_ENV")
        path = self.root / "out" / "tracker_build_identity_generated.hpp"
        self.assertTrue(write_if_changed(path, content))
        self.assertFalse(write_if_changed(path, content))
        self.assertIn(identity.head, path.read_text(encoding="utf-8"))
        self.assertIn("TEST_ENV", path.read_text(encoding="utf-8"))
        self.assertIn(FIRMWARE_FEATURE_VERSION, path.read_text(encoding="utf-8"))

    def test_platformio_hook_generates_header_and_include_path(self) -> None:
        script = Path(__file__).with_name("generate_build_identity.py")
        build_dir = self.root / ".pio" / "build" / "TEST_ENV"

        class FakeEnv:
            def __init__(self) -> None:
                self.cpppath: list[str] = []

            def subst(self, value: str) -> str:
                return {
                    "$PROJECT_DIR": str(self_root),
                    "$BUILD_DIR": str(build_dir),
                    "$PIOENV": "TEST_ENV",
                }[value]

            def Prepend(self, **kwargs: list[str]) -> None:
                self.cpppath[:0] = kwargs.get("CPPPATH", [])

        self_root = self.root
        fake_env = FakeEnv()
        # SCons may execute pre-build scripts without defining __file__. The
        # hook must rely only on the PlatformIO environment in that case.
        namespace: dict[str, object] = {
            "__name__": "__main__",
        }

        def fake_import(name: str) -> None:
            self.assertEqual(name, "env")
            namespace["env"] = fake_env

        namespace["Import"] = fake_import
        with contextlib.redirect_stdout(io.StringIO()):
            exec(compile(script.read_text(encoding="utf-8"), str(script), "exec"), namespace)

        generated_dir = build_dir / "generated"
        header = generated_dir / "tracker_build_identity_generated.hpp"
        self.assertTrue(header.exists())
        self.assertEqual(fake_env.cpppath, [str(generated_dir)])
        self.assertIn("TRACKER_BUILD_PIO_ENVIRONMENT \"TEST_ENV\"", header.read_text(encoding="utf-8"))
        after_hook = collect_build_identity(self.root)
        self.assertFalse(after_hook.dirty)


if __name__ == "__main__":
    unittest.main()
