"""DEV-02 behavioral tests; no network, installations or connected device."""
from __future__ import annotations

import contextlib
import hashlib
import io
import json
import subprocess
import sys
import unittest
import zipfile
from pathlib import Path
from unittest import mock

import doctor as env
import lock_dev_wheels as locks
from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[1]


class DoctorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scratch = project_temp_directory(ROOT, "dev02-test-")
        self.root = Path(self.scratch.__enter__())
        self.addCleanup(self.scratch.__exit__, None, None, None)
        (self.root / "tools").mkdir()
        (self.root / "tools/requirements-platformio.txt").write_bytes(
            (ROOT / "tools/requirements-platformio.txt").read_bytes())
        (self.root / "platformio.ini").write_bytes((ROOT / "platformio.ini").read_bytes())
        self.doctor = env.Doctor(self.root, "firmware", 0.5)
        self.core = self.root / "core"
        for name, (_, version) in env.target_pins(self.root).items():
            path = self.core / ("platforms" if name == "espressif32" else "packages") / name
            path.mkdir(parents=True)
            (path / ("platform.json" if name == "espressif32" else "package.json")).write_text(
                json.dumps({"version": version}))

    def fake_run(self, command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        self.assertEqual(kwargs["timeout_s"], 0.5)
        if command == ["pio", "--version"]:
            out = "PlatformIO Core, version " + locks.bootstrap_version(self.root)
        elif "system" in command:
            out = json.dumps({"core_dir": {"value": str(self.core)}})
        elif "-dumpmachine" in command:
            out = "riscv32-esp-elf" if "riscv32" in command[0] else "x86_64-linux-gnu"
        elif "-o" in command:
            Path(command[command.index("-o") + 1]).write_bytes(b"object")
            out = ""
        elif "--version" in command:
            out = "GCC fixture"
        else:
            out = "TRACKER_HOST_CXX20_OK pointer_bits=64\n"
        return subprocess.CompletedProcess(command, 0, out, "")

    def test_target_success_with_object_proof(self) -> None:
        with mock.patch.object(env, "run_bounded_process", side_effect=self.fake_run):
            env.check_target(self.doctor, "pio")
        self.assertTrue(self.doctor.checks)
        self.assertTrue(all(c["status"] == "PASS" for c in self.doctor.checks))
        self.assertTrue(any(c["name"] == "target.compile" for c in self.doctor.checks))

    def test_missing_pio_is_failure(self) -> None:
        env.check_target(self.doctor, None)
        self.assertEqual(self.doctor.checks[-1]["status"], "FAIL")

    def test_wrong_core_version_stops_before_packages(self) -> None:
        with mock.patch.object(env, "run_bounded_process", return_value=subprocess.CompletedProcess([], 0, "PlatformIO Core, version 0.0.1", "")) as run:
            env.check_target(self.doctor, "pio")
        self.assertEqual(run.call_count, 1)
        self.assertEqual(self.doctor.checks[-1]["status"], "FAIL")

    def test_invalid_pin_is_reported(self) -> None:
        (self.root / "tools/requirements-platformio.txt").write_text("platformio>=6\n")
        env.check_target(self.doctor, "pio")
        self.assertEqual(self.doctor.checks[-1]["status"], "FAIL")

    def test_package_missing_wrong_version_or_bad_shape_fails(self) -> None:
        path = self.core / "packages/framework-arduinoespressif32/package.json"
        for value in (None, [], {}, {"version": "0.0.1"}, {"version": 3}):
            with self.subTest(value=value):
                self.doctor.checks.clear()
                if value is None:
                    path.unlink()
                else:
                    path.write_text(json.dumps(value))
                with mock.patch.object(env, "run_bounded_process", side_effect=self.fake_run):
                    env.check_target(self.doctor, "pio")
                check = next(c for c in self.doctor.checks if c["name"] == "package.framework-arduinoespressif32")
                self.assertEqual(check["status"], "FAIL")

    def test_target_rejects_host_abi(self) -> None:
        def wrong(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
            result = self.fake_run(command, **kwargs)
            if "-dumpmachine" in command:
                result.stdout = "x86_64-linux-gnu"
            return result
        with mock.patch.object(env, "run_bounded_process", side_effect=wrong):
            env.check_target(self.doctor, "pio")
        check = next(c for c in self.doctor.checks if c["name"] == "target.cxx.abi")
        self.assertEqual(check["status"], "FAIL")
        self.assertFalse(any(c["label"] == "target.compile" for c in self.doctor.commands))

    def test_host_success_requires_execution_proof(self) -> None:
        for output in ("TRACKER_HOST_CXX20_OK pointer_bits=64\n", "", "unrelated success", "TRACKER_HOST_CXX20_OK pointer_bits=64\nextra"):
            with self.subTest(output=output):
                self.doctor.checks.clear()
                def run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
                    result = self.fake_run(command, **kwargs)
                    if len(command) == 1:
                        result.stdout = output
                    return result
                with mock.patch.object(env, "run_bounded_process", side_effect=run):
                    env.check_host(self.doctor, "cxx")
                self.assertEqual(self.doctor.checks[-1]["status"], "PASS" if output.endswith("64\n") else "FAIL")

    def test_compile_zero_without_artifact_is_failure(self) -> None:
        with mock.patch.object(env, "run_bounded_process", return_value=subprocess.CompletedProcess([], 0, "version", "")):
            env.check_host(self.doctor, "cxx")
        self.assertEqual(self.doctor.checks[-1]["status"], "FAIL")

    def test_timeout_and_launch_error_preserve_evidence(self) -> None:
        for exc, code, state in ((subprocess.TimeoutExpired(["cxx"], .5, b"partial", b"details"), 124, "timeout"),
                                  (OSError("missing executable"), 127, "launch_error")):
            with self.subTest(state=state), mock.patch.object(env, "run_bounded_process", side_effect=exc):
                result = self.doctor.run("failure", ["cxx"])
            self.assertEqual(result.returncode, code)
            self.assertEqual(self.doctor.commands[-1]["state"], state)
            self.assertTrue(result.stderr)

    def test_explicit_bad_path_never_falls_back(self) -> None:
        with mock.patch.object(env.shutil, "which", return_value=None) as which:
            self.assertIsNone(env.executable("missing", ("g++", "clang++")))
        which.assert_called_once_with("missing")

    def test_core_info_rejects_malformed_and_relative(self) -> None:
        for raw in ("null", "[]", "{}", '{"core_dir": 2}', '{"core_dir":"relative"}', "broken"):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                env.core_directory(raw)
        self.assertEqual(env.core_directory(json.dumps({"core_dir": str(self.core)})), self.core)

    def test_unpinned_target_package_rejected(self) -> None:
        p = self.root / "platformio.ini"
        p.write_text(p.read_text().replace("@6.7.0", "@^6.7.0"))
        with self.assertRaises(ValueError):
            env.target_pins(self.root)

    def test_required_tool_absence_fails_optional_skips(self) -> None:
        self.doctor.version("gdb", None, False)
        self.doctor.version("gdb", None, True)
        self.assertEqual([c["status"] for c in self.doctor.checks], ["SKIP", "FAIL"])

    def test_cli_report_and_failure_exit(self) -> None:
        def checks(doctor: env.Doctor, *args: object) -> None:
            doctor.add("injected", "FAIL", "expected failure")
        with mock.patch.object(env, "ROOT", self.root), mock.patch.object(env, "perform", side_effect=checks), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(env.main(["--profile", "host"]), 1)
        reports = list((self.root / "build/doctor").glob("*.json"))
        self.assertEqual(len(reports), 1)
        report = json.loads(reports[0].read_text())
        self.assertTrue(report["failed"])
        self.assertEqual(report["schema"], "tracker-doctor-v1")
        self.assertIn("doctor: FAIL", output.getvalue())
        self.assertNotIn("environment_variables", report)

    def test_invalid_deadlines_rejected_before_work(self) -> None:
        for value in ("nan", "inf", "0", "-1"):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                env.main(["--timeout-s", value])
            self.assertEqual(exc.exception.code, 2)


class WheelLockTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scratch = project_temp_directory(ROOT, "dev02-wheels-")
        self.root = Path(self.scratch.__enter__())
        self.addCleanup(self.scratch.__exit__, None, None, None)
        self.version = locks.bootstrap_version(ROOT)

    def wheel(self, name: str, version: str, filename: str | None = None) -> Path:
        path = self.root / (filename or f"{name}-{version}-py3-none-any.whl")
        with zipfile.ZipFile(path, "w") as wheel:
            wheel.writestr("package.dist-info/METADATA", f"Name: {name}\nVersion: {version}\n")
        return path

    def test_deterministic_canonical_names_and_exact_hash(self) -> None:
        pio = self.wheel("platformio", self.version)
        self.wheel("Some_Package", "1.0")
        result = locks.render_lock(self.root, self.version)
        self.assertEqual(result, locks.render_lock(self.root, self.version))
        self.assertIn("some-package==1.0 --hash=sha256:", result)
        self.assertIn("platformio==" + self.version + " --hash=sha256:" + hashlib.sha256(pio.read_bytes()).hexdigest(), result)
        self.assertLess(result.index("platformio=="), result.index("some-package=="))

    def test_empty_missing_and_wrong_bootstrap_rejected(self) -> None:
        with self.assertRaises(ValueError):
            locks.render_lock(self.root, self.version)
        self.wheel("another", "1.0")
        with self.assertRaises(ValueError):
            locks.render_lock(self.root, self.version)
        self.wheel("platformio", "0.0.1")
        with self.assertRaises(ValueError):
            locks.render_lock(self.root, self.version)

    def test_multiple_versions_rejected(self) -> None:
        self.wheel("platformio", self.version)
        self.wheel("some_package", "1.0")
        self.wheel("some-package", "2.0")
        with self.assertRaisesRegex(ValueError, "multiple versions"):
            locks.render_lock(self.root, self.version)

    def test_changed_wheel_bytes_change_hash(self) -> None:
        wheel = self.wheel("platformio", self.version)
        before = locks.render_lock(self.root, self.version)
        with zipfile.ZipFile(wheel, "a") as archive:
            archive.writestr("payload", "changed")
        self.assertNotEqual(before, locks.render_lock(self.root, self.version))

    def test_missing_or_oversized_metadata_rejected(self) -> None:
        for content in (None, "X" * (1024 * 1024 + 1)):
            path = self.root / "bad.whl"
            with zipfile.ZipFile(path, "w") as archive:
                archive.writestr("payload" if content is None else "bad.dist-info/METADATA", content or "")
            with self.assertRaises(ValueError):
                locks.render_lock(self.root, self.version)

    def test_invalid_metadata_name_version_rejected(self) -> None:
        for name, version in (("--index-url", "1.0"), ("bad name", "1.0"), ("platformio", "1.0 --extra-index-url")):
            self.wheel(name, version, "bad.whl")
            with self.assertRaises(ValueError):
                locks.render_lock(self.root, self.version)

    def test_existing_lock_not_overwritten(self) -> None:
        self.wheel("platformio", self.version)
        output = self.root / "requirements.lock"
        output.write_text("accepted\n")
        argv = ["lock_dev_wheels.py", "--wheelhouse", str(self.root), "--output", str(output)]
        with mock.patch.object(sys, "argv", argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
            locks.main()
        self.assertEqual(exc.exception.code, 1)
        self.assertEqual(output.read_text(), "accepted\n")

    def test_ambiguous_metadata_rejected(self) -> None:
        path = self.root / "bad.whl"
        with zipfile.ZipFile(path, "w") as archive:
            archive.writestr("p.dist-info/METADATA", "Name: platformio\nVersion: 6.1.18\nVersion: 1.0\n")
        with self.assertRaisesRegex(ValueError, "one Name and one Version"):
            locks.render_lock(self.root, self.version)


if __name__ == "__main__":
    unittest.main()
