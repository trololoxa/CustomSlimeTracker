#!/usr/bin/env python3
"""Build and run host-side standalone C++ tests.

The tests intentionally compile only host-safe headers. They do not link the
Arduino framework, do not talk to ESP32 hardware, and do not flash firmware.

All independent compile/link/run stages are attempted. Failures are collected
and printed together at the end instead of aborting at the first error.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import shlex
import shutil
import subprocess
import sys
import tempfile
from contextlib import contextmanager
from dataclasses import dataclass
from typing import BinaryIO, Iterable, Iterator, Mapping

from quality_gate_runtime import (
    asan_ubsan_environment,
    lsan_environment,
    quality_gate_environment,
    run_bounded_process,
)


ROOT = pathlib.Path(__file__).resolve().parents[1]
TEST_DIR = ROOT / "tests" / "native"
BUILD_ROOT = ROOT / "build" / "native_tests"
RUNNER_LOCK = ROOT / "build" / "native_tests.lock"
BUILD_DIR = BUILD_ROOT
OBJ_DIR = BUILD_DIR / "obj"
DEFAULT_COMMAND_TIMEOUT_S = 180.0
INVALID_OUTPUT_RETURNCODE = 125

_COMMAND_TIMEOUT_S = DEFAULT_COMMAND_TIMEOUT_S
_COMMAND_ENV: Mapping[str, str] | None = None
_LAST_COMMAND_RETURNCODE = 0


PROJECT_SOURCES = [
    pathlib.Path("src/sensor/ahrs_6dof.cpp"),
    pathlib.Path("src/sensor/imu_quality.cpp"),
    pathlib.Path("src/sensor/calibration.cpp"),
    pathlib.Path("src/sensor/gyro_temperature_compensation.cpp"),
    pathlib.Path("src/sensor/mag_runtime.cpp"),
    pathlib.Path("src/sensor/mag_heading.cpp"),
    pathlib.Path("src/sensor/mag_field_reliability.cpp"),
    pathlib.Path("src/sensor/mag_axis_alignment.cpp"),
    pathlib.Path("src/sensor/mag_yaw_correction.cpp"),
    pathlib.Path("src/sensor/accel_6pos_calibration.cpp"),
    pathlib.Path("src/sensor/mag_calibration.cpp"),
    pathlib.Path("src/sensor/sensor_to_device_alignment.cpp"),
    pathlib.Path("src/runtime/runtime_gyro_bias_controller.cpp"),
    pathlib.Path("src/runtime/runtime_profiler.cpp"),
    pathlib.Path("src/runtime/runtime_motion_diagnostics.cpp"),
    pathlib.Path("src/runtime/calibration_autonomy_store.cpp"),
    pathlib.Path("src/runtime/calibration_autonomy_controller.cpp"),
    pathlib.Path("src/runtime/machine_log_runtime.cpp"),
    pathlib.Path("src/runtime/output_runtime.cpp"),
    pathlib.Path("src/runtime/setup_output_verifier.cpp"),
    pathlib.Path("src/runtime/orientation_runtime_reset.cpp"),
    pathlib.Path("src/runtime/gyro_temp_calibration_capture.cpp"),
    pathlib.Path("src/runtime/gyro_temp_static_fit.cpp"),
    pathlib.Path("src/runtime/tracker_console_suppress.cpp"),
    pathlib.Path("src/runtime/tracking_state_controller.cpp"),
    pathlib.Path("src/runtime/motion_light_sleep_controller.cpp"),
    pathlib.Path("src/runtime/tap_accumulator.cpp"),
    pathlib.Path("src/runtime/tap_runtime_controller.cpp"),
    pathlib.Path("src/runtime/status_led_runtime.cpp"),
    pathlib.Path("src/runtime/battery_runtime.cpp"),
    pathlib.Path("src/runtime/battery_adc_batch_sampler.cpp"),
    pathlib.Path("src/connection/lsm6dsv_driver.cpp"),
    pathlib.Path("src/connection/lsm6dsv_fifo.cpp"),
    pathlib.Path("src/runtime/fifo_runtime_processor.cpp"),
    pathlib.Path("src/config/tracker_config_runtime.cpp"),
    pathlib.Path("src/config/tracker_config_calibration_capture.cpp"),
    pathlib.Path("src/config/tracker_config_storage.cpp"),
    pathlib.Path("src/config/tracker_config_store.cpp"),
    pathlib.Path("src/config/tracker_network_config.cpp"),
    pathlib.Path("src/output/slimevr_packet_writer.cpp"),
    pathlib.Path("src/network/wifi_manager.cpp"),
    pathlib.Path("src/network/udp_transport.cpp"),
    pathlib.Path("src/runtime/slimevr_output_runtime.cpp"),
    pathlib.Path("tests/native/sanitizer_runtime_options.cpp"),
]


# Sources that must stay host-compilable but are intentionally not linked into
# every native executable because they depend on hardware-backed persistence
# implementations at link time. Compiling them here catches interface drift in
# production-only paths before PlatformIO.
COMPILE_ONLY_SOURCES = [
    pathlib.Path("src/sensor/fifo_calibrations.cpp"),
    pathlib.Path("src/runtime/imu_sample_pipeline.cpp"),
    pathlib.Path("src/runtime/runtime_test_runner.cpp"),
    pathlib.Path("src/runtime/static_test_runner.cpp"),
    pathlib.Path("src/runtime/runtime_status_reporter.cpp"),
    pathlib.Path("src/runtime/mag_runtime_controller.cpp"),
    pathlib.Path("src/network/wifi_remote_console.cpp"),
    pathlib.Path("src/serial/tracker_slimevr_commands.cpp"),
    pathlib.Path("src/serial/tracker_fifo_config_control.cpp"),
    pathlib.Path("src/serial/tracker_config_commands.cpp"),
    pathlib.Path("src/serial/tracker_calibration_commands.cpp"),
    pathlib.Path("src/serial/tracker_setup_commands.cpp"),
    pathlib.Path("src/serial/tracker_perf_commands.cpp"),
    pathlib.Path("src/serial/tracker_motion_commands.cpp"),
    pathlib.Path("src/serial/tracker_imu_fifo_commands.cpp"),
    pathlib.Path("src/serial/tracker_network_commands.cpp"),
    pathlib.Path("src/serial/tracker_mag_commands.cpp"),
    pathlib.Path("src/serial/tracker_slimevr_serial_compat_commands.cpp"),
    pathlib.Path("src/serial/tracker_serial_commands.cpp"),
    pathlib.Path("src/serial/tracker_system_commands.cpp"),
    pathlib.Path("src/serial/tracker_output_commands.cpp"),
    pathlib.Path("src/serial/tracker_test_commands.cpp"),
    pathlib.Path("src/app/tracker_command_wiring.cpp"),
]

ARDUINO_COMPILE_ONLY_SOURCES = [
    pathlib.Path("src/app/tracker_app.cpp"),
    pathlib.Path("src/app/tracker_bootstrap.cpp"),
]


# Most compile-only production units are intentionally excluded from every
# native executable. Tests that exercise one of those units opt in explicitly
# so the rest of the suite does not inherit unrelated persistence/hardware
# dependencies at link time.
TEST_EXTRA_LINK_SOURCES = {
    "test_diagnostic_completion_deferred.cpp": (
        pathlib.Path("src/runtime/runtime_test_runner.cpp"),
        pathlib.Path("src/runtime/static_test_runner.cpp"),
    ),
}

WARNING_FLAGS = [
    "-Wall",
    "-Wextra",
    "-Wshadow",
    "-Wdouble-promotion",
    "-Wformat=2",
    "-Wno-unused-parameter",
]


BASE_FLAGS = [
    "-std=c++20",
    "-O2",
    *WARNING_FLAGS,
    "-Isrc",
    "-Itests/native",
]


SANITIZER_FLAGS = {
    "none": (),
    "undefined": (
        "-O1",
        "-g",
        "-fsanitize=undefined",
        "-fno-sanitize-recover=undefined",
        "-fno-omit-frame-pointer",
    ),
    "address-undefined": (
        "-O1",
        "-g",
        "-fsanitize=address,undefined",
        "-fno-sanitize-recover=undefined",
        "-fno-omit-frame-pointer",
    ),
    "leak": (
        "-O1",
        "-g",
        "-fsanitize=leak",
        "-fno-omit-frame-pointer",
        "-DTRACKER_ENABLE_LSAN=1",
    ),
}


@dataclass
class NativeFailure:
    stage: str
    item: str
    returncode: int


class NativeRunnerBusyError(RuntimeError):
    pass


def _lock_file(handle: BinaryIO) -> None:
    if os.name == "nt":
        import msvcrt

        handle.seek(0)
        if handle.read(1) == b"":
            handle.seek(0)
            handle.write(b"\0")
            handle.flush()
        handle.seek(0)
        msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
    else:
        import fcntl

        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)


def _unlock_file(handle: BinaryIO) -> None:
    if os.name == "nt":
        import msvcrt

        handle.seek(0)
        msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
    else:
        import fcntl

        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


@contextmanager
def native_runner_lock(path: pathlib.Path | None = None) -> Iterator[None]:
    """Serialize native runners so --clean cannot delete another live build."""
    lock_path = RUNNER_LOCK if path is None else path
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    handle = lock_path.open("a+b")
    try:
        try:
            _lock_file(handle)
        except OSError as exc:
            raise NativeRunnerBusyError(
                f"another native test runner owns {lock_path}"
            ) from exc
        try:
            yield
        finally:
            _unlock_file(handle)
    finally:
        handle.close()


def executable_suffix() -> str:
    return ".exe" if os.name == "nt" else ""


def object_suffix() -> str:
    return ".obj" if os.name == "nt" else ".o"


def find_compiler(explicit: str | None) -> str:
    candidates: list[str] = []
    if explicit:
        candidates.append(explicit)
    if os.environ.get("CXX"):
        candidates.append(os.environ["CXX"])
    candidates.extend(["g++", "clang++", "c++"])

    for candidate in candidates:
        if shutil.which(candidate):
            return candidate
    raise RuntimeError("No C++ compiler found. Set CXX or install g++/clang++.")


def test_sources() -> list[pathlib.Path]:
    return sorted(p for p in TEST_DIR.glob("test_*.cpp") if p.name != "test_common.hpp")


def object_id_for(source: pathlib.Path) -> pathlib.Path:
    """Return a stable project-relative id for an object file name."""
    try:
        return source.resolve().relative_to(ROOT)
    except ValueError:
        return pathlib.Path(*[part for part in source.parts if part not in (source.anchor, source.drive)])


def object_path_for(source: pathlib.Path) -> pathlib.Path:
    source_id = object_id_for(source).with_suffix("")
    safe_name = "__".join(part for part in source_id.parts if part) + object_suffix()
    return OBJ_DIR / safe_name


def configure_build_directory(*, clean: bool) -> pathlib.Path:
    """Select an invocation-private build tree immune to stale compiler children."""
    global BUILD_DIR, OBJ_DIR
    if clean and BUILD_ROOT.exists():
        shutil.rmtree(BUILD_ROOT)
    BUILD_ROOT.mkdir(parents=True, exist_ok=True)
    if not clean:
        # Keep the most recent prior invocation for debugging/build-only use,
        # but do not let repeated focused runs grow without bound. The caller
        # holds native_runner_lock(), so every run-* directory is inactive.
        previous = sorted(
            (path for path in BUILD_ROOT.glob("run-*") if path.is_dir()),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
        for stale in previous[1:]:
            shutil.rmtree(stale, ignore_errors=True)
    BUILD_DIR = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=BUILD_ROOT))
    OBJ_DIR = BUILD_DIR / "obj"
    OBJ_DIR.mkdir(parents=True, exist_ok=True)
    return BUILD_DIR


def _run_command(cmd: list[str]) -> int:
    global _LAST_COMMAND_RETURNCODE
    try:
        proc = run_bounded_process(
            cmd,
            cwd=ROOT,
            timeout_s=_COMMAND_TIMEOUT_S,
            env=_COMMAND_ENV,
        )
        _LAST_COMMAND_RETURNCODE = proc.returncode
    except subprocess.TimeoutExpired as exc:
        elapsed = exc.timeout if exc.timeout is not None else _COMMAND_TIMEOUT_S
        print(
            f"error: command timed out after {elapsed:g}s: {shlex.join(cmd)}",
            file=sys.stderr,
            flush=True,
        )
        _LAST_COMMAND_RETURNCODE = 124
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr, flush=True)
        _LAST_COMMAND_RETURNCODE = 127
    return _LAST_COMMAND_RETURNCODE


def command_failure_code() -> int:
    return _LAST_COMMAND_RETURNCODE if _LAST_COMMAND_RETURNCODE != 0 else 1


def _output_is_valid(path: pathlib.Path, *, executable: bool = False) -> bool:
    global _LAST_COMMAND_RETURNCODE
    valid = path.is_file() and path.stat().st_size > 0
    if valid and executable and os.name != "nt":
        valid = os.access(path, os.X_OK)
    if not valid:
        kind = "executable" if executable else "object"
        print(f"error: compiler returned success but produced an invalid {kind}: {path}", file=sys.stderr)
        _LAST_COMMAND_RETURNCODE = INVALID_OUTPUT_RETURNCODE
    return valid


def compile_object(cxx: str, source: pathlib.Path, out: pathlib.Path, extra: Iterable[str]) -> bool:
    out.parent.mkdir(parents=True, exist_ok=True)
    out.unlink(missing_ok=True)
    cmd = [
        cxx,
        *BASE_FLAGS,
        str(source),
        "-c",
        "-o",
        str(out),
        *extra,
    ]
    print("[obj]", source, flush=True)
    return _run_command(cmd) == 0 and _output_is_valid(out)


def compile_project_objects(
    cxx: str,
    extra: Iterable[str],
) -> tuple[list[pathlib.Path], list[NativeFailure]]:
    objects: list[pathlib.Path] = []
    failures: list[NativeFailure] = []
    for source in PROJECT_SOURCES:
        obj = object_path_for(source)
        if compile_object(cxx, source, obj, extra):
            objects.append(obj)
        else:
            failures.append(NativeFailure("project-compile", str(source), command_failure_code()))
    return objects, failures


def compile_one(
    cxx: str,
    source: pathlib.Path,
    out: pathlib.Path,
    project_objects: list[pathlib.Path],
    extra: Iterable[str],
) -> bool:
    test_obj = object_path_for(source)
    if not compile_object(cxx, source, test_obj, extra):
        return False

    out.unlink(missing_ok=True)
    cmd = [
        cxx,
        str(test_obj),
        *(str(p) for p in project_objects),
        *(str(object_path_for(p)) for p in TEST_EXTRA_LINK_SOURCES.get(source.name, ())),
        "-o",
        str(out),
        *extra,
    ]
    print("[link]", source.name, flush=True)
    return _run_command(cmd) == 0 and _output_is_valid(out, executable=True)


def run_one(exe: pathlib.Path) -> bool:
    print("[run]", exe.name, flush=True)
    return _run_command([str(exe)]) == 0


def print_failure_summary(failures: list[NativeFailure]) -> None:
    print("\n# standalone tests: FAIL", flush=True)
    print(f"# failures={len(failures)}", flush=True)
    for failure in failures:
        print(
            f"# FAIL [{failure.stage}] {failure.item} (exit={failure.returncode})",
            flush=True,
        )


def _normalize_extra_cxxflag_args(argv: list[str]) -> list[str]:
    """Accept both '--extra-cxxflag=-Werror' and '--extra-cxxflag -Werror'."""
    known_options = {
        "--build-only",
        "--clean",
        "--cxx",
        "--extra-cxxflag",
        "--sanitizer",
        "--timeout-s",
    }
    normalized: list[str] = []
    index = 0
    while index < len(argv):
        token = argv[index]
        if token == "--extra-cxxflag" and index + 1 < len(argv):
            value = argv[index + 1]
            if value not in known_options:
                normalized.append(f"--extra-cxxflag={value}")
                index += 2
                continue
        normalized.append(token)
        index += 1
    return normalized


def _run_suite(args: argparse.Namespace, cxx: str, extra_flags: list[str]) -> int:
    build_dir = configure_build_directory(clean=args.clean)
    print(f"# native build directory: {build_dir.relative_to(ROOT)}", flush=True)

    sources = test_sources()
    if not sources:
        print("No native tests found", file=sys.stderr)
        return 1

    failures: list[NativeFailure] = []
    executables: list[pathlib.Path] = []
    passed_runs = 0

    project_objects, project_failures = compile_project_objects(cxx, extra_flags)
    failures.extend(project_failures)

    for source in COMPILE_ONLY_SOURCES:
        if not compile_object(cxx, source, object_path_for(source), extra_flags):
            failures.append(NativeFailure("compile-only", str(source), command_failure_code()))

    for source in ARDUINO_COMPILE_ONLY_SOURCES:
        flags = [*extra_flags, "-DARDUINO"]
        if not compile_object(cxx, source, object_path_for(source), flags):
            failures.append(NativeFailure("arduino-compile-only", str(source), command_failure_code()))

    project_linkable = len(project_objects) == len(PROJECT_SOURCES)
    if project_linkable:
        for source in sources:
            exe = BUILD_DIR / (source.stem + executable_suffix())
            if compile_one(cxx, source, exe, project_objects, extra_flags):
                executables.append(exe)
                # Execute while the freshly validated file is still local to
                # this build step. Besides reducing peak retained state, this
                # closes the window in which an interrupted/stale host tool or
                # delayed filesystem filter can mutate a completed binary.
                if not args.build_only:
                    if run_one(exe):
                        passed_runs += 1
                    else:
                        failures.append(
                            NativeFailure("test-run", exe.name, command_failure_code())
                        )
            else:
                failures.append(
                    NativeFailure("test-compile-or-link", source.name, command_failure_code())
                )
    else:
        # Test translation units are still compiled so syntax/interface errors in
        # every test are reported even when one shared project source failed.
        for source in sources:
            if not compile_object(cxx, source, object_path_for(source), extra_flags):
                failures.append(NativeFailure("test-compile", source.name, command_failure_code()))
        failures.append(
            NativeFailure(
                "test-link",
                "all test links blocked by failed shared project object(s)",
                1,
            )
        )

    if failures:
        print_failure_summary(failures)
        print(
            f"# built_executables={len(executables)} passed_runs={passed_runs}",
            flush=True,
        )
        return 1

    if args.build_only:
        print(
            f"OK {len(executables)} standalone test executable(s) built "
            f"(sanitizer={args.sanitizer})",
            flush=True,
        )
    else:
        print(
            f"OK {passed_runs}/{len(executables)} standalone test executable(s) "
            f"(sanitizer={args.sanitizer})",
            flush=True,
        )
    return 0


def main(argv: list[str] | None = None) -> int:
    global _COMMAND_ENV, _COMMAND_TIMEOUT_S

    parser = argparse.ArgumentParser(description="Build and run standalone native tests")
    parser.add_argument("--cxx", help="C++ compiler to use; defaults to CXX/g++/clang++/c++")
    parser.add_argument("--build-only", action="store_true", help="Compile tests but do not run them")
    parser.add_argument("--clean", action="store_true", help="Remove build/native_tests before compiling")
    parser.add_argument("--extra-cxxflag", action="append", default=[], help="Extra compiler flag; may be repeated")
    parser.add_argument(
        "--sanitizer",
        choices=tuple(SANITIZER_FLAGS),
        default="none",
        help="select one explicit sanitizer gate; leak detection is never implicit in ASan/UBSan",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=DEFAULT_COMMAND_TIMEOUT_S,
        help="timeout for each compile, link or test subprocess",
    )
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    args = parser.parse_args(_normalize_extra_cxxflag_args(raw_argv))

    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")

    _COMMAND_TIMEOUT_S = args.timeout_s
    if args.sanitizer == "leak":
        _COMMAND_ENV = lsan_environment(ROOT, scope="native-tests-lsan")
    elif args.sanitizer in ("undefined", "address-undefined"):
        _COMMAND_ENV = asan_ubsan_environment(ROOT, scope="native-tests-sanitizers")
    else:
        _COMMAND_ENV = quality_gate_environment(ROOT, scope="native-tests")

    extra_flags = [*args.extra_cxxflag, *SANITIZER_FLAGS[args.sanitizer]]

    try:
        cxx = find_compiler(args.cxx)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    try:
        with native_runner_lock():
            return _run_suite(args, cxx, extra_flags)
    except NativeRunnerBusyError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 126


if __name__ == "__main__":
    raise SystemExit(main())
