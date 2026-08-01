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
import shutil
import subprocess
import sys
from dataclasses import dataclass
from typing import Iterable


ROOT = pathlib.Path(__file__).resolve().parents[1]
TEST_DIR = ROOT / "tests" / "native"
BUILD_DIR = ROOT / "build" / "native_tests"
OBJ_DIR = BUILD_DIR / "obj"


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
]


# Sources that must stay host-compilable but are intentionally not linked into
# every native executable because they depend on hardware-backed persistence
# implementations at link time. Compiling them here catches interface drift in
# production-only paths before PlatformIO.
COMPILE_ONLY_SOURCES = [
    pathlib.Path("src/sensor/fifo_calibrations.cpp"),
    pathlib.Path("src/runtime/imu_sample_pipeline.cpp"),
    pathlib.Path("src/runtime/runtime_status_reporter.cpp"),
    pathlib.Path("src/runtime/mag_runtime_controller.cpp"),
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
    pathlib.Path("src/app/tracker_command_wiring.cpp"),
]

ARDUINO_COMPILE_ONLY_SOURCES = [
    pathlib.Path("src/app/tracker_app.cpp"),
    pathlib.Path("src/app/tracker_bootstrap.cpp"),
]

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


@dataclass
class NativeFailure:
    stage: str
    item: str
    returncode: int


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


def _run_command(cmd: list[str]) -> int:
    try:
        subprocess.run(cmd, cwd=ROOT, check=True)
        return 0
    except subprocess.CalledProcessError as exc:
        return exc.returncode if exc.returncode else 1
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr, flush=True)
        return 1


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
    return _run_command(cmd) == 0


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
            failures.append(NativeFailure("project-compile", str(source), 1))
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
        "-o",
        str(out),
        *extra,
    ]
    print("[link]", source.name, flush=True)
    return _run_command(cmd) == 0


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


def main() -> int:
    parser = argparse.ArgumentParser(description="Build and run standalone native tests")
    parser.add_argument("--cxx", help="C++ compiler to use; defaults to CXX/g++/clang++/c++")
    parser.add_argument("--build-only", action="store_true", help="Compile tests but do not run them")
    parser.add_argument("--clean", action="store_true", help="Remove build/native_tests before compiling")
    parser.add_argument("--extra-cxxflag", action="append", default=[], help="Extra compiler flag; may be repeated")
    args = parser.parse_args()

    try:
        cxx = find_compiler(args.cxx)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if args.clean and BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    OBJ_DIR.mkdir(parents=True, exist_ok=True)

    sources = test_sources()
    if not sources:
        print("No native tests found", file=sys.stderr)
        return 1

    failures: list[NativeFailure] = []
    executables: list[pathlib.Path] = []

    project_objects, project_failures = compile_project_objects(cxx, args.extra_cxxflag)
    failures.extend(project_failures)

    for source in COMPILE_ONLY_SOURCES:
        if not compile_object(cxx, source, object_path_for(source), args.extra_cxxflag):
            failures.append(NativeFailure("compile-only", str(source), 1))

    for source in ARDUINO_COMPILE_ONLY_SOURCES:
        flags = [*args.extra_cxxflag, "-DARDUINO"]
        if not compile_object(cxx, source, object_path_for(source), flags):
            failures.append(NativeFailure("arduino-compile-only", str(source), 1))

    project_linkable = len(project_objects) == len(PROJECT_SOURCES)
    if project_linkable:
        for source in sources:
            exe = BUILD_DIR / (source.stem + executable_suffix())
            if compile_one(cxx, source, exe, project_objects, args.extra_cxxflag):
                executables.append(exe)
            else:
                failures.append(NativeFailure("test-compile-or-link", source.name, 1))
    else:
        # Test translation units are still compiled so syntax/interface errors in
        # every test are reported even when one shared project source failed.
        for source in sources:
            if not compile_object(cxx, source, object_path_for(source), args.extra_cxxflag):
                failures.append(NativeFailure("test-compile", source.name, 1))
        failures.append(
            NativeFailure(
                "test-link",
                "all test links blocked by failed shared project object(s)",
                1,
            )
        )

    passed_runs = 0
    if not args.build_only:
        for exe in executables:
            if run_one(exe):
                passed_runs += 1
            else:
                failures.append(NativeFailure("test-run", exe.name, 1))

    if failures:
        print_failure_summary(failures)
        print(
            f"# built_executables={len(executables)} passed_runs={passed_runs}",
            flush=True,
        )
        return 1

    if args.build_only:
        print(f"OK {len(executables)} standalone test executable(s) built", flush=True)
    else:
        print(f"OK {passed_runs}/{len(executables)} standalone test executable(s)", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
