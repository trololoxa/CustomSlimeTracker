#!/usr/bin/env python3
"""Build and run host-side standalone C++ tests.

The tests intentionally compile only host-safe headers. They do not link the
Arduino framework, do not talk to ESP32 hardware, and do not flash firmware.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import shutil
import subprocess
import sys
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
    pathlib.Path("src/sensor/mag_yaw_correction.cpp"),
    pathlib.Path("src/sensor/accel_6pos_calibration.cpp"),
    pathlib.Path("src/sensor/mag_calibration.cpp"),
    pathlib.Path("src/runtime/runtime_gyro_bias_controller.cpp"),
    pathlib.Path("src/runtime/gyro_temp_calibration_capture.cpp"),
    pathlib.Path("src/runtime/tracker_console_suppress.cpp"),
    pathlib.Path("src/runtime/tracking_state_controller.cpp"),
    pathlib.Path("src/runtime/tap_accumulator.cpp"),
    pathlib.Path("src/runtime/status_led_runtime.cpp"),
    pathlib.Path("src/runtime/battery_runtime.cpp"),
    pathlib.Path("src/connection/lsm6dsv_driver.cpp"),
    pathlib.Path("src/config/tracker_config_runtime.cpp"),
    pathlib.Path("src/output/slimevr_packet_writer.cpp"),
    pathlib.Path("src/network/wifi_manager.cpp"),
    pathlib.Path("src/network/udp_transport.cpp"),
    pathlib.Path("src/runtime/slimevr_output_runtime.cpp"),
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
    raise RuntimeError("No C++ compiler found. Set CXX or install g++/clang++." )


def test_sources() -> list[pathlib.Path]:
    return sorted(p for p in TEST_DIR.glob("test_*.cpp") if p.name != "test_common.hpp")


def object_id_for(source: pathlib.Path) -> pathlib.Path:
    """Return a stable project-relative id for an object file name.

    pathlib treats joining OBJ_DIR with an absolute source-derived path as an
    absolute path again.  On POSIX that escaped to /__mnt__... .o; on Windows
    the same pattern can escape to the drive root as .obj files.
    """
    try:
        return source.resolve().relative_to(ROOT)
    except ValueError:
        return pathlib.Path(*[part for part in source.parts if part not in (source.anchor, source.drive)])


def object_path_for(source: pathlib.Path) -> pathlib.Path:
    source_id = object_id_for(source).with_suffix("")
    safe_name = "__".join(part for part in source_id.parts if part) + object_suffix()
    return OBJ_DIR / safe_name


def compile_object(cxx: str, source: pathlib.Path, out: pathlib.Path, extra: Iterable[str]) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
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
    subprocess.run(cmd, cwd=ROOT, check=True)


def compile_project_objects(cxx: str, extra: Iterable[str]) -> list[pathlib.Path]:
    objects: list[pathlib.Path] = []
    for source in PROJECT_SOURCES:
        obj = object_path_for(source)
        compile_object(cxx, source, obj, extra)
        objects.append(obj)
    return objects


def compile_one(cxx: str, source: pathlib.Path, out: pathlib.Path, project_objects: list[pathlib.Path], extra: Iterable[str]) -> None:
    test_obj = object_path_for(source)
    compile_object(cxx, source, test_obj, extra)

    cmd = [
        cxx,
        str(test_obj),
        *(str(p) for p in project_objects),
        "-o",
        str(out),
    ]
    print("[link]", source.name, flush=True)
    subprocess.run(cmd, cwd=ROOT, check=True)


def run_one(exe: pathlib.Path) -> None:
    print("[run]", exe.name, flush=True)
    subprocess.run([str(exe)], cwd=ROOT, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Build and run standalone native tests")
    parser.add_argument("--cxx", help="C++ compiler to use; defaults to CXX/g++/clang++/c++")
    parser.add_argument("--build-only", action="store_true", help="Compile tests but do not run them")
    parser.add_argument("--clean", action="store_true", help="Remove build/native_tests before compiling")
    parser.add_argument("--extra-cxxflag", action="append", default=[], help="Extra compiler flag; may be repeated")
    args = parser.parse_args()

    cxx = find_compiler(args.cxx)

    if args.clean and BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    OBJ_DIR.mkdir(parents=True, exist_ok=True)

    sources = test_sources()
    if not sources:
        print("No native tests found", file=sys.stderr)
        return 1

    executables: list[pathlib.Path] = []
    try:
        project_objects = compile_project_objects(cxx, args.extra_cxxflag)
        for source in sources:
            exe = BUILD_DIR / (source.stem + executable_suffix())
            compile_one(cxx, source, exe, project_objects, args.extra_cxxflag)
            executables.append(exe)

        if not args.build_only:
            for exe in executables:
                run_one(exe)
    except subprocess.CalledProcessError as exc:
        return exc.returncode if exc.returncode else 1
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"OK {len(executables)} standalone test executable(s)", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
