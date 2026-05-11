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


WARNING_FLAGS = [
    "-Wall",
    "-Wextra",
    "-Wshadow",
    "-Wdouble-promotion",
    "-Wformat=2",
    "-Wno-unused-parameter",
]


def executable_suffix() -> str:
    return ".exe" if os.name == "nt" else ""


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


def compile_one(cxx: str, source: pathlib.Path, out: pathlib.Path, extra: Iterable[str]) -> None:
    cmd = [
        cxx,
        "-std=c++20",
        "-O2",
        *WARNING_FLAGS,
        "-Isrc",
        "-Itests/native",
        str(source),
        "-o",
        str(out),
        *extra,
    ]
    print("[build]", source.name)
    subprocess.run(cmd, cwd=ROOT, check=True)


def run_one(exe: pathlib.Path) -> None:
    print("[run]", exe.name)
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

    sources = test_sources()
    if not sources:
        print("No native tests found", file=sys.stderr)
        return 1

    executables: list[pathlib.Path] = []
    try:
        for source in sources:
            exe = BUILD_DIR / (source.stem + executable_suffix())
            compile_one(cxx, source, exe, args.extra_cxxflag)
            executables.append(exe)

        if not args.build_only:
            for exe in executables:
                run_one(exe)
    except subprocess.CalledProcessError as exc:
        return exc.returncode if exc.returncode else 1
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"OK {len(executables)} standalone test executable(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
