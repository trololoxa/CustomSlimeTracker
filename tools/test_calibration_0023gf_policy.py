#!/usr/bin/env python3
"""Guard 0023gf cross-ABI magnetic callback stack and API contracts."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def forbid(text: str, needle: str, description: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {description}: {needle}")


def compiler() -> str:
    explicit = os.environ.get("CXX")
    if explicit and shutil.which(explicit):
        return explicit
    for name in ("g++", "clang++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    raise SystemExit("no C++ compiler available for 0023gf policy")


def parse_stack_usage(directory: Path) -> dict[str, int]:
    usage: dict[str, int] = {}
    for path in directory.rglob("*.su"):
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            parts = line.split("\t")
            if len(parts) < 2:
                continue
            try:
                size = int(parts[1])
            except ValueError:
                continue
            name = parts[0].split(":", 3)[-1]
            usage[name] = max(size, usage.get(name, 0))
    return usage


def require_limit(usage: dict[str, int], needle: str, limit: int) -> None:
    matches = [(name, size) for name, size in usage.items() if needle in name]
    if not matches:
        raise SystemExit(f"0023gf stack policy did not find function: {needle}")
    worst_name, worst_size = max(matches, key=lambda item: item[1])
    if worst_size > limit:
        raise SystemExit(
            f"0023gf stack budget exceeded: {worst_name} uses {worst_size} bytes, limit {limit}"
        )


def process_body(controller: str) -> str:
    marker = "TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::processRawSample("
    start = controller.find(marker)
    if start < 0:
        raise SystemExit("0023gf policy could not find processRawSample noinline definition")
    end = controller.find("\nStream& MagRuntimeController::stream() const", start)
    if end < 0:
        raise SystemExit("0023gf policy could not isolate processRawSample body")
    return controller[start:end]


def compile_stack(cxx: str, optimization: str, tmp: Path) -> dict[str, int]:
    out_dir = tmp / optimization.replace("-", "")
    out_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            cxx, "-std=c++20", optimization, "-fstack-usage",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
            "-c", str(ROOT / "src/runtime/mag_runtime_controller.cpp"),
            "-o", str(out_dir / "controller.o"),
        ],
        check=True,
    )
    return parse_stack_usage(out_dir)


def main() -> int:
    controller_h = (ROOT / "src/runtime/mag_runtime_controller.hpp").read_text(encoding="utf-8")
    controller = (ROOT / "src/runtime/mag_runtime_controller.cpp").read_text(encoding="utf-8")
    frame = (ROOT / "src/sensor/frame_transform.hpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")
    report = (ROOT / "docs/0023gf_magnetometer_callback_cross_abi_hardening_report.md").read_text(
        encoding="utf-8"
    )

    require(controller, "#define TRACKER_MAG_RUNTIME_NOINLINE __attribute__((noinline))",
            "GCC/Clang noinline boundary")
    require(controller, "TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::captureGyroEndpoint",
            "gyro endpoint phase isolation")
    require(controller, "TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::updateHeadingSnapshot",
            "heading phase isolation")
    require(controller, "TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::updateFieldReliabilitySnapshot",
            "field reliability phase isolation")
    require(controller, "TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::updateYawCorrectionSnapshot",
            "yaw phase isolation")
    require(controller, "TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::processRawSample",
            "orchestrator noinline boundary")
    require(controller_h, "void captureGyroEndpoint(MagProcessedSample& processed) const;",
            "private gyro endpoint phase")
    require(controller_h, "void updateFieldReliabilitySnapshot(uint32_t nowMs,",
            "private reliability phase")
    require(controller_h, "void updateYawCorrectionSnapshot(uint32_t nowMs,",
            "private yaw phase")

    body = process_body(controller)
    if body.count("millis()") != 1:
        raise SystemExit(
            f"0023gf processRawSample must capture one coherent millis() value, found {body.count('millis()')}"
        )
    if body.count("runtimeConfig()") != 1:
        raise SystemExit(
            "0023gf processRawSample must build MagRuntimeConfig exactly once per sample"
        )
    forbid(body, "MagFieldReliabilityInput", "large reliability input in orchestrator frame")
    forbid(body, "MagYawCorrectionInput", "large yaw input in orchestrator frame")
    require(controller, "inverseApplyAcceptedSensorToDevice", "controller-local accepted inverse")
    forbid(frame, "inverseApplyValidatedSensorToDevice", "public unchecked inverse API")
    require(report, "MSYS2", "Windows cross-ABI defect report")
    require(check_all,
            '("tools/test_calibration_0023gf_policy.py", "0023gf mag callback cross-ABI policy")',
            "aggregate runner entry")

    cxx = compiler()
    with tempfile.TemporaryDirectory(prefix="tracker-0023gf-") as tmp_name:
        tmp = Path(tmp_name)
        for optimization in ("-O2", "-Os"):
            usage = compile_stack(cxx, optimization, tmp)
            require_limit(usage, "MagRuntimeController::processRawSample", 512)
            require_limit(usage, "captureGyroEndpoint", 192)
            require_limit(usage, "updateHeadingSnapshot", 256)
            require_limit(usage, "updateFieldReliabilitySnapshot", 640)
            require_limit(usage, "updateYawCorrectionSnapshot", 640)

    subprocess.run([sys.executable, str(ROOT / "tools/test_calibration_0023ge_policy.py")], check=True)
    print("# calibration_0023gf_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
