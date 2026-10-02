#!/usr/bin/env python3
"""Guard magnetic_callback_abi cross-ABI magnetic callback stack and API contracts."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, parse_stack_usage, require, require_limit, run_contract_command,
)


from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]


def process_body(controller: str) -> str:
    marker = "TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::processRawSample("
    start = controller.find(marker)
    if start < 0:
        raise SystemExit("magnetic_callback_abi policy could not find processRawSample noinline definition")
    end = controller.find("\nStream& MagRuntimeController::stream() const", start)
    if end < 0:
        raise SystemExit("magnetic_callback_abi policy could not isolate processRawSample body")
    return controller[start:end]


def compile_stack(cxx: str, optimization: str, tmp: Path) -> dict[str, int]:
    out_dir = tmp / optimization.replace("-", "")
    out_dir.mkdir(parents=True, exist_ok=True)
    run_contract_command(
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
            f"magnetic_callback_abi processRawSample must capture one coherent millis() value, found {body.count('millis()')}"
        )
    if body.count("runtimeConfig()") != 1:
        raise SystemExit(
            "magnetic_callback_abi processRawSample must build MagRuntimeConfig exactly once per sample"
        )
    forbid(body, "MagFieldReliabilityInput", "large reliability input in orchestrator frame")
    forbid(body, "MagYawCorrectionInput", "large yaw input in orchestrator frame")

    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-magnetic_callback_abi-") as tmp_name:
        tmp = Path(tmp_name)
        for optimization in ("-O2", "-Os"):
            usage = compile_stack(cxx, optimization, tmp)
            require_limit(usage, "MagRuntimeController::processRawSample", 512)
            require_limit(usage, "captureGyroEndpoint", 192)
            require_limit(usage, "updateHeadingSnapshot", 256)
            require_limit(usage, "updateFieldReliabilitySnapshot", 640)
            require_limit(usage, "updateYawCorrectionSnapshot", 640)

    print("# magnetic_callback_abi: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
