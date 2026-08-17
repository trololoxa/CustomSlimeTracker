#!/usr/bin/env python3
"""Guard 0026b hot-path optimization without semantic drift."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[1]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(haystack: str, needle: str, label: str) -> None:
    if needle not in haystack:
        raise SystemExit(f"missing {label}: {needle}")


def forbid(haystack: str, needle: str, label: str) -> None:
    if needle in haystack:
        raise SystemExit(f"forbidden {label}: {needle}")


def compiler() -> str:
    explicit = os.environ.get("CXX")
    if explicit and shutil.which(explicit):
        return explicit
    for name in ("g++", "clang++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    raise SystemExit("no C++ compiler available for 0026b stack policy")


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
        raise SystemExit(f"0026b stack policy did not find function: {needle}")
    name, size = max(matches, key=lambda item: item[1])
    if size > limit:
        raise SystemExit(f"0026b stack budget exceeded: {name} uses {size} bytes, limit {limit}")


def compile_stack(cxx: str, optimization: str, tmp: Path) -> dict[str, int]:
    out = tmp / optimization.replace("-", "")
    out.mkdir(parents=True, exist_ok=True)
    for source, obj in (
        ("src/runtime/mag_runtime_controller.cpp", "controller.o"),
        ("src/sensor/mag_field_reliability.cpp", "field.o"),
        ("src/sensor/mag_yaw_correction.cpp", "yaw.o"),
        ("src/runtime/slimevr_output_runtime.cpp", "slime.o"),
    ):
        subprocess.run(
            [
                cxx,
                "-std=c++20",
                optimization,
                "-fstack-usage",
                "-I",
                str(ROOT / "src"),
                "-I",
                str(ROOT / "tests/native"),
                "-c",
                str(ROOT / source),
                "-o",
                str(out / obj),
            ],
            check=True,
        )
    return parse_stack_usage(out)


def main() -> int:
    field_h = text("src/sensor/mag_field_reliability.hpp")
    field = text("src/sensor/mag_field_reliability.cpp")
    horizontal = text("src/sensor/mag_horizontal_trust.hpp")
    yaw_h = text("src/sensor/mag_yaw_correction.hpp")
    controller_h = text("src/runtime/mag_runtime_controller.hpp")
    controller = text("src/runtime/mag_runtime_controller.cpp")
    slime = text("src/runtime/slimevr_output_runtime_impl.inc")
    reporter = text("src/runtime/mag_status_reporter.cpp")
    mag_test = text("tests/native/test_mag_heading_reliability.cpp")
    check_all = text("tools/check_all.py")
    testing = text("docs/testing.md")
    project = text("docs/project_status.md")
    report = text("docs/0026b_hotpath_optimization_report.md")

    # Runtime hot paths must reference coherent snapshots instead of copying the
    # large aggregate inputs introduced by the magnetic/yaw layers.
    require(field_h, "struct MagFieldReliabilityInputView", "field lightweight input view")
    require(yaw_h, "struct MagYawCorrectionInputView", "yaw lightweight input view")
    require(controller, "const MagFieldReliabilityInputView fieldIn", "field runtime input view")
    require(controller, "const MagYawCorrectionInputView yawIn", "yaw runtime input view")
    forbid(controller, "MagFieldReliabilityInput fieldIn;", "large field aggregate on runtime stack")
    forbid(controller, "MagYawCorrectionInput yawIn;", "large yaw aggregate on runtime stack")
    require(controller, "!deps_.lastFieldReliability ||", "runtime field output dependency")
    forbid(controller, "MagFieldReliabilityOutput fallbackReliability;", "field fallback object on sample stack")
    forbid(field_h, "MagFieldReliabilityOutput last_;", "duplicate field output retained inside monitor")
    forbid(field, "last_ = out;", "per-sample duplicate field output copy")
    require(controller, "const YawEnableState yawEnable = yawEnableState();", "single yaw enable-state snapshot per magnetic sample")
    require(controller, "const MagYawCorrectionConfig yawCfg = yawConfig(yawEnable);", "single full yaw config construction")
    forbid(controller, "updateYawCorrectionPhase(nowMs,", "extra yaw phase frame")

    # Horizontal geometry decisions are mathematically equivalent squared
    # comparisons. sqrt is reserved for human diagnostics outside the callback.
    require(horizontal, "headingNoiseScaleSquared", "squared heading geometry scale")
    require(field, "magnitudeAtLeastScaledThreshold", "squared jump threshold")
    require(field, "magnitudeExceedsScaledThreshold", "squared step threshold")
    require(field, "magnitudeAtMostScaledThreshold", "squared recovery threshold")
    forbid(field, "std::sqrt", "sqrt in field reliability hot path")
    require(reporter, "magHeadingNoiseScale(f.headingNoiseScaleSquared)", "lazy diagnostic sqrt")

    # Immutable field tuning is shared as one read-only default instead of being
    # rebuilt/copied or cached per controller. The two persisted yaw thresholds
    # remain explicit scalars in the lightweight view so runtime config changes
    # are still observed immediately without another RAM cache.
    require(controller_h, "const MagFieldReliabilityConfig& fieldReliabilityConfig() const", "field config reference API")
    require(controller, "static const MagFieldReliabilityConfig defaults{};", "shared immutable field tuning")
    require(field_h, "float horizontalNormBad", "view horizontal bad threshold")
    require(field_h, "float horizontalNormGood", "view horizontal good threshold")
    require(controller, "deps_.config->data.magYaw.horizontalNormBad", "persisted horizontal bad wiring")
    require(controller, "deps_.config->data.magYaw.horizontalNormGood", "persisted horizontal good wiring")
    forbid(controller_h, "fieldReliabilityConfigCache_", "per-controller field config cache")

    # Quaternion-only is the default wire policy from 0026. It must return before
    # acceleration conversion; valid accel modes still use the same conversion.
    start = slime.find("void SlimeVROutputRuntime::sendRotation(")
    end = slime.find("\nvoid SlimeVROutputRuntime::makeHandshakeInfo", start)
    if start < 0 or end < 0:
        raise SystemExit("could not isolate sendRotation")
    body = slime[start:end]
    early = body.find("mode == SlimeVRMotionPacketMode::Rotation17Only ||")
    conversion = body.find("const Vec3 accelerationMps2 = slimevr_motion_frame::accelerationWireMps2FromDeviceG(")
    if early < 0 or conversion < 0 or early > conversion:
        raise SystemExit("quaternion-only path must branch before acceleration conversion")
    if body.count("const Vec3 accelerationMps2 = slimevr_motion_frame::accelerationWireMps2FromDeviceG(") != 1:
        raise SystemExit("sendRotation must keep one shared acceleration conversion")

    # Behavioral coverage remains present after the arithmetic refactor.
    require(mag_test, "testThirtyMinuteHighDipNoiseAndDisturbanceRecovery", "long high-dip regression")
    require(mag_test, "testHighDipLegacyHeadingStepGateScalesWithObservability", "high-dip gate regression")
    require(check_all, '("tools/test_0026b_hotpath_optimization_policy.py", "0026b hot-path optimization")', "aggregate gate entry")
    require(testing, "0026b hot-path optimization", "testing documentation")
    require(project, "0026b_hotpath_optimization", "project status entry")
    require(report, "decision stream", "equivalence evidence")

    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-0026b-stack-") as tmp_name:
        tmp = Path(tmp_name)
        for optimization in ("-O2", "-Os"):
            usage = compile_stack(cxx, optimization, tmp)
            # Immediate-predecessor Linux measurements were 144/208-224/112/
            # 224-240/48/192-208 bytes. The previous ceilings left only 16-32
            # bytes of margin and therefore encoded the Linux ABI rather than a
            # cross-ABI regression budget. These ceilings still require the
            # optimized phase split while leaving bounded MSYS2 shadow-space and
            # register-spill margin.
            require_limit(usage, "updateFieldReliabilitySnapshot", 256)
            require_limit(usage, "updateYawCorrectionSnapshot", 352)
            require_limit(usage, "processRawSample", 192)
            require_limit(usage, "MagFieldReliabilityMonitor::update", 384)
            require_limit(usage, "MagYawCorrectionController::update(const tracker::MagYawCorrectionInputView", 96)
            require_limit(usage, "SlimeVROutputRuntime::sendRotation", 320)

    print("# 0026b_hotpath_optimization_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
