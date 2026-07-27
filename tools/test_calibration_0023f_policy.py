#!/usr/bin/env python3
"""Guard full guided-setup and gyro-temperature reliability hardening."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def compiler() -> str:
    explicit = os.environ.get("CXX")
    if explicit and shutil.which(explicit):
        return explicit
    for name in ("g++", "clang++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    raise SystemExit("no C++ compiler available for 0023f stack policy")


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
        raise SystemExit(f"0023f stack policy did not find function: {needle}")
    worst_name, worst_size = max(matches, key=lambda item: item[1])
    if worst_size > limit:
        raise SystemExit(
            f"0023f stack budget exceeded: {worst_name} uses {worst_size} bytes, limit {limit}"
        )


def main() -> int:
    capture_h = (ROOT / "src/runtime/gyro_temp_calibration_capture.hpp").read_text(encoding="utf-8")
    capture = (ROOT / "src/runtime/gyro_temp_calibration_capture.cpp").read_text(encoding="utf-8")
    fit = (ROOT / "src/runtime/gyro_temp_static_fit.cpp").read_text(encoding="utf-8")
    setup = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    verifier_h = (ROOT / "src/runtime/setup_output_verifier.hpp").read_text(encoding="utf-8")
    verifier = (ROOT / "src/runtime/setup_output_verifier.cpp").read_text(encoding="utf-8")
    pipeline = (ROOT / "src/runtime/imu_sample_pipeline.cpp").read_text(encoding="utf-8")
    capture_test = (ROOT / "tests/native/test_gyro_temp_calibration_capture.cpp").read_text(encoding="utf-8")
    fit_test = (ROOT / "tests/native/test_gyro_temp_static_fit.cpp").read_text(encoding="utf-8")
    verify_test = (ROOT / "tests/native/test_setup_output_verifier.cpp").read_text(encoding="utf-8")
    runner = (ROOT / "tools/run_standalone_tests.py").read_text(encoding="utf-8")

    require(capture_h, "maxGyroMeanStdErrorAxisDps", "temperature mean-precision gate")
    require(capture_h, "gyroThermalConsistencyRejectedWindows", "temperature consistency diagnostics")
    require(capture, "standardError(gyroStdDps, count)", "window standard error")
    require(capture, "maxThermalSlopeDpsPerC", "physically bounded thermal drift")
    require(capture_test, "0.58189f", "reported hardware-noise fixture")
    require(capture_test, "0.55f", "late slow-rotation rejection fixture")

    require(fit, "TempFitWorkspace g_tempFitWorkspace", "setup-only static fit workspace")
    if "TempFitPoint training[STATIC_TEMP_BIN_COUNT]" in fit:
        raise SystemExit("leave-one-bin-out validation must not copy all bins onto task stack")
    require(fit, "heldOut", "copy-free leave-one-bin-out validation")
    require(fit_test, "fitGyroTempFromCompletedStaticTestEx", "capture-to-fit native regression")
    require(runner, 'pathlib.Path("src/runtime/gyro_temp_static_fit.cpp")', "linked gyro temperature fit source")

    require(setup, "Type q then Enter to abort this stage safely.", "temperature-stage abort")
    require(setup, "kNoAcceptedProgressMs", "temperature no-progress fail-fast")
    require(setup, "plateau_window_range_c", "relative plateau diagnostics")
    require(setup, "calibration_6dof_ready", "calibration-only readiness")
    require(setup, "temp_runtime_confident", "temperature runtime confidence status")
    require(setup, "drainSetupInput(ctx);", "CRLF/input cleanup between guided stages")
    require(setup, "stationary_input_passed=", "final stationary input report")
    require(setup, "!ctx.lastCalibratedSample || !ctx.lastImuSampleSequence", "final input dependency gate")

    require(verifier_h, "minimumInputSamples = 256", "bounded final input sample requirement")
    require(verifier_h, "stationaryInputPassed", "stationary verifier result")
    require(verifier, "result.valid =", "final verifier aggregate")
    require(verify_test, "0.45f", "moving-input rejection regression")

    require(pipeline, "const bool gyroTempCaptureActive", "inactive setup capture cold gate")
    require(pipeline, "if (hasCoherentAccel && (staticTestCaptureActive || gyroTempCaptureActive))", "setup transform admission")

    with tempfile.TemporaryDirectory(prefix="tracker-0023f-stack-") as tmp:
        tmp_path = Path(tmp)
        source = ROOT / "src/runtime/gyro_temp_static_fit.cpp"
        subprocess.run(
            [
                compiler(), "-std=c++20", "-O2", "-fstack-usage",
                "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
                "-c", str(source), "-o", str(tmp_path / "fit.o"),
            ],
            check=True,
        )
        usage = parse_stack_usage(tmp_path)
    require_limit(usage, "fitGyroTempFromCompletedStaticTestEx", 768)
    require_limit(usage, "persistTempModelCandidate", 1024)
    require_limit(usage, "leaveOneBinOutValidation", 384)

    schema = (ROOT / "src/config/tracker_config_detail.hpp").read_text(encoding="utf-8")
    storage = (ROOT / "src/config/tracker_config_storage.hpp").read_text(encoding="utf-8")
    require(schema, "CONFIG_VERSION = 2", "unchanged config schema")
    require(storage, "CANDIDATE_VERSION = 3", "unchanged candidate format")

    print("# calibration_0023f_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
