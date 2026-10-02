#!/usr/bin/env python3
"""Guard full guided-setup and gyro-temperature reliability hardening."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, parse_stack_usage, require, require_limit, run_contract_command,
)


from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]


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

    with project_temp_directory(ROOT, "tracker-guided_setup-stack-") as tmp:
        tmp_path = Path(tmp)
        source = ROOT / "src/runtime/gyro_temp_static_fit.cpp"
        run_contract_command(
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

    print("# guided_setup: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
