#!/usr/bin/env python3
"""Integration guards for 0028d; behavioral cases run in the native suite."""
from __future__ import annotations
import ast
import os
from pathlib import Path
import shutil
import subprocess
from quality_gate_runtime import quality_gate_environment, project_temp_directory
from test_calibration_storage_stack_policy import function_body, parse_stack_usage, require_limit

ROOT = Path(__file__).resolve().parents[1]
def read(path): return (ROOT / path).read_text(encoding="utf-8")
def require(text, needle):
    if needle not in text: raise SystemExit(f"0028d missing contract: {needle}")

def main():
    app = read("src/app/tracker_app.cpp")
    recovery = read("src/runtime/sensor_recovery_controller.hpp")
    setup = read("src/serial/tracker_setup_commands.cpp")
    tx = read("src/serial/tracker_calibration_transaction.hpp")
    capture = read("src/sensor/fifo_calibrations.cpp")
    require(recovery, "AwaitingProgress")
    require(recovery, "recordFailure(nowMs)")
    require(app, "sensorRecovery_.confirmProgress(millis())")
    require(app, "preserveOrientation ? recoveryTimestampUs_ : 0u")
    require(app, "!fifoCalibrationCaptureActive_ && sensorRuntimeReady_")
    require(capture, "FifoCalibrationCaptureSession& session")
    require(capture, "std::min<uint32_t>(20u")
    require(capture, "now() - startedMs_ >= maximumMs_")
    require(capture, "raw.t_us > lastSampleUs_")
    require(tx, "tempSnapshot")
    require(tx, "*ctx.runtimeBias = runtimeBiasSnapshot")
    require(tx, "setAutonomyProbationWriteBarrier(true)")
    body = function_body(tx, "bool commit(")
    assert body.index("prepareAuthoritativeCommit") < body.index("*ctx.config = candidate")
    assert body.index("trackerSerialVerifyCalibrationCandidate") < body.index("commitPreparedAuthoritative")
    for file in ("tracker_calibration_commands.cpp", "tracker_config_commands.cpp", "tracker_mag_commands.cpp"):
        require(read("src/serial/" + file), "trackerSerialCommitCalibrationCandidate")
    require(setup, "setupVerificationMayRetry")
    require(setup, "setupVerificationDeadlineReached")
    attempt = function_body(setup, "bool setupVerifyOutputAttempt")
    if "setupVerifyOutputAttempt(" in attempt or "setupVerifyOutputRuntime(" in attempt:
        raise SystemExit("0028d verification must not recurse")
    require(setup, "for (uint8_t attempt = 0u; attempt < 2u;")
    # The normal native millis()==0 stub eliminates these loops. Use an opaque
    # clock for stack analysis so this gate measures real verification frames.
    cxx = os.environ.get("CXX") or shutil.which("g++") or shutil.which("c++")
    if not cxx: raise SystemExit("0028d stack gate needs a C++ compiler")
    with project_temp_directory(ROOT, "tracker-0028d-") as name:
        tmp = Path(name)
        arduino = read("tests/native/Arduino.h")
        start = arduino.index("inline uint32_t millis()")
        end = arduino.index("\n}", start) + 2
        (tmp / "Arduino.h").write_text(arduino[:start] + "uint32_t millis();" + arduino[end:])
        subprocess.run([cxx, "-std=c++20", "-O2", "-fstack-usage", "-I", str(tmp),
                        "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"), "-c",
                        str(ROOT / "src/serial/tracker_setup_commands.cpp"), "-o", str(tmp / "setup.o")],
                       check=True, cwd=ROOT, env=quality_gate_environment(ROOT, scope="0028d"))
        usage = parse_stack_usage(tmp)
        require_limit(usage, "setupVerifyOutputAttempt", 2048)
        require_limit(usage, "setupVerifyOutputRuntime", 384)
        require_limit(usage, "TrackerCalibrationTransaction::commit", 512)
        print("# 0028d opaque-clock stack:", {k:v for k,v in usage.items() if "setupVerifyOutput" in k})
    ast.parse(read("tools/check_all.py"))
    print("# 0028d_recovery_and_calibration_contract_policy: PASS")
    return 0
if __name__ == "__main__": raise SystemExit(main())
