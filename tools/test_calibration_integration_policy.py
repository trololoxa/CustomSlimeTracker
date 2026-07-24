#!/usr/bin/env python3
"""Guard 0021e calibration-field ownership and epoch transitions."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")

def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"FAIL calibration integration policy: missing {label}: {needle}")

runtime = read("src/config/tracker_config_runtime.cpp")
storage = read("src/config/tracker_config_storage.cpp")
store = read("src/config/tracker_config_store.cpp")
cal = read("src/serial/tracker_calibration_commands.cpp")
setup = read("src/serial/tracker_setup_commands.cpp")
compat = read("src/serial/tracker_slimevr_serial_compat_commands.cpp")
mag = read("src/runtime/mag_runtime_controller.cpp")
standalone = read("tools/run_standalone_tests.py")
version = read("src/build_config/firmware_feature_version.hpp")

require(version, '"c3-6dsv-calibration-epoch-field-safe"', "0021e identity")
require(runtime, "data.frame.sensorToDeviceValid = false", "full clear invalidates frame")
require(runtime, "TrackerAccelCalibrationQualityPersisted{}", "invalid accel clears evidence")
require(runtime, "TrackerGyroTempQualityConfigPersisted{}", "invalid temp model clears evidence")
require(storage, "signature.sensorToDeviceValid", "logical dormant-frame signature")
require(store, "RuntimeSensorSignatureDiverged", "live sensor contract promotion guard")
require(cal, "trackerSerialResetCalibrationWorkspaces", "workspace epoch reset helper")
require(cal, "gyroTempRuntimeModelEqual", "no-op temperature transition detection")
require(setup, "workspaceMask", "stage-scoped workspace cleanup")
require(setup, "magnetometer hardware disable failed", "6DoF hardware transition validation")
require(setup, "sensorToDeviceValid = false", "guided accel/frame atomic readiness")
require(compat, "discardCandidate", "full erase removes candidate")
require(mag, "axisAlignmentValid = false", "field-model change invalidates axis")
require(standalone, 'src/serial/tracker_setup_commands.cpp', "setup compile-only gate")
print("PASS calibration integration policy")
