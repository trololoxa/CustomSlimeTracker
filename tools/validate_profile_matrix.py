#!/usr/bin/env python3
"""Validate the firmware build-profile/source-filter contract.

This check is deliberately project-specific.  `validate_source_filters.py`
checks whether filter paths exist; this script checks whether the important
Debug/Production/Slim contract is still represented in `platformio.ini`.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PLATFORMIO_INI = ROOT / "platformio.ini"

ENV_RE = re.compile(r"^\s*\[env:([^\]]+)\]\s*$")
EXCLUDE_RE = re.compile(r"^\s*-<([^>]+)>\s*(?:[;#].*)?$")
PROFILE_RE = re.compile(r"TRACKER_BUILD_PROFILE=(TRACKER_PROFILE_[A-Z_]+)")
EXTENDS_RE = re.compile(r"^\s*extends\s*=\s*(.+?)\s*(?:[;#].*)?$")
DEFAULT_ENVS_RE = re.compile(r"^\s*default_envs\s*=\s*(.+?)\s*(?:[;#].*)?$")

DEBUG_ENV = "BOARD_LOLIN_C3_MINI_DEBUG"
PRODUCTION_ENV = "BOARD_LOLIN_C3_MINI_PRODUCTION"
SLIM_ENV = "BOARD_LOLIN_C3_MINI_SLIM"
DIAG_ENV = "BOARD_LOLIN_C3_MINI_DIAG"

EXPECTED_PROFILE_FLAGS = {
    DEBUG_ENV: "TRACKER_PROFILE_DEBUG",
    PRODUCTION_ENV: "TRACKER_PROFILE_PRODUCTION",
    SLIM_ENV: "TRACKER_PROFILE_SLIM",
}

PRODUCTION_REQUIRED_EXCLUDES = {
    "config/tracker_config_print.cpp",
    "runtime/machine_log_runtime.cpp",
    "runtime/mag_status_reporter.cpp",
    "runtime/runtime_status_reporter.cpp",
    "runtime/runtime_test_runner.cpp",
    "runtime/static_test_runner.cpp",
    "runtime/gyro_temp_static_fit.cpp",
    "serial/tracker_ahrs_commands.cpp",
    "serial/tracker_bias_commands.cpp",
    "serial/tracker_imu_fifo_commands.cpp",
    "serial/tracker_output_commands.cpp",
    "serial/tracker_test_commands.cpp",
}

SLIM_REQUIRED_EXCLUDES = PRODUCTION_REQUIRED_EXCLUDES | {
    "app/tracker_command_wiring.cpp",
    "runtime/battery_runtime.cpp",
    "runtime/status_led_runtime.cpp",
    "runtime/tap_accumulator.cpp",
    "runtime/tap_runtime_controller.cpp",
    "runtime/tracker_console_suppress.cpp",
    "runtime/gyro_temp_calibration_capture.cpp",
    "sensor/accel_6pos_calibration.cpp",
    "sensor/fifo_calibrations.cpp",
    "serial/tracker_battery_commands.cpp",
    "serial/tracker_calibration_commands.cpp",
    "serial/tracker_config_commands.cpp",
    "serial/tracker_led_commands.cpp",
    "serial/tracker_mag_commands.cpp",
    "serial/tracker_network_commands.cpp",
    "serial/tracker_serial_commands.cpp",
    "serial/tracker_setup_commands.cpp",
    "serial/tracker_slimevr_commands.cpp",
    "serial/tracker_slimevr_serial_compat_commands.cpp",
    "serial/tracker_tap_commands.cpp",
    "serial/tracker_system_commands.cpp",
}

# Core tracking/network translation units that must not be excluded by product
# profiles.  If one of these appears in a filter, the profile is no longer a
# quality-preserving optimization profile.
MUST_KEEP_IN_PRODUCT_PROFILES = {
    "connection/lsm6dsv_driver.cpp",
    "connection/lsm6dsv_fifo.cpp",
    "runtime/fifo_runtime_processor.cpp",
    "runtime/imu_sample_pipeline.cpp",
    "runtime/slimevr_output_runtime.cpp",
    "runtime/tracking_state_controller.cpp",
    "sensor/ahrs_6dof.cpp",
    "sensor/calibration.cpp",
    "sensor/gyro_temperature_compensation.cpp",
    "sensor/imu_quality.cpp",
    "network/wifi_manager.cpp",
    "network/esp32_wifi_station.cpp",
    "network/esp32_udp_transport.cpp",
    "output/slimevr_packet_writer.cpp",
}


def parse_platformio() -> tuple[dict[str, set[str]], dict[str, str], dict[str, str], str | None]:
    current_env = "<global>"
    excludes: dict[str, set[str]] = {}
    profiles: dict[str, str] = {}
    extends: dict[str, str] = {}
    default_envs: str | None = None

    for line in PLATFORMIO_INI.read_text(encoding="utf-8").splitlines():
        env_match = ENV_RE.match(line)
        if env_match:
            current_env = env_match.group(1)
            excludes.setdefault(current_env, set())
            continue

        if current_env == "<global>":
            default_match = DEFAULT_ENVS_RE.match(line)
            if default_match:
                default_envs = default_match.group(1).strip()

        exclude_match = EXCLUDE_RE.match(line)
        if exclude_match:
            excludes.setdefault(current_env, set()).add(exclude_match.group(1))

        profile_match = PROFILE_RE.search(line)
        if profile_match:
            profiles[current_env] = profile_match.group(1)

        extends_match = EXTENDS_RE.match(line)
        if extends_match:
            extends[current_env] = extends_match.group(1).strip()

    return excludes, profiles, extends, default_envs


def check_required_subset(errors: list[str], env: str, actual: set[str], required: set[str]) -> None:
    missing = sorted(required - actual)
    if missing:
        errors.append(
            f"{env}: missing required source-filter excludes:\n  " + "\n  ".join(missing)
        )


def main() -> int:
    excludes, profiles, extends, default_envs = parse_platformio()
    errors: list[str] = []

    if default_envs != DEBUG_ENV:
        errors.append(f"[platformio] default_envs should be {DEBUG_ENV}, got {default_envs!r}")

    for env, expected in EXPECTED_PROFILE_FLAGS.items():
        actual = profiles.get(env)
        if actual != expected:
            errors.append(f"{env}: expected -DTRACKER_BUILD_PROFILE={expected}, got {actual!r}")

    if extends.get(DIAG_ENV) != f"env:{DEBUG_ENV}":
        errors.append(f"{DIAG_ENV}: expected 'extends = env:{DEBUG_ENV}', got {extends.get(DIAG_ENV)!r}")

    production_excludes = excludes.get(PRODUCTION_ENV, set())
    slim_excludes = excludes.get(SLIM_ENV, set())
    check_required_subset(errors, PRODUCTION_ENV, production_excludes, PRODUCTION_REQUIRED_EXCLUDES)
    check_required_subset(errors, SLIM_ENV, slim_excludes, SLIM_REQUIRED_EXCLUDES)

    forbidden_production = sorted(production_excludes & MUST_KEEP_IN_PRODUCT_PROFILES)
    if forbidden_production:
        errors.append(
            f"{PRODUCTION_ENV}: must not exclude tracking/network core files:\n  " + "\n  ".join(forbidden_production)
        )

    forbidden_slim = sorted(slim_excludes & MUST_KEEP_IN_PRODUCT_PROFILES)
    if forbidden_slim:
        errors.append(
            f"{SLIM_ENV}: must not exclude tracking/network core files:\n  " + "\n  ".join(forbidden_slim)
        )

    # Debug should be the full diagnostic build.  It should not have a source
    # filter that excludes project modules.
    debug_excludes = excludes.get(DEBUG_ENV, set())
    if debug_excludes:
        errors.append(f"{DEBUG_ENV}: Debug profile should not exclude source files: {sorted(debug_excludes)}")

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    print(
        "# validate_profile_matrix: OK "
        f"(Production required={len(PRODUCTION_REQUIRED_EXCLUDES)}, Slim required={len(SLIM_REQUIRED_EXCLUDES)})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
