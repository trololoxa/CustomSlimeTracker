#!/usr/bin/env python3
"""Validate the firmware build-profile/source-filter contract.

`validate_source_filters.py` checks whether concrete filter paths exist. This
script checks the semantic contract represented by `platformio.ini`: all
committed environments must exist, the committed default must remain the
wearable Production Diagnostic environment, profile flags must match, and
product source filters must keep quality-critical tracking/network modules.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PLATFORMIO_INI = ROOT / "platformio.ini"
CONFIG_RUNTIME_CPP = ROOT / "src" / "config" / "tracker_config_runtime.cpp"

ENV_RE = re.compile(r"^\s*\[env:([^\]]+)\]\s*$")
EXCLUDE_RE = re.compile(r"^\s*-<([^>]+)>\s*(?:[;#].*)?$")
PROFILE_RE = re.compile(r"TRACKER_BUILD_PROFILE=(TRACKER_PROFILE_[A-Z_]+)")
DEFAULT_ENVS_RE = re.compile(r"^\s*default_envs\s*=\s*(.+?)\s*(?:[;#].*)?$")

BASE_ENV = "BOARD_LOLIN_C3_MINI"
DEBUG_ENV = "BOARD_LOLIN_C3_MINI_DEBUG"
DEBUG_LINKCHECK_ENV = "BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK"
PRODUCTION_ENV = "BOARD_LOLIN_C3_MINI_PRODUCTION"
PRODUCTION_DIAG_ENV = "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG"
SLIM_ENV = "BOARD_LOLIN_C3_MINI_SLIM"

EXPECTED_DEFAULT_ENVS = (PRODUCTION_DIAG_ENV,)
EXPECTED_ENVIRONMENTS = {
    BASE_ENV,
    DEBUG_ENV,
    DEBUG_LINKCHECK_ENV,
    PRODUCTION_ENV,
    PRODUCTION_DIAG_ENV,
    SLIM_ENV,
}
EXPECTED_PROFILE_FLAGS = {
    DEBUG_ENV: "TRACKER_PROFILE_DEBUG",
    PRODUCTION_ENV: "TRACKER_PROFILE_PRODUCTION",
    PRODUCTION_DIAG_ENV: "TRACKER_PROFILE_PRODUCTION",
    SLIM_ENV: "TRACKER_PROFILE_SLIM",
}

# Exclusions shared by normal Production and Production Diagnostic. The
# diagnostic environment intentionally keeps the live profiler/motion modules.
PRODUCT_COMMON_REQUIRED_EXCLUDES = {
    "config/tracker_config_print.cpp",
    "runtime/machine_log_runtime.cpp",
    "runtime/mag_status_reporter.cpp",
    "runtime/runtime_status_reporter.cpp",
    "runtime/runtime_test_runner.cpp",
    "runtime/static_test_runner.cpp",
    "serial/tracker_ahrs_commands.cpp",
    "serial/tracker_bias_commands.cpp",
    "serial/tracker_imu_fifo_commands.cpp",
    "serial/tracker_output_commands.cpp",
    "serial/tracker_test_commands.cpp",
}

LIVE_DIAGNOSTIC_SOURCES = {
    "runtime/runtime_profiler.cpp",
    "runtime/runtime_motion_diagnostics.cpp",
    "serial/tracker_perf_commands.cpp",
    "serial/tracker_motion_commands.cpp",
}

PRODUCTION_REQUIRED_EXCLUDES = PRODUCT_COMMON_REQUIRED_EXCLUDES | LIVE_DIAGNOSTIC_SOURCES
PRODUCTION_DIAG_REQUIRED_EXCLUDES = PRODUCT_COMMON_REQUIRED_EXCLUDES

SLIM_REQUIRED_EXCLUDES = PRODUCTION_REQUIRED_EXCLUDES | {
    "config/tracker_config_calibration_capture.cpp",
    "app/tracker_command_wiring.cpp",
    "network/wifi_remote_console.cpp",
    "runtime/status_led_runtime.cpp",
    "runtime/tap_accumulator.cpp",
    "runtime/tap_runtime_controller.cpp",
    "runtime/tracker_console_suppress.cpp",
    "runtime/gyro_temp_calibration_capture.cpp",
    "runtime/gyro_temp_static_fit.cpp",
    "sensor/accel_6pos_calibration.cpp",
    "sensor/fifo_calibrations.cpp",
    "serial/tracker_battery_commands.cpp",
    "serial/tracker_calibration_commands.cpp",
    "serial/tracker_config_commands.cpp",
    "serial/tracker_fifo_config_control.cpp",
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
# profiles. If one of these appears in a filter, the profile is no longer a
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


def parse_default_envs(raw: str | None) -> tuple[str, ...]:
    if raw is None:
        return ()
    return tuple(part for part in re.split(r"[\s,]+", raw.strip()) if part)


def parse_platformio() -> tuple[set[str], dict[str, set[str]], dict[str, str], tuple[str, ...]]:
    current_env = "<global>"
    environments: set[str] = set()
    excludes: dict[str, set[str]] = {}
    profiles: dict[str, str] = {}
    default_envs_raw: str | None = None

    for line in PLATFORMIO_INI.read_text(encoding="utf-8").splitlines():
        env_match = ENV_RE.match(line)
        if env_match:
            current_env = env_match.group(1)
            environments.add(current_env)
            excludes.setdefault(current_env, set())
            continue

        if current_env == "<global>":
            default_match = DEFAULT_ENVS_RE.match(line)
            if default_match:
                default_envs_raw = default_match.group(1).strip()

        exclude_match = EXCLUDE_RE.match(line)
        if exclude_match:
            excludes.setdefault(current_env, set()).add(exclude_match.group(1))

        profile_match = PROFILE_RE.search(line)
        if profile_match:
            profiles[current_env] = profile_match.group(1)

    return environments, excludes, profiles, parse_default_envs(default_envs_raw)


def check_required_subset(errors: list[str], env: str, actual: set[str], required: set[str]) -> None:
    missing = sorted(required - actual)
    if missing:
        errors.append(f"{env}: missing required source-filter excludes:\n  " + "\n  ".join(missing))


def check_forbidden_core(errors: list[str], env: str, actual: set[str]) -> None:
    forbidden = sorted(actual & MUST_KEEP_IN_PRODUCT_PROFILES)
    if forbidden:
        errors.append(
            f"{env}: must not exclude tracking/network core files:\n  " + "\n  ".join(forbidden)
        )


def main() -> int:
    environments, excludes, profiles, default_envs = parse_platformio()
    errors: list[str] = []
    platformio_text = PLATFORMIO_INI.read_text(encoding="utf-8")

    missing_envs = sorted(EXPECTED_ENVIRONMENTS - environments)
    if missing_envs:
        errors.append("[platformio] missing committed environments:\n  " + "\n  ".join(missing_envs))

    if default_envs != EXPECTED_DEFAULT_ENVS:
        errors.append(
            "[platformio] default_envs should be "
            f"{','.join(EXPECTED_DEFAULT_ENVS)}, got {default_envs!r}"
        )
    for env in default_envs:
        if env not in environments:
            errors.append(f"[platformio] default environment does not exist: {env}")

    linkcheck_match = re.search(
        rf"^\[env:{re.escape(DEBUG_LINKCHECK_ENV)}\]\s*$([\s\S]*?)(?=^\[|\Z)",
        platformio_text,
        re.MULTILINE,
    )
    if linkcheck_match is None:
        errors.append(f"[platformio] missing internal Debug link-check section: {DEBUG_LINKCHECK_ENV}")
    else:
        linkcheck_section = linkcheck_match.group(1)
        if f"extends = env:{DEBUG_ENV}" not in linkcheck_section:
            errors.append(f"{DEBUG_LINKCHECK_ENV}: must extend {DEBUG_ENV}")
        if "board_build.partitions = partitions/debug_linkcheck.csv" not in linkcheck_section:
            errors.append(f"{DEBUG_LINKCHECK_ENV}: missing dedicated debug_linkcheck partition")
        if "board_upload.maximum_size = 3145728" not in linkcheck_section:
            errors.append(f"{DEBUG_LINKCHECK_ENV}: unexpected link-check maximum app size")

    if "extra_scripts = pre:tools/generate_build_identity.py" not in platformio_text:
        errors.append("[platformio] automatic build identity pre-script is missing")
    if not (ROOT / "tools" / "generate_build_identity.py").exists():
        errors.append("[platformio] build identity pre-script path does not exist")
    if not (ROOT / "partitions" / "debug_linkcheck.csv").exists():
        errors.append("[platformio] debug link-check partition file does not exist")

    for env, expected in EXPECTED_PROFILE_FLAGS.items():
        actual = profiles.get(env)
        if actual != expected:
            errors.append(f"{env}: expected -DTRACKER_BUILD_PROFILE={expected}, got {actual!r}")

    debug_excludes = excludes.get(DEBUG_ENV, set())
    production_excludes = excludes.get(PRODUCTION_ENV, set())
    production_diag_excludes = excludes.get(PRODUCTION_DIAG_ENV, set())
    slim_excludes = excludes.get(SLIM_ENV, set())

    # Debug is the full diagnostic build and must not exclude project modules.
    if debug_excludes:
        errors.append(f"{DEBUG_ENV}: Debug profile should not exclude source files: {sorted(debug_excludes)}")

    check_required_subset(errors, PRODUCTION_ENV, production_excludes, PRODUCTION_REQUIRED_EXCLUDES)
    check_required_subset(
        errors,
        PRODUCTION_DIAG_ENV,
        production_diag_excludes,
        PRODUCTION_DIAG_REQUIRED_EXCLUDES,
    )
    check_required_subset(errors, SLIM_ENV, slim_excludes, SLIM_REQUIRED_EXCLUDES)

    # The config runtime is linked in every profile, while the six-position
    # calibration implementation is intentionally removed from Slim. Keep the
    # capture adapter in the same profile boundary so tracker_config_runtime.cpp
    # never carries unresolved calibration-only references into the Slim link.
    accel_impl = "sensor/accel_6pos_calibration.cpp"
    accel_capture_adapter = "config/tracker_config_calibration_capture.cpp"
    for env, actual in excludes.items():
        if accel_impl in actual and accel_capture_adapter not in actual:
            errors.append(
                f"{env}: excluding {accel_impl} also requires excluding "
                f"{accel_capture_adapter}"
            )

    config_runtime_text = CONFIG_RUNTIME_CPP.read_text(encoding="utf-8")
    if "Accel6PosCalibration::" in config_runtime_text:
        errors.append(
            "tracker_config_runtime.cpp must not reference Accel6PosCalibration methods; "
            "keep calibration-only capture code in tracker_config_calibration_capture.cpp"
        )

    fifo_basic_control = "serial/tracker_fifo_config_control.cpp"
    for env, actual in ((PRODUCTION_ENV, production_excludes),
                        (PRODUCTION_DIAG_ENV, production_diag_excludes)):
        if fifo_basic_control in actual:
            errors.append(
                f"{env}: Production-safe FIFO control must remain linked: "
                f"{fifo_basic_control}"
            )

    accidentally_removed_diag = sorted(production_diag_excludes & LIVE_DIAGNOSTIC_SOURCES)
    if accidentally_removed_diag:
        errors.append(
            f"{PRODUCTION_DIAG_ENV}: live diagnostic sources must remain linked:\n  "
            + "\n  ".join(accidentally_removed_diag)
        )

    check_forbidden_core(errors, PRODUCTION_ENV, production_excludes)
    check_forbidden_core(errors, PRODUCTION_DIAG_ENV, production_diag_excludes)
    check_forbidden_core(errors, SLIM_ENV, slim_excludes)

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    print(
        "# validate_profile_matrix: OK "
        f"(default={default_envs[0]}, DebugLinkcheck=yes, Production required={len(PRODUCTION_REQUIRED_EXCLUDES)}, "
        f"ProductionDiag required={len(PRODUCTION_DIAG_REQUIRED_EXCLUDES)}, "
        f"Slim required={len(SLIM_REQUIRED_EXCLUDES)})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
