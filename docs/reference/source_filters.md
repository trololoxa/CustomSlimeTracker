# Source filter matrix

This file documents why whole translation units are excluded from non-Debug
profiles. It is intentionally conservative: tracking, FIFO, AHRS, magnetometer
runtime and SlimeVR UDP output stay compiled unless explicitly listed here.

## Production excludes

| Source | Reason | Replacement / fallback |
|---|---|---|
| `config/tracker_config_print.cpp` | Full config dump is large and developer-oriented. | Compact `printTrackerConfigSummary()` in `tracker_config_print.hpp`. |
| `runtime/machine_log_runtime.cpp` | Machine log is a debug/replay fixture source. | None in Production. |
| `runtime/mag_status_reporter.cpp` | Detailed mag dump is debug-heavy. | Compact mag status hooks. |
| `runtime/runtime_status_reporter.cpp` | Detailed runtime dump is debug-heavy. | Compact `status`/`health` paths. |
| `runtime/runtime_test_runner.cpp` | Runtime tests are diagnostic-only. | Kept by Debug/ProductionDiag. |
| `runtime/runtime_profiler.cpp` | Live loop profiler is service-only. | Kept by Debug/ProductionDiag. |
| `runtime/runtime_motion_diagnostics.cpp` | Per-sample motion profiler is service-only. | Kept by Debug/ProductionDiag. |
| `runtime/static_test_runner.cpp` | Static tests are diagnostic-only. | Kept by Debug/ProductionDiag. |
| `network/wifi_remote_console.cpp` | Unauthenticated TCP listener is not a product feature. | Explicit Debug/ProductionDiag diagnostic image only. |
| `runtime/gyro_temp_static_fit.cpp` | Gyro temperature fit is kept in Production for guided setup; Slim excludes it. | Stored NVS temp compensation is still applied. |
| `serial/tracker_ahrs_commands.cpp` | Developer AHRS commands. | Basic status only. |
| `serial/tracker_bias_commands.cpp` | Developer bias commands. | Calibration/config commands remain. |
| `serial/tracker_imu_fifo_commands.cpp` | Low-level FIFO/IMU debug commands. | Compact FIFO/quality health output. |
| `serial/tracker_output_commands.cpp` | Local serial output/debug stream commands. | SlimeVR UDP output remains. |
| `serial/tracker_perf_commands.cpp` | Live profiler CLI is service-only. | Kept by Debug/ProductionDiag. |
| `serial/tracker_motion_commands.cpp` | Per-sample motion diagnostic CLI is service-only. | Kept by Debug/ProductionDiag. |
| `serial/tracker_test_commands.cpp` | Test commands are diagnostic-only. | Kept by Debug/ProductionDiag. |

## Production Diagnostic

`BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG` uses its distinct
`TRACKER_PROFILE_PRODUCTION_DIAG` identity and an include-all source filter. It
keeps Production-family runtime cadence but links the complete bounded
diagnostic/capture surface: TCP console, machine log, static/runtime tests,
detailed reporters, profiler/motion and their command units.

The profile validator checks both sides: ordinary Production must exclude the
listener and diagnostic modules, while ProductionDiag must retain every source
needed by unattended cable-free capture.

## Slim excludes

Slim excludes everything above plus all serial command modules and UI-only
runtime helpers. Slim assumes Wi-Fi/server/calibration data already exists in
NVS and only keeps the autonomous tracking + SlimeVR path: RotationData,
PingPong responses, SignalStrength/RSSI, Temperature and BatteryLevel.

Additional Slim-only excludes:

| Source | Reason |
|---|---|
| `app/tracker_command_wiring.cpp`, `network/wifi_remote_console.cpp` | No serial CLI in Slim. |
| `runtime/status_led_runtime.cpp` | No status LED runtime in Slim. |
| `runtime/tap_accumulator.cpp` | No tap runtime in Slim. |
| `runtime/tap_runtime_controller.cpp` | No tap runtime in Slim. |
| `runtime/tracker_console_suppress.cpp` | No serial console in Slim. |
| `runtime/gyro_temp_calibration_capture.cpp` | No interactive temp calibration capture in Slim. |
| `sensor/accel_6pos_calibration.cpp` | No interactive accel calibration in Slim. |
| `sensor/fifo_calibrations.cpp` | No interactive FIFO calibration runners in Slim. |
| `serial/*.cpp` command modules | No serial CLI in Slim. |


`runtime/battery_runtime.cpp` is **not** a Slim exclude. It remains linked so
headless Slim builds can feed SlimeVR BatteryLevel telemetry; only the battery
CLI module is excluded.

## Validation

Run:

```bash
python tools/validate_source_filters.py
```

The validator checks that concrete `-<...>` entries in `platformio.ini` still
point to real files and are not duplicated inside an environment. `tools/check_all.py`
runs this check before optional PlatformIO builds.

Run the profile-contract validator as well:

```bash
python tools/validate_profile_matrix.py
```

It checks the product contract, not just path spelling: the committed default
must be Production, all four user-facing profiles plus DebugLinkcheck must
exist with the correct profile flags, Debug must stay unfiltered, Production and Slim must keep the
required diagnostic/UI modules excluded, Production Diagnostic must retain the
complete capture surface, and quality-critical tracking/network translation
units must not be accidentally excluded from product profiles.
