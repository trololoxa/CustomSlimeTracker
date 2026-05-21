# Build profiles

The firmware is split into three compile-time profiles. Select the profile with
`TRACKER_BUILD_PROFILE` in `platformio.ini`; do not use a runtime setting for
this, because the goal is to remove unused code from the binary.

## Environments

```bash
pio run -e BOARD_LOLIN_C3_MINI_DEBUG
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_SLIM
```

`BOARD_LOLIN_C3_MINI_DIAG` remains as a backward-compatible alias for the Debug
profile.

## Debug

Debug is the full development profile. It keeps the serial console, full CLI,
static/runtime tests, serial stream, machine log, boot heartbeat, LED runtime,
tap runtime, battery runtime, SlimeVR serial compatibility and all diagnostic
commands enabled. It is the only profile that keeps the boot serial settle delay.

## Production

Production keeps user-facing functionality: Wi-Fi/NVS setup, SlimeVR networking,
basic CLI, calibration/config commands, battery runtime and server telemetry. It
defaults off for developer-only diagnostics such as machine log, serial stream,
static tests, runtime tests and boot heartbeat.

Production uses compact status hooks by default. Full runtime and magnetometer
reporters are Debug-only (`TRACKER_ENABLE_DETAILED_RUNTIME_STATUS` and
`TRACKER_ENABLE_DETAILED_MAG_STATUS`). Basic `status`, `health` and `mag status`
still work, but large diagnostic dumps are not linked into the product firmware.
Full `config print` is also Debug-only by default (`TRACKER_ENABLE_FULL_CONFIG_PRINT`);
Production keeps a compact config/network summary for service checks.
Developer FIFO/IMU CLI modules and the static-test gyro-temperature fit
translation unit are also excluded from Production; compact `status`/`health`
still expose the runtime counters needed for service checks.

## Slim

Slim assumes the tracker has already been provisioned and calibrated in NVS. It
keeps the tracking pipeline, Wi-Fi manager, UDP transport and SlimeVR quaternion
output path, while disabling serial console/CLI, boot banner, LED, tap runtime,
battery runtime and optional telemetry by default.


## Contract headers

Profile policy lives in three layers:

- `src/build_config/feature_flags.hpp` defines low-level `TRACKER_ENABLE_*` module switches.
- `src/build_config/profile_contract.hpp` defines higher-level `TRACKER_HAS_*` subsystem aliases and validates incompatible combinations.
- `platformio.ini` `build_src_filter` removes whole `.cpp` files that a profile can never use.

New app/runtime code should prefer `TRACKER_HAS_*` aliases. See `docs/profile_contract.md` and `docs/source_filter_matrix.md` before adding new profile-specific code.

Wave-1 profile infrastructure is considered complete only when both validators pass:

```bash
python tools/validate_source_filters.py
python tools/validate_profile_matrix.py
```

## Size report

Use the helper below after profile changes:

```bash
python tools/report_firmware_size.py
```

It runs PlatformIO's `-t size` target for Debug, Production and Slim and stores
raw reports under `build/firmware_size/`.

## Why `build_src_filter` is still used

Feature flags and `#if` blocks are necessary, but they are not enough for a
firmware-size profile when code lives in separate `.cpp` files under `src/`.
PlatformIO compiles every matching translation unit unless the source filter
excludes it. A disabled dispatcher can therefore leave a command module compiled
anyway, which costs build time, can leave string/data sections available to the
linker, and can produce `-Wunused-function` warnings in Debug.

The intended rule is:

- use `TRACKER_ENABLE_*` flags inside shared files and headers;
- use `build_src_filter` for whole `.cpp` modules that a profile can never use;
- keep both layers aligned, so a module is not compiled in a profile where its
  dispatcher/wiring is disabled.

## Slim power policy

Slim does not reduce IMU ODR, IMU high-performance modes or AHRS quality. It
reduces power by removing service features and by lowering non-tracking work:

- no Serial/CLI polling;
- no LED, tap or battery runtime;
- no server battery/temperature/RSSI telemetry by default;
- slower Wi-Fi status polling and reconnect backoff;
- lower SlimeVR RotationData cap (`TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX`, default
  50 Hz in Slim);
- fewer incoming UDP packets processed per update;
- small network runtime scheduler interval so the Wi-Fi/UDP state machines are
  not polled on every high-rate IMU loop iteration.

These settings trade service responsiveness and packet rate, not orientation
estimation quality. Override them in `platformio.ini` with `-D...` flags for A/B
runtime tests.

### Slim source-level cuts after runtime-context gating

Slim does not just turn CLI/test features off at runtime. Whole translation
units are excluded once their objects are no longer referenced from the global
runtime context:

```text
runtime/tracker_console_suppress.cpp
runtime/gyro_temp_calibration_capture.cpp
runtime/gyro_temp_static_fit.cpp
sensor/accel_6pos_calibration.cpp
sensor/fifo_calibrations.cpp
```

These modules are calibration/setup/console helpers. Slim still applies the
calibration already stored in NVS and keeps the FIFO/AHRS/magnetometer tracking
path enabled; it simply cannot run interactive calibration or suppress console
noise because it has no console.
