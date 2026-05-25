# Build profiles

The firmware has three committed compile-time profiles. Select a profile with
`TRACKER_BUILD_PROFILE` in `platformio.ini`; this must stay a build-time choice
because the goal is to remove unused code from the final binary.

## Environments

```bash
pio run -e BOARD_LOLIN_C3_MINI_DEBUG
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_SLIM
```

No temporary A/B environments are kept in the committed matrix. For local
experiments, pass `-D...` overrides from a private PlatformIO config or a one-off
build command.

## Debug

Debug is the full development profile. It keeps the serial console, full CLI,
static/runtime tests, serial stream, machine log, boot heartbeat, LED runtime,
tap runtime, battery runtime, Wi-Fi remote console, SlimeVR serial compatibility
and all diagnostic commands enabled. It is the only profile that keeps the boot serial settle delay.

Use Debug for:

- feature development;
- `test runtime` and `test static`;
- calibration and setup debugging;
- replay/machine-log capture;
- Wi-Fi/thermal A/B experiments.

## Production

Production keeps user-facing functionality: Wi-Fi/NVS setup, SlimeVR networking,
basic CLI, Wi-Fi remote console for cable-free calibration, calibration/config
commands, battery runtime and server telemetry. It
excludes developer-only diagnostics such as machine log, serial stream, static
tests, runtime tests and boot heartbeat.

Production uses compact status hooks by default. Full runtime and magnetometer
reporters are Debug-only (`TRACKER_ENABLE_DETAILED_RUNTIME_STATUS` and
`TRACKER_ENABLE_DETAILED_MAG_STATUS`). Basic `status`, `health` and `mag status`
still work, but large diagnostic dumps are not linked into the product firmware.
Full `config print` is also Debug-only by default (`TRACKER_ENABLE_FULL_CONFIG_PRINT`);
Production keeps a compact config/network summary for service checks.

## Slim

Slim assumes the tracker has already been provisioned and calibrated in NVS. It
keeps the tracking pipeline, Wi-Fi manager, UDP transport and SlimeVR quaternion
output path, while disabling serial console/CLI, Wi-Fi remote console, boot banner, LED, tap
runtime, battery runtime and optional telemetry by default.

Slim does not reduce IMU ODR, IMU high-performance modes or AHRS quality. It
reduces power/size by removing service features and by lowering non-tracking
work:

- no Serial/CLI polling;
- no TCP remote console;
- no LED, tap or battery runtime;
- no server battery/temperature/RSSI telemetry by default;
- slower Wi-Fi status polling and reconnect backoff;
- lower SlimeVR RotationData cap (`TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX`, default
  50 Hz in Slim);
- fewer incoming UDP packets processed per update;
- a small network runtime scheduler interval so Wi-Fi/UDP state machines are not
  polled on every high-rate IMU loop.

These settings trade service responsiveness and packet rate, not orientation
estimation quality.

## Contract headers

Profile policy lives in three layers:

- `src/build_config/feature_flags.hpp` defines low-level `TRACKER_ENABLE_*` module switches.
- `src/build_config/profile_contract.hpp` defines higher-level `TRACKER_HAS_*` subsystem aliases and validates incompatible combinations.
- `platformio.ini` `build_src_filter` removes whole `.cpp` files that a profile can never use.

New app/runtime code should prefer `TRACKER_HAS_*` aliases. See
`docs/profile_contract.md` and `docs/source_filter_matrix.md` before adding new
profile-specific code.

The profile infrastructure is healthy when both validators pass:

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
linker, and can produce warnings from unused static helpers.

The intended rule is:

- use `TRACKER_ENABLE_*` flags inside shared files and headers;
- use `build_src_filter` for whole `.cpp` modules that a profile can never use;
- keep both layers aligned, so a module is not compiled in a profile where its
  dispatcher/wiring is disabled.
