# Build profiles

The committed default environment is `BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG`.

The firmware has three committed compile-time profiles. Select a profile with
`TRACKER_BUILD_PROFILE` in `platformio.ini`; this must stay a build-time choice
because the goal is to remove unused code from the final binary.

## Environments

```bash
pio run -e BOARD_LOLIN_C3_MINI_DEBUG
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
pio run -e BOARD_LOLIN_C3_MINI_SLIM
```

The committed matrix also contains one service environment,
`BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG`. It is not a fourth product profile: it
uses `TRACKER_PROFILE_PRODUCTION` plus `TRACKER_ENABLE_RUNTIME_PROFILER=1` so a
wearable tracker can expose `perf`/`motion` over serial/telnet without linking
the full Debug profile. It is also the committed default environment in
`platformio.ini`, because it is the normal on-device diagnostic baseline for the
`Upgrades` branch. For other local A/B experiments, pass `-D...` overrides from
a private PlatformIO config or a one-off build command.

The profile validator treats this as an explicit contract: all four build
environments must exist, Production Diagnostic must map to
`TRACKER_PROFILE_PRODUCTION`, and `default_envs` must remain the committed
wearable diagnostic environment unless the policy and documentation are changed
together.

## Debug

Debug is the full development profile. It keeps an explicit `build_src_filter = +<*>`
so local/inherited source filters cannot accidentally drop core translation units.
It keeps the serial console, full CLI, static/runtime tests, serial stream,
machine log, boot heartbeat, LED runtime, tap runtime, battery runtime, Wi-Fi
remote console, SlimeVR serial compatibility, `perf`/`motion` live diagnostics and all diagnostic commands enabled.
It is the only profile that keeps the boot serial settle delay.

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
excludes developer-only diagnostics such as machine log, serial stream, `perf`/`motion`, static
tests, runtime tests and boot heartbeat.

Production uses compact status hooks by default. Full runtime and magnetometer
reporters are Debug-only (`TRACKER_ENABLE_DETAILED_RUNTIME_STATUS` and
`TRACKER_ENABLE_DETAILED_MAG_STATUS`). Basic `status`, `health` and `mag status`
still work, but large diagnostic dumps are not linked into the product firmware.
Full `config print` is also Debug-only by default (`TRACKER_ENABLE_FULL_CONFIG_PRINT`);
Production keeps a compact config/network summary for service checks.

## Production Diagnostic

`BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG` is the recommended wearable stress-test
build when normal Production boots but full Debug is too heavy or unstable. It
keeps the Production feature policy and source-filter exclusions, but leaves in:

- `runtime/runtime_profiler.cpp`;
- `runtime/runtime_motion_diagnostics.cpp`;
- `serial/tracker_perf_commands.cpp`;
- `serial/tracker_motion_commands.cpp`.

Use it for ankle/thermal/TPS diagnostics:

```bash
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG -t upload
```

Then connect over serial or telnet and run:

```text
perf on
motion on
perf status
motion status
```

This environment is intentionally closer to Production than Debug: no machine
log, no serial stream, no static/runtime test runners and no boot heartbeat.

Safety note: profiler and motion diagnostic objects are optional diagnostic sinks.
The application no longer treats missing profiler/motion pointers as a boot-blocking
condition, and `motion` records samples only while explicitly enabled with
`motion on`. This keeps source-filter/profile drift from turning into a watchdog
reset loop before the serial banner is printed.

## Slim

Slim assumes the tracker has already been provisioned and calibrated in NVS. It
keeps the tracking pipeline, Wi-Fi manager, UDP transport, battery sampling and
the SlimeVR packet set needed for a headless wearable: RotationData, PingPong
responses, SignalStrength/RSSI, Temperature and BatteryLevel. It disables the
serial console/CLI, Wi-Fi remote console, boot banner, LED, tap runtime and live
diagnostics.

Slim does not reduce IMU ODR, IMU high-performance modes or AHRS quality. It
reduces power/size by removing service features and by lowering non-tracking
work:

- no Serial/CLI polling;
- no TCP remote console;
- no `perf`/`motion` live diagnostics;
- no LED or tap runtime;
- no setup/calibration/test/config command code;
- battery runtime remains compiled only to feed SlimeVR BatteryLevel telemetry;
- SlimeVR SignalStrength/RSSI and Temperature telemetry remain enabled;
- Slim forces a 125 Hz RotationData target at runtime (`TRACKER_SLIMEVR_FORCE_ROTATION_RATE_HZ=125`) so an older NVS `slime rate` value cannot silently cap TPS;
- Slim clamps a too-large NVS FIFO watermark down to the Slim default (`cfg::FIFO_WATERMARK_WORDS`, currently 9 words) to keep prepared quaternion snapshots refreshing with margin for the 125 TPS sender;
- fewer incoming UDP packets are processed per service update;
- a small network runtime scheduler interval keeps Wi-Fi/UDP state machines responsive without polling them on every high-rate IMU loop.

These settings trade service responsiveness and local diagnostics, not orientation
estimation quality.

## Contract headers

Profile policy lives in three layers:

- `src/build_config/feature_flags.hpp` defines low-level `TRACKER_ENABLE_*` module switches.
- `src/build_config/profile_contract.hpp` defines higher-level `TRACKER_HAS_*` subsystem aliases and validates incompatible combinations.
- `platformio.ini` `build_src_filter` removes whole `.cpp` files that a profile can never use.

New app/runtime code should prefer `TRACKER_HAS_*` aliases. See
`docs/profile_contract.md` and `docs/source_filter_matrix.md` before adding new
profile-specific code.

The profile infrastructure is healthy when the project-contract validators pass:

```bash
python tools/validate_source_filters.py
python tools/validate_profile_matrix.py
python tools/validate_documentation.py
```

## Size report

Use the helper below after profile changes:

```bash
python tools/report_firmware_size.py
```

It runs PlatformIO's `-t size` target for Debug, Production, Production
Diagnostic and Slim and stores raw reports under `build/firmware_size/`.
A size-only overflow of the normal Debug partition is advisory; product profile
failures remain fatal.

`BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK` is an internal validation environment, not
an upload/product profile. It inherits the complete Debug source set and warning
flags but uses `partitions/debug_linkcheck.csv` so `check_all` can distinguish a
known wearable flash-size limit from real compile/type/link-symbol breakage.

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

### SlimeVR runtime link unit

PlatformIO builds compile the SlimeVR output runtime through
`src/slimevr_output_runtime_link_unit.cpp`. The original nested file
`src/runtime/slimevr_output_runtime.cpp` remains as a native-test/non-PlatformIO
wrapper, but PlatformIO defines
`TRACKER_SLIMEVR_OUTPUT_RUNTIME_DISABLE_STANDALONE_TU=1` so only the root link
unit emits the implementation. This avoids `undefined reference to
SlimeVROutputRuntime...` after branch merges or source-filter drift.
