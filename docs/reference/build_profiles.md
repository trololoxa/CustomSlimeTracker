# Build profiles

The committed default environment is `BOARD_LOLIN_C3_MINI_PRODUCTION`.

The firmware has four committed compile-time profiles. Select a profile with
`TRACKER_BUILD_PROFILE` in `platformio.ini`; this must stay a build-time choice
because the goal is to remove unused code from the final binary.

## Environments

```bash
pio run -e BOARD_LOLIN_C3_MINI_DEBUG
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
pio run -e BOARD_LOLIN_C3_MINI_SLIM
```

`BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG` is the explicit capture/service image. It
has its own `TRACKER_PROFILE_PRODUCTION_DIAG` identity, uses Production-family
scheduling and includes the complete bounded diagnostic surface needed for
cable-free LOGVER3 capture. It cannot present itself as Production. For local
A/B experiments, pass `-D...` overrides from a private PlatformIO config or a
one-off build command.

The profile validator treats this as an explicit contract: all four build
environments must exist, Production Diagnostic must map to its distinct
profile, and `default_envs` must remain locked-down Production unless policy and
documentation are changed together.

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
basic USB CLI, calibration/config commands, battery runtime and server
telemetry. The TCP listener is compiled out. It
excludes developer-only diagnostics such as machine log, serial stream, `perf`/`motion`, static
tests, runtime tests and boot heartbeat.

Production uses compact status hooks by default. Full runtime and magnetometer
reporters are Debug-only (`TRACKER_ENABLE_DETAILED_RUNTIME_STATUS` and
`TRACKER_ENABLE_DETAILED_MAG_STATUS`). Basic `status`, `health` and `mag status`
still work, but large diagnostic dumps are not linked into the product firmware.
Full `config print` is also Debug-only by default (`TRACKER_ENABLE_FULL_CONFIG_PRINT`);
Production keeps a compact config/network summary for service checks.

## Production Diagnostic

`BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG` is the explicit wearable stress-test and
capture build. It keeps Production-family runtime cadence while compiling the
full CLI, static/runtime tests, machine log, detailed reporters, profiler,
motion diagnostics and bounded TCP transport for the same command dispatcher.

Use it for ankle/thermal/TPS diagnostics:

```bash
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG -t upload
```

Then connect over USB or TCP; both expose the same commands compiled into ProductionDiag. Example cable-free capture:

```bash
python3 tools/capture_telnet_log.py --host <tracker-ip> --seconds 600 \
  --rate 20 --mode full --output logver3_static_clean_001.log
```

TCP has no origin-specific allowlist: setup, persistence, calibration, network
mutation, reset/reboot, full reports and diagnostic control match USB. `log rate`
accepts 1..200 Hz and static/runtime tests accept 1..21600 seconds on either
transport. The listener remains unauthenticated and is therefore a trusted-network
service-image feature only.

Safety note: profiler and motion diagnostic objects are optional diagnostic sinks.
The application no longer treats missing profiler/motion pointers as a boot-blocking
condition, and `motion` records samples only while explicitly enabled with
`motion on`. This keeps source-filter/profile drift from turning into a watchdog
reset loop before the serial banner is printed.

## USB diagnostic bench variant

`BOARD_LOLIN_C3_MINI_USB_DIAG` inherits ProductionDiag, including its CLI and
scheduling, but disables manual/automatic motion light sleep with
`TRACKER_ENABLE_MOTION_LIGHT_SLEEP=0` to keep the bench USB endpoint available.
It cannot certify sleep/wake. It is an optional environment, not a fifth compile-time profile or
an extra default release build. Build it explicitly for native USB health smoke:

```powershell
& $TrackerPio run -e BOARD_LOLIN_C3_MINI_USB_DIAG
```

Its serial output queue is 8192 bytes instead of 1536 (+6656 bytes static RAM).
The Wi-Fi queue remains 8192 bytes; record size, bounded drop behavior and serial
drain budget (48 bytes per 4 ms opportunity) remain unchanged. This absorbs a
full health burst; it does not guarantee delivery under unlimited output or a
stalled USB host. No new CLI command or synchronous console write is introduced.
Normal ProductionDiag remains the cable-free capture profile.

The base already defines `ARDUINO_USB_MODE=1` and `ARDUINO_USB_CDC_ON_BOOT=1`.
The bench environment explicitly sets `board_upload.before_reset=usb_reset` and
`board_upload.after_reset=hard_reset`, with esptool at its initial 115200 baud. This avoids the v4 `CHANGE_BAUDRATE`
command that failed to receive a reply on the Windows native USB bench. Pinned
PlatformIO espressif32 6.7.0 reads both settings, but its esptool 4.5.1 still failed
on this bench. Controlled flashing uses a separate verified esptool 4.9.0 environment;
the direct PlatformIO upload target still uses its old pinned tool.
This targets ESP32-C3 native USB Serial/JTAG (303A:1001), not an external UART.
A responsive native USB path can enter ROM download and restart without buttons.
Disabled/remapped USB, sleep, port ownership or driver problems can prevent it;
read-back proves written bytes; boot/pose health needs a separate smoke run.
See [device workflow](../development/device_smoke.md) for app-only flashing.

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
- active calibration still uses the 0021 dual-slot/selector store and legacy migration, but the 876-byte RAM candidate cache and interactive candidate staging/promotion are compiled out;
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
`docs/reference/profile_contract.md` and `docs/reference/source_filters.md` before adding new
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
Diagnostic and Slim and stores raw reports under `build/firmware_size/`. All
profiles share `partitions/tracker_4mb_no_ota.csv`: one 3 MiB factory app on the
tracker's 4 MiB flash. The project does not implement OTA, so the Arduino
default two-slot layout must not be used. Any profile size overflow is fatal.

`BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK` is an internal validation environment, not
an upload/product profile. It inherits the complete Debug source set, warning
flags and the same production partition contract, and exists only as an explicit
compile/type/link-symbol gate in `check_all`.

After changing from an older OTA partition, perform a normal PlatformIO upload
that writes `partitions.bin`; copying only `firmware.bin` is insufficient because
the bootloader would still use the old flash layout.

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
