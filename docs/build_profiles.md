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

## Slim

Slim assumes the tracker has already been provisioned and calibrated in NVS. It
keeps the tracking pipeline, Wi-Fi manager, UDP transport and SlimeVR quaternion
output path, while disabling serial console/CLI, boot banner, LED, tap runtime,
battery runtime and optional telemetry by default.

## Size report

Use the helper below after profile changes:

```bash
python tools/report_firmware_size.py
```

It runs PlatformIO's `-t size` target for Debug, Production and Slim and stores
raw reports under `build/firmware_size/`.
