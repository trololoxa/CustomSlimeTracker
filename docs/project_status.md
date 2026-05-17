# Project status

This document replaces the completed code-quality roadmap notes. It records the current structural state of the firmware after the architecture cleanup patches.

## Current structural baseline

- `main.cpp` is an Arduino entrypoint only.
- `app/` owns setup/loop orchestration and app-level singleton wiring.
- `app/hooks/` contains include-only glue sections; hooks must stay wiring-only.
- `runtime/` owns FIFO processing, sample pipeline, static tests, machine logs, output snapshots, runtime bias, mag runtime, and tracking state.
- `sensor/` owns math/model code for AHRS, calibration, quality, magnetometer heading, and yaw correction.
- `serial/` command domains are split into `.hpp/.cpp` pairs; parser glue remains fixed-buffer/no-heap.
- `config/` owns persisted schema, runtime apply/capture/sanitize, NVS store, and printing.
- `connection/` owns low-level hardware/protocol drivers.
- `src/network/` owns real Wi-Fi station management and UDP transport; SlimeVR packet formatting/output runtime lives outside low-level network primitives.

## Deliberate developer conveniences

- Boot-time `sleep(2)` remains intentionally. It gives time to open Serial Monitor during development and can be removed in final production cleanup.
- `defines.h` is the canonical compile-time defaults header. No `defines.hpp` shim is used.
- `BOARD_LOLIN_C3_MINI_DIAG` is the normal local build/upload environment while warnings are being kept clean.

## Current quality gate

Run host tests and PlatformIO builds:

```bash
python tools/check_all.py --clean --require-pio
```

If PlatformIO is not on `PATH`, pass `--pio-bin` or set `PIO`.

Host-only check:

```bash
python tools/check_all.py --clean --skip-pio
```

## Remaining structural work

1. Keep `serial/tracker_serial_context.hpp` lightweight by moving reusable parse/print helpers to focused headers.
2. Maintain `docs/cli_reference.md` when commands or side effects change.
3. Maintain `docs/config_schema.md` when persisted schema changes.
4. Maintain replay/log tooling and add fixtures before major tracking-filter changes.
5. Keep SlimeVR UDP output decoupled from AHRS/FIFO: it must consume prepared output snapshots only, never low-level sensor state directly.

## Current SlimeVR network baseline

The tracker now has a working SlimeVR UDP MVP:

- non-blocking Wi-Fi station manager;
- ESP32-C3 TX power workaround through `TRACKER_WIFI_TX_POWER`;
- UDP discovery and server reconnect;
- SlimeVR protocol v19 metadata;
- `SensorInfo`, `RotationData`, heartbeat, ping/pong, RSSI and temperature telemetry;
- incoming `SetConfigFlag` handling for runtime mag/yaw toggle;
- local serial output decoupled from SlimeVR UDP;
- `slime status` compact view and `slime debug` full counter dump;
- `test runtime <seconds>` for full Wi-Fi/server/FIFO loop-load measurement.

Next optimization work should start from a `test runtime 600` baseline rather than from `test static` alone.

## Current setup baseline

The firmware now has a compact user-facing `setup` layer for first-run preparation. It no longer exposes manual `setup rest/accel/mag/axis/temp` wrappers; those jobs belong either to the full guided setup flow or to the lower-level service commands.

Current setup coverage:

- `setup guide` prints the recommended first-run sequence.
- `setup status` reports production, 6DoF, mag-yaw, temperature-model, Wi-Fi, and SlimeVR readiness plus next commands.
- `setup wifi` is an interactive serial Wi-Fi provisioner: scan visible networks, choose one by number, enter password, connect, save successful credentials to NVS, start SlimeVR discovery, and leave Wi-Fi/SlimeVR autostart enabled. If Wi-Fi succeeds but the server is not found in the setup timeout, Wi-Fi remains saved and discovery continues in normal runtime.
- `setup calibration [axis <bodyX> <bodyY> <bodyZ>]` runs a blocking guided production calibration flow: rest/gyro, Wi-Fi heat warm-up, dedicated stationary gyro temperature capture until relative plateau, auto-detected accel 6-position full 3x3 affine calibration, magnetometer hard/soft collection, automatic mag axis inference, production tracking feature enable and one final transactional save. Failed/aborted setup calibration restores the previous RAM calibration/config and leaves the previous NVS calibration untouched.
- SlimeVR `SensorInfo.hasCompletedRestCalibration` follows local gyro/rest validity instead of being hardcoded true.

The guided calibration command services FIFO, magnetometer runtime, Wi-Fi and SlimeVR internally while blocking the CLI. Temperature fitting now uses a dedicated setup temperature capture instead of the developer `test static` runner, while reusing the same fit quality gates. Mag hard/soft apply still uses the existing magnetometer quality gates. The final setup save captures runtime calibration/config to NVS after production features are enabled. Magnetic axis inference now uses accel-face samples and simultaneous raw mag samples to score signed-axis permutations; explicit `axis ...` tokens remain available as an override/fallback.


## Calibration implementation notes

- Magnetometer calibration now uses a full ellipsoid fit and stores hard-iron plus a full 3x3 soft-iron matrix when coverage/residual quality gates pass.
- `setup calibration` remains transaction-safe: failed late calibration stages must not overwrite the last saved NVS calibration.
