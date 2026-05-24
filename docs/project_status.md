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

- The boot serial settle delay is Debug-profile only (`TRACKER_ENABLE_BOOT_DELAY`). Production and Slim do not keep the old unconditional `sleep(2)`.
- `defines.h` is now a compatibility umbrella over `src/build_config/*`; new profile/config defaults should go into the focused build-config headers.
- `BOARD_LOLIN_C3_MINI_DEBUG` is the normal local build/upload environment while warnings are being kept clean. The old DIAG alias was removed; use the explicit Debug/Production/Slim environments.

## Current quality gate

Run host tests and all profile PlatformIO builds:

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
- LSM6DSV embedded physical tap runtime on INT1 with firmware-side 2..10 tap aggregation, `FUNCTIONS_ENABLE.INTERRUPTS_ENABLE` gating, masked register verification, SlimeVR Tap packet output and serial `tap ...` diagnostics;
- non-blocking GPIO status LED runtime for ESP32-C3 SuperMini (`TRACKER_STATUS_LED_PIN=8`, active-low by default), SlimeVR-style status/error blink patterns and serial `led ...` diagnostics;
- incoming `SetConfigFlag` handling for runtime mag/yaw toggle;
- local serial output decoupled from SlimeVR UDP;
- `slime status` compact view and `slime debug` full counter dump;
- `test runtime <seconds>` for full Wi-Fi/server/FIFO loop-load measurement.

Optimization work should keep using a `test runtime 600` baseline rather than `test static` alone, and firmware-size deltas should be checked with `python tools/report_firmware_size.py`.

## Current setup baseline

The firmware now has a compact user-facing `setup` layer for first-run preparation. It no longer exposes manual `setup rest/accel/mag/axis/temp` wrappers; those jobs belong either to the full guided setup flow or to the lower-level service commands.

Current setup coverage:

- `setup guide` prints the recommended first-run sequence.
- `setup status` reports production, 6DoF, mag-yaw, temperature-model, Wi-Fi, and SlimeVR readiness plus next commands.
- `setup wifi` is an interactive serial Wi-Fi provisioner: scan visible networks, choose one by number, enter password, connect, save successful credentials to NVS, start SlimeVR discovery, and leave Wi-Fi/SlimeVR autostart enabled. If Wi-Fi succeeds but the server is not found in the setup timeout, Wi-Fi remains saved and discovery continues in normal runtime.
- `setup calibration [axis <bodyX> <bodyY> <bodyZ>]` runs a blocking guided production calibration flow: rest/gyro, Wi-Fi heat warm-up, dedicated stationary gyro temperature capture until relative plateau, auto-detected accel 6-position full 3x3 affine calibration, magnetometer hard/soft collection, gyro-assisted automatic mag axis inference with static accel-face cross-check, production tracking feature enable and one final transactional save. Failed/aborted setup calibration restores the previous RAM calibration/config and leaves the previous NVS calibration untouched.
- SlimeVR `SensorInfo.hasCompletedRestCalibration` follows local gyro/rest validity instead of being hardcoded true.

The guided calibration command services FIFO, magnetometer runtime, Wi-Fi and SlimeVR internally while blocking the CLI. Temperature fitting now uses a dedicated setup temperature capture instead of the developer `test static` runner, while reusing the same fit quality gates. Mag hard/soft apply now uses robust full-ellipsoid quality gates: bounded sample reservoir, raw-norm prefilter, geometric outlier rejection, inlier ratio, box coverage, directional coverage, algebraic residual and corrected-norm residual checks. The final setup save captures runtime calibration/config to NVS after production features are enabled. Magnetic axis inference now prefers gyro-assisted motion scoring from the mag motion stage, cross-checks against static accel-face/mag inclination samples when available, and keeps explicit `axis ...` tokens as an override/fallback.


## Calibration implementation notes

- Magnetometer calibration now uses a full ellipsoid fit and stores hard-iron plus a full 3x3 soft-iron matrix when coverage/residual quality gates pass.
- `setup calibration` remains transaction-safe: failed late calibration stages must not overwrite the last saved NVS calibration.

### RC1 serial provisioning compatibility


Battery ADC runtime reads the RC1 divider `BAT+ -> R_TOP -> GPIO -> R_BOTTOM -> GND` with defaults `GPIO4`, `R_TOP=180 kΩ`, and `R_BOTTOM=180 kΩ`. GPIO4 is ESP32-C3 ADC1_CH4, the supported ADC path for battery telemetry. The runtime samples sparsely (`TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS`, default 10000 ms), takes a small median-filtered ADC burst, maps 3.30-4.20 V to 0-100%, and reports safe 0.000 V / 0.0% when the divider is absent, below the present threshold, invalid, or unreadable. The value is exposed through `battery status`, `GET INFO`, `slime status`, and periodic SlimeVR BatteryLevel telemetry.

The firmware now exposes SlimeVR Server serial-compatibility commands for initial provisioning: `SET WIFI`, `SET BWIFI`, `GET INFO`, `GET CONFIG`, `GET TEST`, `GET WIFISCAN`, `REBOOT`, `FRST`, `DELCAL`, and temperature-only `TCAL`. `SET WIFI`/`SET BWIFI` save credentials into the separate network NVS config, enable Wi-Fi/discovery, and restart SlimeVR discovery. Blocking Wi-Fi scans suppress expected FIFO-recovery console noise for a short grace window without disabling recovery, counters, or machine-log events. They are still tracking interruptions: gyro motion during the scan is not reconstructable, but FIFO/AHRS recovery must resume integration afterwards. `ahrs status` exposes `large_dt_rebase_count`, `fifo_rebase_count`, `last_rebase_t_us`, and `post_fifo_recovery_samples` for post-scan diagnostics. FIFO timestamp resets also clear the magnetometer sensor-hub timestamp baseline so mag samples restart from the new IMU stream. `TCAL SAVE` is intentionally scoped to gyro temperature compensation so host-side temperature-calibration commands cannot accidentally capture unrelated runtime output/accel state. `TCAL RESET` changes only the in-RAM temperature compensation slope/quality metadata and does not mutate the persistent config object unless a later explicit temperature save is requested.
