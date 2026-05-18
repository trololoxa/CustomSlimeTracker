# Production firmware roadmap

This roadmap replaces the old development roadmaps with a compact release plan for the
ESP32-C3 + LSM6DSV + QMC6309 SlimeVR tracker firmware.

The goal is not to add every possible experiment. The goal is a reliable, convenient,
battery-aware product firmware with high-quality local 6DoF tracking and safe magnetic
yaw correction.

## 1. Product target

Target hardware:

- ESP32-C3 MCU.
- One LSM6DSV IMU as the primary accelerometer/gyroscope.
- QMC6309 magnetometer through the LSM6DSV sensor hub.
- Optional 1S LiPo battery monitor, LED, button, and charger-status pins when the board
  exposes them.

Target behavior:

- Boot without a serial monitor attached.
- Connect to Wi-Fi and SlimeVR Server automatically after first setup.
- Produce local orientation as a quaternion.
- Send SlimeVR-compatible rotation packets at the configured output rate.
- Use magnetometer data only as a gated yaw correction source.
- Report rest-calibration, battery, temperature, RSSI, and relevant tracking state to the
  host/server.
- Support accumulative 2..10 tap input through the LSM6DSV embedded event engine.
- Keep production hot paths free of debug logging, blocking work, and unnecessary output.

## 2. Release scope

### Must-have for the first production release

- Production/dev/calibration build profiles.
- First-run Wi-Fi provisioning.
- Guided initial calibration.
- Clear setup/status commands.
- Stable 6DoF + gated mag-yaw runtime.
- SlimeVR rest-calibration state reporting.
- LSM6DSV physical tap event support, firmware-side accumulation and SlimeVR tap packet output.
- Battery/temp/RSSI telemetry when the hardware supports it.
- Power profiles and measured power/performance trade-offs.
- Config migration, factory reset, watchdog, and safe fallback behavior.
- Documentation updated in the same patch series as behavior changes.

### Service/calibration tools, not always enabled in production

- Detailed serial diagnostics.
- Machine-readable logs.
- Static/runtime tests.
- Replay metrics.
- Manual calibration override commands.

### Explicitly out of scope for the first production release

These items may be explored later, but they must not block the production firmware:

- BLE provisioning.
- Full mobile setup application.
- Complex captive portal beyond a minimal SoftAP provisioning page.
- Replacing the current AHRS with MEKF/ESKF.
- SFLP as the production orientation source.
- Earth-rotation-level compensation.
- Advanced latency prediction.
- Online magnetic calibration rollback.
- Full PC plotting/viewer application.

## 3. Hardware capability matrix

Add and maintain a small hardware capability table in documentation and in firmware status
output.

Required capabilities:

- LSM6DSV WHO_AM_I check.
- LSM6DSV FIFO with hardware timestamp reconstruction.
- QMC6309 presence check through the sensor hub.
- One interrupt line for FIFO/events.
- Wi-Fi station mode.

Optional capabilities:

- Battery ADC input.
- Battery divider enable pin.
- Charger status pin.
- User button.
- Status LED.
- Additional interrupt pin.

Firmware requirements:

- Unsupported optional hardware must be reported as `unsupported`, not as a fault.
- Features depending on missing hardware must compile out or remain disabled.
- `setup status` must show which optional capabilities are available.

## 4. Build profiles and production hot path

Add explicit firmware profiles:

- `dev`: full CLI, streams, logs, tests, verbose boot diagnostics.
- `calibration`: CLI and calibration logs enabled, runtime output enabled, minimal extra spam.
- `production`: no periodic serial spam, no stream by default, no heavy tests in the hot path.

Production requirements:

- No boot delay unless explicitly requested.
- No float formatting in the normal sample/network loop.
- No dynamic allocation in the sample path.
- No machine logs unless enabled.
- No serial stream unless enabled.
- Network output consumes a prepared runtime snapshot instead of reading sensor internals.
- CLI polling remains non-blocking when no command is present.
- All long operations are explicit commands or setup-mode tasks.

Acceptance:

- Production firmware boots and starts tracking without Serial Monitor.
- Normal runtime has zero periodic human-readable serial output.
- 10-30 minute runtime test produces no FIFO recovery under normal static conditions.
- Host tests and PlatformIO build pass.

## 5. First-run setup and Wi-Fi provisioning

The tracker must be usable by a new user without editing source code.

### 5.1 First-run state machine

Add persistent setup state:

- `factory_empty`: no Wi-Fi and no valid calibration.
- `wifi_configured`: Wi-Fi credentials saved.
- `server_seen`: SlimeVR Server reached at least once.
- `calibration_required`: Wi-Fi works, but calibration is missing or invalid.
- `ready_6dof`: gyro/accel setup complete.
- `ready_mag_yaw`: mag calibration and mag axis validation complete.
- `production_ready`: runtime can start automatically.

Boot behavior:

- If Wi-Fi is missing, enter setup mode.
- If Wi-Fi exists but calibration is missing, connect and expose calibration-required state.
- If all required setup is valid, start SlimeVR runtime automatically.
- If repeated boot/runtime failures occur, fall back to setup-safe mode.

### 5.2 Serial provisioning, required

Commands:

```text
wifi scan
wifi set <ssid> <password>
wifi clear
wifi test
wifi save
wifi status
setup wifi
```

Requirements:

- Do not echo saved passwords in status output.
- `wifi test` must connect before saving when possible.
- `wifi status` must show SSID, connection state, RSSI, IP, server discovery state, and last error.
- `setup wifi` must guide the user through scan, set, test, and save.

### 5.3 Minimal SoftAP provisioning, should-have

SoftAP is useful for convenience, but it must remain small and safe.

Requirements:

- Only starts on first-run, factory reset, or explicit `setup ap start`.
- Has a timeout.
- Does not expose stored passwords.
- Has a unique or user-confirmed setup password if practical.
- Allows entering SSID/password and running a connection test.
- Stops after successful provisioning.

Out of scope for first release:

- Full mobile UI.
- BLE provisioning.
- Persistent web dashboard.

## 6. Guided calibration and setup UX

The user should not need to memorize low-level calibration commands.

### 6.1 Main setup command

Add:

```text
setup calibrate
setup status
setup quickstart
setup factory_reset
```

`setup calibrate` flow:

1. Hardware self-test:
   - LSM6DSV identity.
   - FIFO timestamps.
   - QMC6309 identity.
   - Sensor-hub health.
   - Optional hardware capability check.
2. Rest/gyro calibration:
   - Stationary gate using gyro norm, accel norm, FIFO health, and optionally LSM6DSV inactivity.
   - Save gyro bias and rest-calibration state.
   - Re-send SlimeVR SensorInfo with `hasCompletedRestCalibration=true`.
3. Accelerometer 6-position calibration:
   - Guided faces or reliable auto-face detection.
   - Quality score and residual/error report.
   - Save only if valid.
4. Magnetometer calibration:
   - Guided motion collection.
   - Coverage and residual checks.
   - Reject bad datasets.
5. Magnetometer axis/sign validation:
   - Guided rotations.
   - Determine or verify mag-to-IMU mapping.
   - Do not enable mag yaw by default until axis validation passes.
6. SlimeVR output sanity:
   - Confirm output frame convention.
   - Show server connection and packet counters.
7. Final report:
   - `ready_6dof`.
   - `ready_mag_yaw`.
   - `production_ready`.
   - Any optional missing hardware.

### 6.2 Rest calibration state

Rest/gyro calibration must become explicit runtime state, not just a saved bias.

States:

- `unknown`.
- `required`.
- `in_progress`.
- `completed`.
- `failed_motion`.
- `failed_timeout`.
- `invalidated`.

SlimeVR behavior:

- `SensorInfo.hasCompletedRestCalibration=false` if rest/gyro calibration is missing or invalid.
- Send updated SensorInfo when the rest state changes.
- Do not claim complete rest calibration after factory reset or invalid config migration.

Commands:

```text
cal rest
cal rest save
cal rest clear
cal rest status
```

## 7. Serial command UX cleanup

The CLI should be consistent, discoverable, and safe.

### 7.1 Command groups

Use these top-level groups:

```text
help
status
setup
wifi
slime
imu
fifo
cal
mag
tap
battery
power
config
test
stream
log
system
```

Requirements:

- Every group has `help` and `status` where useful.
- Destructive commands require explicit verbs: `clear`, `erase`, `factory_reset`.
- Save behavior is explicit: runtime changes do not silently persist unless the command says `save`.
- Status output is concise by default and detailed only with `debug`/`dump` commands.
- Command names in documentation and firmware must match exactly.

### 7.2 Recommended status commands

```text
status
setup status
wifi status
slime status
imu status
fifo status
cal status
mag status
tap status
battery status
power status
config print
```

### 7.3 Machine-readable output

Keep machine-readable logs available for tests, but not active by default in production.

Requirements:

- Stable prefixes for parser tools.
- Versioned log format when fields change.
- No machine log output unless enabled or running a test.

## 8. SlimeVR protocol completeness

Current SlimeVR output is a runtime feature, not just a debug stream. Complete the missing
user-visible pieces.

Must-have:

- Dynamic SensorInfo rest-calibration state.
- RotationData from the prepared tracking snapshot.
- Heartbeat/ping/pong handling.
- Tap packet for accumulated LSM6DSV physical tap events.
- Battery telemetry when hardware supports it.
- Temperature telemetry.
- Signal/RSSI telemetry.
- Clear handling of server feature flags that enable/disable calibration or mag use.

Should-have:

- Real magnetometer accuracy telemetry after mag quality scoring is trustworthy.

Requirements:

- Do not send fake magnetometer quality.
- Do not let server-side body calibration leak into local IMU calibration.
- Keep local IMU calibration, sensor fusion, mag gates, and sensor-to-board mapping on the tracker.

## 9. LSM6DSV embedded features for one-IMU setup

Use the useful hardware features without replacing the proven tracking pipeline.

### 9.1 Accumulative tap input, must-have

Add an embedded-event driver layer for:

- LSM6DSV physical tap detection.
- Optional hardware double-tap detection for diagnostics only.
- Tap source axis/sign when available.
- Interrupt/status demux with FIFO events.
- Firmware-side aggregation of 2..10 physical taps into one SlimeVR Tap packet.

Runtime requirements:

- ISR only records that an interrupt happened.
- Register reads and event parsing happen in the main loop.
- Duplicate suppression prevents one latched/source event from being counted twice.
- A sliding aggregation window, controlled by `TRACKER_TAP_AGGREGATION_WINDOW_MS`, extends after every accepted physical tap.
- A post-send lockout suppresses mechanical tails after a completed gesture.
- Very high-motion windows can later suppress false tap events.
- `FUNCTIONS_ENABLE.INTERRUPTS_ENABLE` is enabled with read-modify-write and verified with a mask so FIFO timestamp state in the same register is preserved.
- Masked register verification runs after configuration and then at a slow interval when no tap window is pending.
- Values below `TRACKER_TAP_MIN_COUNT` are suppressed; values above `TRACKER_TAP_MAX_COUNT` are clamped/flushed.
- Default `TRACKER_LSM6DSV_TAP_THRESHOLD` is `4` for RC1 hand-tap usability; production tuning can raise it after enclosure testing.

Commands:

```text
tap status
tap enable [save]
tap disable [save]
tap test [2..10]
tap inject <1..10>
```

Machine log when enabled:

```text
TAP,t_us,type,axis,sign,flags,gyro_norm_dps,accel_norm_g
```

### 9.2 Activity/inactivity, must-have for calibration/power hints

Use LSM6DSV activity/inactivity as a hint, not as the sole source of truth.

Use cases:

- Permit rest calibration only when software gates and inactivity agree.
- Permit runtime gyro bias learning only in stable windows.
- Enter lower-power idle behavior when not connected or not moving.
- Wake from idle quickly on motion.

### 9.3 Wake-up and 6D/free-fall, optional

Useful for diagnostics and future UX, but not blocking for first release.

### 9.4 SFLP, experimental only

The embedded SFLP quaternion path may be logged for comparison, but it is not part of the
first production runtime and must not block release.

## 10. Tracking and mag-yaw quality

The existing AHRS path remains the production path unless metrics prove otherwise.

Must-have improvements:

- Clear tracking states and confidence.
- Runtime metrics for accel rejection, mag rejection, FIFO health, and yaw correction.
- Safe gyro bias learning policy.
- Temperature-compensation validity range and quality report.
- Mag calibration coverage/residual quality gates.
- Mag axis/sign validation before enabling mag yaw by default.
- Mag disturbance detection and recovery.
- Frame/convention sanity tests for SlimeVR output.

Acceptance:

- Bad mag calibration cannot silently enable mag yaw.
- Magnetic disturbance degrades to 6DoF instead of corrupting yaw.
- Mag yaw resumes slowly after disturbance clears.
- Runtime bias learning does not learn during motion.
- Rejected samples do not advance AHRS integration time incorrectly.

## 11. Power optimization

Power work must be measured, not guessed.

### 11.1 Power profiles

Add:

```text
power profile performance
power profile balanced
power profile battery
power status
```

Profile knobs:

- Wi-Fi power-save mode.
- Wi-Fi TX power.
- SlimeVR output rate.
- Telemetry interval.
- IMU ODR/FIFO watermark.
- Magnetometer ODR.
- Serial/log/test availability.

### 11.2 Measurement matrix

Benchmark at least:

- IMU 960 Hz / SlimeVR 100 Hz.
- IMU 480 Hz / SlimeVR 100 Hz.
- IMU 240 Hz / SlimeVR 72-100 Hz if quality allows.
- Mag 60 Hz.
- Mag 30 Hz.
- Mag 10-30 Hz during static/idle.
- Wi-Fi power save off/min/max where supported.

Metrics:

- Average current.
- Peak current.
- Effective packet rate.
- Packet jitter.
- FIFO recovery count.
- Sample/network processing max time.
- Tracking quality metrics.
- Reconnect behavior.

Rule:

- Do not reduce tracking quality for the default profile.
- Lower-power profiles may trade output rate or latency only if clearly documented.

## 12. Config, migration, and fault recovery

Production firmware must survive updates and bad configs.

Requirements:

- Versioned config schema.
- Migration path for old configs.
- Per-section validity where practical.
- Safe defaults for missing/invalid sections.
- Factory reset command.
- Optional calibration-only reset.
- Last-known-good config or backup calibration slot if practical.
- Watchdog policy.
- Boot-loop detection.
- Reboot reason reporting.
- Fallback setup mode after repeated failures.

Commands:

```text
config print
config save
config load
config defaults
config erase
config migrate
system reboot_reason
system factory_reset
```

## 13. Tests and acceptance gates

### 13.1 Host tests

Required:

- Native unit tests pass.
- Config schema/migration tests.
- SlimeVR packet writer/runtime tests.
- CLI parse tests.
- Tracking state tests.
- Mag yaw/gating tests.
- Tap event parser tests after tap support is added.

### 13.2 Hardware tests

Required before release:

```text
test static 600
test runtime 600
test runtime 1800
Wi-Fi reconnect test
SlimeVR server restart test
power-cycle after calibration
accumulative tap test
mag disturbance test
battery telemetry test when supported
```

Release acceptance:

- No FIFO recovery in normal static runtime test.
- No unexpected serial output in production profile.
- SlimeVR preview is smooth at the default output rate.
- Rest-calibration state is correct before and after calibration.
- Factory reset returns to first-run setup mode.
- Documentation matches commands and behavior.

## 14. Documentation requirements

Every behavior-changing patch must update documentation in the same series.

Required docs:

- `docs/project_status.md`: current implemented state.
- `docs/cli_reference.md`: exact commands and examples.
- `docs/config_schema.md`: config fields, migration, defaults.
- `docs/slimevr_network.md`: protocol behavior and telemetry.
- `docs/tracking_pipeline.md`: AHRS, mag yaw, quality states.
- `docs/testing.md`: host and hardware acceptance tests.
- `docs/production_firmware_roadmap.md`: this roadmap.

Documentation rules:

- No stale “future network” or “future output” language for implemented features.
- Roadmap-only items must be marked as planned, not implemented.
- Command examples must be tested or copied from firmware help output.
- Optional hardware behavior must say what happens when unsupported.

## 15. Suggested implementation order

1. Add build profiles and production hot-path cleanup.
2. Add first-run setup state and serial Wi-Fi provisioning flow.
3. Add `setup status` and rest-calibration state reporting to SlimeVR SensorInfo.
4. Add guided rest/gyro calibration.
5. Add guided accel calibration wrapper around existing 6-position calibration.
6. Add LSM6DSV embedded physical tap driver, accumulator and SlimeVR tap packet runtime.
7. Add battery telemetry skeleton and optional ADC backend.
8. Add mag calibration quality gates and mag axis/sign validation.
9. Add minimal SoftAP provisioning if code size and stability remain acceptable.
10. Add power profiles and measurement tooling.
11. Add config migration/fallback/watchdog hardening.
12. Expand replay/acceptance tests and lock release criteria.
13. Update docs after each phase and keep this roadmap current.

## 16. Definition of done

The roadmap is complete when:

- A new user can provision Wi-Fi without editing source code.
- The tracker guides the user through required calibration.
- The firmware reports correct rest-calibration state to SlimeVR.
- 6DoF tracking is stable, and mag yaw only applies when trustworthy.
- Accumulated 2..10 tap gestures work reliably and can trigger the expected SlimeVR action.
- Battery/temp/RSSI telemetry works or is clearly reported unsupported.
- Production profile runs without debug spam or unnecessary hot-path work.
- Power profiles are measured and documented.
- Config survives firmware updates or fails safely.
- Factory reset and setup recovery work.
- Host and hardware acceptance tests pass.
- Documentation matches the actual firmware.
