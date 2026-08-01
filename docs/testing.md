# Testing strategy

This project has two very different kinds of tests:

1. **Firmware-side diagnostics** that run on the ESP32-C3 and exercise the real
   IMU/FIFO/AHRS runtime.
2. **Standalone native tests** that compile selected C++ headers on the host
   machine without Arduino, without ESP32 hardware, and without flashing.

Both are useful, but they catch different classes of bugs.

## What native tests are for

Native tests are for pure or mostly-pure logic:

- `core/math.hpp` vector, matrix and quaternion invariants.
- `sensor/ahrs_6dof.hpp` timestamp and quaternion invariants.
- `runtime/static_test_types.hpp` statistics helpers.
- `sensor/imu_quality.hpp` timestamp/FIFO/saturation/accel-gate decisions.
- `sensor/gyro_temperature_compensation.hpp` bias model, range metadata and learning gates.
- `runtime/runtime_bias_types.hpp` state reset semantics.
- `runtime/runtime_gyro_bias_controller.hpp` host-safe decision helpers.
- `sensor/mag_yaw_correction.hpp` gate/reject/cooldown behavior.
- `config/tracker_config_detail.hpp` CRC helpers and schema constants.
- `config/tracker_config_schema.hpp` default schema layout expectations.
- `config/tracker_config_storage.hpp` dual-slot, selector, migration, candidate, signature, wear and promotion state rules using the native Preferences substitute.
- `network/wifi_manager.hpp` non-blocking connection/reconnect decisions through fake adapters.
- `network/udp_transport.hpp` host-safe endpoint helpers.
- `output/slimevr_packet_writer.hpp` packet encoding/parsing helpers.
- `runtime/slimevr_output_runtime.hpp` host-safe session/output state rules.
- `serial/bounded_duplex_stream.hpp` bounded queue, byte-budget drain, stall and drop rules.

Native tests are intentionally not a replacement for firmware tests. They do
not verify SPI, GPIO interrupts, LSM6DSV FIFO timing, QMC6309 sensor-hub traffic,
NVS/Preferences, Serial output, or CPU timing on the ESP32-C3.

## Running the local quality gate

The convenience entrypoint is:

```bash
python tools/check_all.py --clean
```

On Linux/macOS/WSL/Git Bash you can also use:

```bash
tools/check_all.sh --clean
```

The script always runs native tests unless `--skip-native` is passed. It also runs the source-filter, profile-matrix and documentation contract validators, plus Python tool smoke tests unless `--skip-tool-smoke` is passed. When `pio`/`platformio` is available, the default gate builds Debug, Production, Production Diagnostic and Slim. If PlatformIO is not installed, ESP32 builds are skipped by default so host-only development machines can still run the native gate. To make missing PlatformIO a failure, use:

```bash
python tools/check_all.py --require-pio
```

If PlatformIO is installed but not on `PATH`, pass it explicitly:

```bash
python tools/check_all.py --require-pio --pio-bin "C:\\Users\\you\\.platformio\\penv\\Scripts\\platformio.exe"
```

Or set `PIO` for the current shell. PowerShell:

```powershell
$env:PIO = "C:\Users\you\.platformio\penv\Scripts\platformio.exe"
python tools/check_all.py --require-pio
```

CMD:

```bat
set PIO=C:\Users\you\.platformio\penv\Scripts\platformio.exe
python tools/check_all.py --require-pio
```

Git Bash:

```bash
PIO=/c/Users/you/.platformio/penv/Scripts/platformio.exe python tools/check_all.py --require-pio
```

WSL:

```bash
PIO=/mnt/c/Users/you/.platformio/penv/Scripts/platformio.exe python3 tools/check_all.py --require-pio
```

On Windows, `tools/check_all.py` is the portable entrypoint for PowerShell/CMD. `tools/check_all.sh` works from Git Bash or WSL.

## Running standalone tests directly

From the project root:

```bash
python tools/run_standalone_tests.py --clean
```

The runner compiles host-safe project `.cpp` files once into object files, then compiles and links every `tests/native/test_*.cpp` as a separate executable into:

```text
build/native_tests/
```

It uses `CXX` when set, otherwise tries `g++`, `clang++`, then `c++`. Keeping project sources as reusable objects avoids recompiling AHRS/quality/mag logic for every single native test executable.

Examples:

```bash
CXX=clang++ python tools/run_standalone_tests.py --clean
python tools/run_standalone_tests.py --build-only
python tools/run_standalone_tests.py --extra-cxxflag -fsanitize=undefined
```

The native test compiler flags intentionally mirror the diagnostic firmware
warning profile. The committed default firmware environment is
`BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG`, but host tests stay independent from that
selection. Tool smokes also verify deterministic Git/worktree identity generation
and PlatformIO build-result classification:

```bash
python tools/test_build_identity.py
python tools/test_check_all_policy.py
```

The warning flags remain:

```text
-Wall
-Wextra
-Wshadow
-Wdouble-promotion
-Wformat=2
-Wno-unused-parameter
```

Do not add ESP32 hardware APIs to native tests. A tiny `tests/native/Arduino.h`
stub exists only for host-safe modules that mention `Stream`/`millis()` in a
print/status helper but do not actually use ESP32 hardware. Do not use that stub
to test app, SPI, GPIO, Preferences/NVS, real Wi-Fi or Serial transport behavior. If a
module needs real Arduino framework semantics, it remains firmware-test only
until the pure decision rule is isolated behind a host-safe helper.

## CRLF patch and whitespace policy

The firmware repository is intentionally CRLF-heavy. `.gitattributes` marks
carriage return at end of line as valid whitespace, so `git diff --check` can
still detect real trailing-space errors without reporting every changed CRLF
line. Patch generation must preserve each file's existing line-ending style; do
not run repository-wide `dos2unix` or `unix2dos`.

Patches are expected to apply from the repository root with:

```bash
patch --dry-run -p1 < /mnt/c/Users/nikol/Downloads/000x_name.patch
patch -p1 < /mnt/c/Users/nikol/Downloads/000x_name.patch
```

## Hardware/runtime test budget

Do not request hardware tests for behavior that is already fully covered by host
logic and profile builds. Use this default matrix:

| Change | Native/project-contract checks | PlatformIO builds | Tracker runtime test |
|---|---:|---:|---:|
| Documentation, profile policy, source filters or host tools | Required | Required | None |
| Pure math, packet encoding or host-safe state machine | Required | Required | None unless hardware integration changed |
| FIFO/IMU driver or timestamp integration | Required where possible | Required | One focused serial/telnet smoke test |
| Calibration capture using real sensors | Required for fit/state logic | Required | One focused capture only |
| Wi-Fi/UDP reconnect or server protocol integration | Required with fake transports | Required | One focused server smoke test |
| Sleep/wake or power policy | Required for controller logic | Required | One dedicated A/B test stage |

Long `test static` or `test runtime` captures are release/acceptance tools, not a
mandatory response to every patch. Prefer one short test that crosses the exact
hardware boundary changed by the patch. Do not ask for several ideal-condition
captures when the same regression can be proven by native tests or a fake
transport.

## Real-time output resilience acceptance

The host gate covers phase-locked rotation deadlines, jitter/late-loop catch-up,
`millis()` wraparound, complete-record admission/drop behavior, oversized-line
rejection, ring wrap, partial drains, drop-warning insertion, reset semantics,
stalled sinks and empty-drain no-op behavior. Compile-only coverage includes the
machine-log producer backpressure path.
Only one hardware test is required for this patch:

```text
perf on
# perf status now reports calibration_0022 and calibration_0023 separately
motion on
tap log on
# generate several minutes of USB/telnet diagnostic output
console status
perf tracking
```

Acceptance requires `fifo_overrun_delta=0`, `fifo_full_delta=0`, no tracking
recovery caused by output, and a stable effective rotation deadline rate.
`serial_output_bytes_dropped`, `remote_console_output_bytes_dropped` or a non-zero
`LOGSTAT,BACKPRESSURE` means diagnostic data was intentionally omitted. These
must not coincide with FIFO loss or a reduced steady-state RotationData rate.
Do not request additional static/motion captures solely for this patch.

## Build identity and `check_all` policy

Every PlatformIO environment runs `tools/generate_build_identity.py` before
compilation. The generated metadata is shown by `version`, `status`, boot and
remote-console headers, static/runtime reports, machine-log `LOGVER`, and the
SlimeVR handshake firmware string.

Use the full local gate before accepting a patch:

```bash
python tools/check_all.py --require-pio
```

Mandatory firmware builds are Production, Production Diagnostic, Slim, Debug
and the explicit `BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK` gate. Every environment
inherits the same committed 4 MiB no-OTA layout with one 3 MiB factory app, so
there is no smaller wearable partition whose overflow may be downgraded to a
warning. Any compile, type, unresolved-symbol or image-size failure is fatal.
The link-check environment remains separate only to preserve an explicit full
Debug source/link validation step.

Explicit `--pio-env` selections are strict and never downgraded to warnings.
Raw PlatformIO logs and parsed RAM/Flash summaries are saved under
`build/check_all/platformio/`.

## Running firmware diagnostics

Firmware diagnostics still need PlatformIO and the ESP32-C3. You can run them directly or through the quality gate:

```bash
python tools/check_all.py --require-pio
```

Direct commands:

```bash
pio run -e BOARD_LOLIN_C3_MINI_DEBUG
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
pio run -e BOARD_LOLIN_C3_MINI_SLIM
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG -t upload
pio device monitor
```

Recommended smoke-test commands after an architectural change:

```text
help
status
health
config nvs
fifo stats
quality stats
ahrs status
bias status
mag status
stream
net status
slime status
stream quat
stream off
test status
test static 120
```

After a CLI-domain refactor, also touch each command domain once. The goal is not
to validate sensor quality, but to catch missing `.cpp` includes, broken hook
wiring, and command router regressions on the real firmware build:

```text
version
config print
imu status
fifo status
quality stats
ahrs status
bias status
mag status
stream
net status
slime status
test status
cal temp print
```

If IMU/FIFO live reconfiguration was touched, verify that the magnetometer path
is re-armed correctly after the change:

```text
imu rate 240 save
fifo stats
mag status
```

For long-run stability after a risky runtime change:

```text
test static 3600
```

Important acceptance metrics:

```text
estimated_dropped_samples=0
fifo_overrun_delta=0
fifo_full_delta=0
fallback_timestamp_samples=0
tracking_recovery_active=no
bad_timestamp_samples=0
```

`yaw_drift_rate_deg_min` is useful, but it is not expected to be zero in 6DoF
mode without magnetometer correction.

## What to test when adding code

### Core math or AHRS changes

Add native tests for:

- Quaternion normalization.
- Rotation direction/convention.
- Timestamp reject/clamp behavior.
- Static accel convergence invariants.

Then run a firmware `test static 120`.

### Runtime bias changes

Add native tests for:

- Reset/counter behavior.
- Base-bias source selection.
- Temperature range gates and cautious/out-of-range gain scaling.
- Decision flag packing.
- Trim clamp rules.
- Window accept/reject criteria when extracted into pure helpers.

Then run:

```text
bias status
test static 120
```

### Mag/yaw changes

Add native tests for:

- Reject flags.
- Gate open/closed transitions.
- Cooldown behavior.
- Correction sign and max-step clamp.

Then run:

```text
mag status
mag heading
mag yaw status
test static 120
```

### Config schema changes

Add native tests for:

- Schema version constants.
- Default construction invariants.
- CRC known vectors.
- Any migration helpers once migrations exist.

Then run firmware commands:

```text
config print
config crc
config nvs
config save
reboot
config nvs
```

## Test boundaries

Native tests must stay independent from the app layer:

```text
Allowed in native tests:
  core/*
  selected sensor/* pure logic
  selected runtime/* pure state/types and host-safe decision helpers
  selected network/output protocol logic through fake host adapters
  config/* schema/detail headers
  the minimal tests/native/Arduino.h stub for Stream/millis-only helpers

Avoid in native tests:
  app/*
  serial/* command dispatcher
  Preferences/NVS
  SPI/GPIO/Serial/real Wi-Fi
  hardware timing or interrupt behavior
```

If a new piece of logic is important but cannot be tested natively because it
pulls in Arduino, consider extracting the pure decision rule into a small helper
or data-only type. Do not add Arduino stubs just to make app/transport code look
unit-testable; hardware behavior must be validated on firmware.

## Current native test coverage

Initial native tests cover:

```text
core math/quaternion basics
AHRS static/no-gyro invariants
AHRS startup rejection and dt clamp/reject policy
AHRS implementation linked from `sensor/ahrs_6dof.cpp`
IMU quality timestamp/gap/recovery/saturation gates
IMU quality implementation linked from `sensor/imu_quality.cpp`
gyro temperature compensation bias/range/learning gates
static-test scalar/vector stats
static temp bin indexing
runtime gyro bias reset state and controller decision helpers
mag yaw gate/reject/cooldown behavior
config detail CRC/schema defaults
config hardening/sanitize behavior
serial parse helper behavior
tracking-state controller transitions and stationary recovery completion
AHRS heading-preserving tilt reacquisition after unreconstructable gaps
Wi-Fi manager state-machine behavior through fake adapters
UDP endpoint helpers
SlimeVR packet writer, negotiated packet-100 bundle, fallback-rate and explicit packet-23 behavior
coherent prepared motion snapshot, gravity removal and stale-timestamp rejection
status LED pattern timing and manual/identify overrides
```

This is a starting point, not complete coverage. The next useful native-test
expansions are:

```text
config validate/sanitize helpers once host-safe
mag runtime processing trust/reject edge cases
runtime bias full window accept/update behavior
accel/mag calibration residual quality edge cases
replay-driven AHRS regression tests from saved machine logs
```

## Replay/metrics from machine logs

For tracking changes, collect a machine-readable E0 log and score it on the host:

```text
log full
log header
test static 120
log summary
log off
```

Then run:

```bash
python tools/replay/replay_machine_log.py tracker.log --pretty
```

Replay gates should use machine-readable frames only. Human `status`/`health` output is useful for inspection, but should not become a regression input format. `tools/check_all.py` runs a small replay smoke test against `tests/fixtures/e0_static_smoke.log` so the replay parser itself stays usable. Full magnetometer replay fixtures should include `MAGR` rows, which are emitted only by `log full`.

## Replay baseline capture smoke sequence

For a log that is useful as a replay fixture, capture machine log output rather
than human-readable status text:

```text
setup status
log reset
log full
log rate 20
log header
test static 600
log summary
log off
```

During `test static 600`, keep the tracker still for the first and last two
minutes. In the middle, gently rotate it through several orientations if you
want the same file to exercise mag/yaw and accel gating. Do not disconnect or
change serial baud during capture.

## Magnetometer replay capture smoke sequence

Use this when creating a fixture for magnetometer calibration, axis mapping, or
magnetic disturbance work:

```text
stream off
output stop
mag enable save
mag heading auto off
mag yaw disable save
log reset
log full
log rate 20
log header
mag cal reset
mag cal start
# rotate slowly through all orientations for 60-120 seconds
mag cal stop
mag cal status
log summary
log off
```

Validate the captured file on the host:

```bash
python tools/replay/replay_machine_log.py logs/mag_sweep_001.log --require-magr --min-magr-rows 500 --pretty
```

Do not mix `stream raw/scaled/quat/debug` with replay capture; the replay parser
ignores human output, but cleaner serial captures are easier to inspect and
archive.

## Full runtime / Wi-Fi / SlimeVR load test

`test static` is an IMU/FIFO stationary test. It is not enough for Wi-Fi/server optimization because it was designed around sensor stability, not around the complete firmware loop. For full network load use:

```text
test runtime <seconds>
```

Recommended baseline before optimizing heat or Wi-Fi power:

```text
net status
slime status
slime counters reset
test runtime 600
slime debug
health
fifo stats
quality stats
```

The runtime test reports:

- loop/CLI/FIFO/network/heartbeat section timing;
- max and average loop costs;
- slow loop/network/FIFO counters;
- IMU runtime sample rate;
- FIFO/perf/quality deltas;
- Wi-Fi disconnect/connect-timeout deltas;
- SlimeVR rotation, ping/pong, unknown packet and send failure deltas;
- start/end temperature.

Healthy Wi-Fi + SlimeVR run targets:

```text
wifi_connected_end=yes
slime_server_found_end=yes
slime_send_failures_delta=0
slime_unknown_packets_delta=0
wifi_disconnects_delta=0
tracking_recovery_delta=0
quality_estimated_dropped_delta=0
fifo_overrun_delta=0
fifo_full_delta=0
```

For SlimeVR output, also check the effective rotation rate:

```text
slime_rotation_rate_hz_observed ~= slime rate
```

New/default Production/Debug configs use an 18-word FIFO watermark and 8 MHz SPI, with a
4 MHz startup fallback. Hardware drains feed a 512-sample raw RAM ring in
Production/ProductionDiag (256 in Slim) and a 64-sample mag ring. The consumer
is work-conserving: each slice guarantees raw progress, favors raw IMU samples
over mag callbacks, services network between slices, and receives a larger app
budget when the raw queue reaches its urgent high-water threshold. Hardware SPI
time is excluded from the callback budget.
Long runtime/SlimeVR diagnostic reports cooperatively service FIFO and network
between output sections. Healthy hardware tests must keep both hardware FIFO and
RAM-ring overflow counters at zero.

For a 100 Hz hardware cadence check on ProductionDiag, prefer the compact,
non-destructive window:

```text
perf tracking reset
# move the tracker continuously for 5-10 minutes
perf tracking
```

Healthy results normally have zero `fifo_overrun_delta`, `fifo_full_delta`,
`quality_dropped_delta`, `quality_recovery_delta`, and RAM-ring overflow deltas.
If a rare bounded FIFO full/overrun does occur, `tracking_soft_recovery_enter_delta`
should increase together with `tracking_soft_recovery_complete_delta`, while
`tracking_recovery_enter_delta` and a long `rotation_no_snapshot_delta` remain
zero. Timestamp-corruption and explicit-reset cases must still use strict recovery.
`rotation_delivery_pct` should be near 100%, `rotation_sent_rate_hz` should be near
the configured rate during movement, and `acceleration_sent_delta` should remain
close to `rotation_sent_delta`. The baseline command does not reset quality,
timestamp reconstruction or recovery state, so the measurement cannot hide an
existing fault. Use full `health`/`slime debug` only after the window when deeper
context is needed. A server GUI may visually report a low idle TPS for nearly
identical quaternions; firmware counter deltas are the source of truth for packet
cadence.

For an existing calibrated tracker, test 8 MHz and an 18-word watermark with:

```text
config spi 8000000 save
fifo watermark 18 save
```

Apply the watermark while stationary. Live hardware reconfiguration clears old
software-queued samples and intentionally requests controlled recovery when a valid
orientation already exists; wait for that short recovery before starting the timed
window. During boot, reconfiguration before the first quaternion must instead report
a startup-bootstrap bypass and allow normal AHRS gravity initialization. Failed
hardware apply or NVS save must restore the previous config. `perf tracking` reports
classified recovery causes so reconfiguration can be distinguished from
FIFO/timestamp faults.

Use `test stop` to finish early. `test status` prints both static and runtime test status.

### RC1 serial provisioning compatibility


Battery ADC runtime reads the RC1 divider `BAT+ -> R_TOP -> GPIO -> R_BOTTOM -> GND` with defaults `GPIO4`, `R_TOP=180 kΩ`, and `R_BOTTOM=180 kΩ`. GPIO4 is ESP32-C3 ADC1_CH4, the supported ADC path for battery telemetry. The divider is high impedance and has no hardware capacitor, so the runtime samples sparsely (`TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS`, default 10000 ms in Debug and 30000 ms in Production), takes a larger ADC burst (`TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT`, default 64), discards the first settle reads (`TRACKER_BATTERY_ADC_DISCARD_COUNT`, default 4), sorts the remaining burst, averages the trimmed center, maps 3.30-4.20 V to 0-100%, applies a slow EMA (`TRACKER_BATTERY_ADC_EMA_ALPHA`, default 0.12), and rejects impossible voltage steps. It reports safe 0.000 V / 0.0% when the divider is absent, below the present threshold, invalid, or unreadable. It also rejects BAT+ values above `TRACKER_BATTERY_PRESENT_MAX_VOLTAGE` (default 4.35 V) and preserves the previous filtered estimate on one-off low/high ADC glitches. CLI/status percentages are 0-100%, while SlimeVR BatteryLevel telemetry is converted to the protocol's 0.0-1.0 fraction at send time. The value is exposed through `battery status`, `GET INFO`, `slime status`, and periodic SlimeVR BatteryLevel telemetry.

## Guided temperature-capture validation

The dedicated setup temperature capture is covered by a native system test that
feeds complete sample/quality sequences. It verifies stable-window acceptance,
brief-touch recovery without restarting the full capture, slow-rotation and
vibration rejection, timestamp-fault pausing, and final partial-window commit.
This patch does not require a tracker runtime test because it changes only the
deterministic capture state machine; the existing physical setup procedure is
unchanged.

The firmware now exposes SlimeVR Server serial-compatibility commands for initial provisioning: `SET WIFI`, `SET BWIFI`, `GET INFO`, `GET CONFIG`, `GET TEST`, `GET WIFISCAN`, `REBOOT`, `FRST`, `DELCAL`, and temperature-only `TCAL`. `SET WIFI`/`SET BWIFI` save credentials into the separate network NVS config, enable Wi-Fi/discovery, and restart SlimeVR discovery. Blocking Wi-Fi scans suppress expected FIFO-recovery console noise for a short grace window without disabling recovery, counters, or machine-log events. `TCAL SAVE` is intentionally scoped to gyro temperature compensation so host-side temperature-calibration commands cannot accidentally capture unrelated runtime output/accel state. `TCAL RESET` changes only the in-RAM temperature model, resets the volatile runtime residual gyro trim learned against the previous model, and does not mutate the persistent config object unless a later explicit `TCAL SAVE` is requested.

## Motion light sleep

See [motion_light_sleep.md](motion_light_sleep.md) for the opt-in GPIO10/INT1 light-sleep bench procedure and its host-test coverage.

## Tap detector diagnostic capture

For a physical tap investigation, keep the tracker connected to SlimeVR and open either USB Serial or the Wi-Fi remote console. Run:

```text
tap reset
tap log on
tap status
```

Tap the enclosure several times, then run `tap status` and `slime status`. `tap log` emits only nonzero `TAP_SRC`, decoded single/double/axis bits, accumulator queue or suppression decisions, and the final SlimeVR send result; it deliberately does not print the idle 5 ms polls. The same `# TAP_LOG ...` lines are mirrored to Serial and the active remote-console/telnet client. Use `tap log off` after the capture.

Interpretation: no `event=tap_src` means the LSM6DSV hardware engine did not report an event; `tap_src` without `physical_tap` indicates an unexpected source-bit pattern; `suppressed_below_min`, `suppressed_duplicate`, or `suppressed_lockout` identifies firmware gesture filtering; `slimevr_no_server` / `slimevr_send_failed` identifies the output path.

## Sensor-to-device alignment validation

`test_sensor_to_device_alignment` is the host system test for the physical case-frame stage. It covers all 24 right-handed signed axis mappings, a non-discrete proper rotation, separation of rotation from a combined full 3x3 accel fit, contiguous stationary capture with an interrupted window, rejection of duplicate/parallel positions, runtime forward/inverse application and config validation. The normal hardware acceptance for this patch is only successful compilation; the next real `setup calibration` run will perform the two short physical observations through the same guided flow.

## Production-only compile coverage

The native gate also compiles production-only translation units that cannot be linked against the host NVS backend. `runtime/gyro_temp_static_fit.cpp` is covered this way so config/API drift fails before PlatformIO.
## FIFO coherency acceptance

After flashing `c3-6dsv-fifo-coherency` in ProductionDiag, use `fifo status`,
`motion status`, and `perf tracking`. `motion status` already contains the full
FIFO and quality correlation blocks; `fifo stats` and `quality stats` belong to
the Full Debug CLI and are intentionally not compiled into ProductionDiag.
Normal operation should keep gyro-only and pair-mismatch counters at zero or
extremely rare values. Isolated component
loss may increment them, but must not request FIFO recovery, stop gyro
integration, or make linear acceleration valid for that degraded sample.

A dynamic `ACCEL_NORM_OUTLIER` is different: it must disable use of accel as an
AHRS gravity observation without suppressing motion output. During movement,
`acceleration_sent_delta` should therefore track `rotation_sent_delta` unless a
separate `acceleration_skipped_*_delta` counter identifies a hard invalidity.
`slime_last_rotation_snapshot_age_us` is MCU publish-to-send age and should stay
near the output period; it no longer compares the LSM6DSV timestamp epoch with
ESP32 `micros()`.


## Protocol 22 motion-frame acceptance

`test_slimevr_motion_frame` locks the rotation/acceleration local-frame
contract. It verifies that the protocol adapter preserves device axes while
converting acceleration from `g` to `m/s^2`, keeps Hamilton
`q_world_from_device`, and emulates current server processing. The protocol-22 branch must reproduce one
coherent world-space motion vector; the legacy pre-22 extra -90 degree local-Z
acceleration correction must demonstrably disagree.

After flashing `c3-6dsv-calibration-epoch-field-safe` in ProductionDiag:

```text
slime status
motion on
# move the tracker strongly along its marked +X, +Y and +Z directions
motion status
```

Expected status includes:

```text
protocol_version=22
motion_frame_contract=device_x_right_y_forward_z_up
rotation_convention=world_from_device
acceleration_frame=device
acceleration_units=mps2
legacy_acceleration_correction=no
motion_frame_config_ready=yes
step_mounting_ready=yes
motion_packet_mode=bundle_100_rotation_17_accel_4
packet23_available=yes
packet23_enabled=no
sensor_info_sync_state=acknowledged
feature_negotiation_state=negotiated
server_feature_flags_available=yes
server_bundle_supported=yes
```

During movement on a bundle-capable server, `bundled_motion_sent`,
`acceleration_sent`, and `rotation_sent` should advance together and all hard
`acceleration_skipped_*` reasons should remain zero.
`bundled_motion_send_failures_delta` and `udp_send_failures_delta` should remain
zero. A server that does not answer FeatureFlags must report
`rotation_17_plus_accel_4_fallback`; rotation remains at the configured rate,
while `acceleration_rate_limited_delta` confirms the 50 Hz packet-4 fallback.
Under intentional UDP TX pressure, `tx_pressure_failures_delta` and
`tx_backoff_drops_delta` may rise. Recent valid ping/heartbeat reception should
select `udp_transport_rebind_successes_delta`; a failed rebind, stale RX or a
second burst may instead increment `udp_full_reopen_escalations_delta` and
`udp_reopen_requests`. `foreign_endpoint_packets_dropped`,
`pre_session_packets_dropped` and all `malformed_*` counters should remain zero on
a normal single-server LAN. `tap_user_action=off` is the default unless a mapping
was explicitly saved. The definitive directional acceptance is a successful
server step-mounting run; ordinary quaternion FBT can look correct even when
acceleration alone has the wrong local axes.

The build-profile validator also verifies that every ESP32-C3 environment inherits
`partitions/tracker_4mb_no_ota.csv`, that the factory app is exactly 3 MiB at
`0x10000`, that no OTA slot exists and that the complete layout ends at the 4 MiB
flash boundary.

## Patch 0020 session acceptance

After the server is found, run:

```text
slime debug
```

A normal bundle-capable session should converge to:

```text
sensor_info_sync_state=acknowledged
feature_negotiation_state=negotiated
server_bundle_supported=yes
foreign_endpoint_packets_dropped=0
pre_session_packets_dropped=0
malformed_packets=0
set_config_flag_apply_failures=0
ack_config_send_failures=0
protocol_change_ignored=0
```

`slime action yaw|full|mounting|pause` must emit packet 21 and advance the
corresponding UserAction counters. `slime tap-action ... save` must survive reboot;
the default is `off`. A repeated server magnetometer command after a deliberately
lost ACK must be acknowledged without another config write or magnetic-runtime
restart. Boot with magnetometer enabled must not report a redundant
`tracking_recovery_reconfigure_delta` solely from starting QMC6309 after FIFO
bootstrap.

## 0021 calibration-storage hardware acceptance

Only one real-device cycle is required after all PlatformIO profiles build:

```text
config slots
config save
config slots
reboot
config verify
config slots
```

Acceptance requires two valid generations after the second save, a valid
selector, the expected active generation after reboot, and unchanged tracking
calibration. The destructive fallback portion is optional unless a development
build exposes safe NVS corruption tooling; native tests already cover torn
writes, corrupt selected slots, selector loss and interrupted promotion.

Candidate smoke test:

```text
cal candidate stage manual
cal candidate status
cal candidate compare
cal candidate discard
```

Staging/status/compare/discard must not reset AHRS, reconfigure FIFO or change the
selected active generation. Do not use `promote force` as a routine hardware
smoke test.

### 0021a first-boot migration regression

When upgrading directly from 0020a, the first boot must reach the console and
report `c3-6dsv-calibration-epoch-field-safe`. Run `config slots` and
`config verify`; the legacy `cfg` key should migrate once without a reboot loop.
The host gate `tools/test_calibration_storage_stack_policy.py` must also pass.


### 0021b/0021c storage lifecycle regression

After flashing `c3-6dsv-calibration-epoch-field-safe`, run:

```text
version
config slots
config verify
cal candidate status
```

A normal boot must report `load_status=loaded` or `migrated`,
`storage_degraded=no`, `authoritative_apply_pending=no`,
`commit_uncertain_count=0`, and no unexpected legacy cleanup pending state.
`defaults_storage_error` is a degraded boot and must not be treated as an empty
first-run device. In that state `config save`, candidate flush/discard/promotion
and calibration persistence must remain blocked. `config verify` is read-only and
must not clear the block; only a successful `config load` plus hardware/runtime
apply may confirm recovery. `config erase` is the deliberate destructive escape.

A device upgraded from clean 0021/0021a/0021b may initially show a legacy-committed
v1 slot. Its first real `config save` must create a committed v2 slot/marker. A
second identical save must increment `noop_save_count` while leaving generation
unchanged.

Candidate promotion is calibration-only: it must not increment FIFO
reconfiguration/recovery counters, change output rate/FIFO/IMU settings, or stop
network output. A v3 candidate remains fresh after unrelated policy/evidence/timestamp saves but
must become stale after any active gyro/accel/mag/alignment calibration change.
After promotion, `last_comparison=promoted` must survive reboot and a repeat
promotion, including `force`, must return `already_promoted` without changing the
active generation.

### 0021d hardware no-op and model-freshness acceptance

After `config load`, run two immediate `config save` commands. With no runtime
policy change, both must leave `active_generation`, selected slot, config CRC and
`successful_active_writes` unchanged while incrementing `noop_save_count` twice.
Active quality/provenance must remain unchanged.

Stage a v3 candidate, toggle only `cal temp enable|disable save` or save another
non-calibration policy, and compare again. It must not report stale. Changing an
actual gyro/accel/mag/alignment model must report stale. A promoted candidate must
preserve current `temp_comp_enabled`, output/AHRS/FIFO/SPI policy, transfer mag
trust bounds, clear magnetic heading/yaw state and leave FIFO reconfigure/recovery
counters unchanged.

Integration acceptance must also verify:

- `cal gyro save` cannot persist an unsaved accel change and `cal accel save` cannot
  persist an unsaved gyro/temperature change;
- `config load/defaults`, gyro-validity changes, candidate promotion and local
  magnetometer enable/disable cause SensorInfo to become dirty and re-ACK;
- enabling magnetic yaw without accel, magnetic-field and axis-alignment calibration fails
  explicitly instead of reporting success after sanitize disables it;
- promotion returns `runtime_calibration_diverged` when RAM contains a third
  unsaved calibration model;
- a persisted temperature slope without a valid gyro bias sanitizes to no
  temperature model and clears its evidence; `cal temp set_slope` must reject
  the same missing-bias state without writing NVS.

## Patch 0022 magnetic acceptance

The native suite includes temporal field disturbance/recovery, repeated
independent disturbance cycles, proper-rotation enumeration/reflection rejection,
gyro/mag timestamp-skew rejection, synthetic axis solve, and large-yaw
reacquisition. Hardware acceptance must also cover a same-norm directional
magnetic disturbance, removal/re-entry, a yaw error beyond the normal innovation
limit, multi-axis motion coverage/candidate staging, and concurrent FIFO/network
counters. See [magnetic_heading_reliability.md](magnetic_heading_reliability.md)
for the exact sequence and expected states.

### 0022a regression matrix

The hotfix adds integrated native coverage for:

- actual 60 Hz noisy heading samples through the field monitor into yaw
  reacquisition;
- stable changed-environment fail-closed behavior and explicit reference restart;
- two-stage coarse-plus-`SO(3)` recovery of an approximately two-degree mechanical
  misalignment;
- orthonormal/determinant constraints and reflection/shear rejection for new
  solver results;
- active and candidate scoring on one dataset;
- measured axis candidate comparison and normal promotion preparation without
  `force`;
- policy enforcement that sample callbacks contain no full storage inspection,
  candidate flush or promotion.

Hardware acceptance must additionally measure solve/storage max duration while
960 Hz FIFO and 100 Hz network output are active, and require zero FIFO
full/overrun, no recovery entry and normal delivery during deferred service.

### 0022b regression matrix

Native and policy coverage additionally requires:

- abrupt stationary heading shifts of 5, 9, 12 and 19 degrees remain disturbed;
- slow drift and the maximum normal 2 deg/s yaw correction do not self-latch;
- real physical rotation with nonzero gyro does not trigger the stationary latch;
- realistic 60 Hz datasets solve at 30, 60, 90 and 120 deg/s using normalized
  confidence;
- a hypothesis that fits training windows but not held-out windows is rejected;
- training and validation each contain multiple independent temporal windows;
- rotation-deadline slack reports armed, future and due states correctly;
- policy gates require real FIFO status, output-deadline admission and
  axis-independent collection for repairing bad active alignment.

Hardware acceptance must still establish actual ESP32-C3 solve/storage duration
and require zero FIFO overrun/full, no recovery entry and normal packet delivery.

### 0022c cross-ABI promotion stack regression

`tools/test_calibration_storage_stack_policy.py` now rejects by-value
`TrackerConfig` composition inside `prepareCandidatePromotion` and enforces a
promotion-specific 768-byte host stack ceiling. This provides margin below the
1 KiB project limit across GCC ABIs; 0022b was 1008 bytes on Linux and 1040 bytes
on Windows/MSYS2.


## 0023a setup/output acceptance

The guided calibration still requires only the six ordinary case faces. Each
capture automatically includes a short held-out tail; do not move the tracker
until the command asks for the next face. The final verifier asks for one stable
ordinary face and checks the already-prepared coherent quaternion/linear
acceleration pair. No 45-degree or diagonal placement is required.

After setup, run:

```text
setup verify
cal autonomy status
config slots
perf on
perf status
```

`setup verify` must report `setup_output_verified=yes`, zero FIFO/timestamp/
recovery deltas, a valid linear-acceleration ratio of at least 0.8 and bounded
stationary residuals. `cal erase_all confirm` is the destructive recovery path
for completely removing calibration, candidate and autonomy transaction state.

## 0023b aggregate runner and cross-ABI stack gate

`tools/check_all.py` and `tools/run_standalone_tests.py` must attempt every
independent runnable gate even after an earlier failure. Compile, link, native
run, Python policy, replay and each PlatformIO environment report their own
result. The process exits nonzero only after printing one consolidated failure
list. Replay outputs are removed before regeneration so a failed command cannot
silently reuse a stale JSON file.

`CalibrationAutonomyController::buildAccelProposal()` keeps only six compact
best/held-out session indices in its local frame instead of copying six full
`Session` records. The autonomy stack policy applies a dedicated 640-byte
cross-ABI ceiling to this function. Linux/GCC measures 304 bytes after the
change; the extra margin is intentional for Windows/MSYS2 GCC ABI spill space.

Power-loss native coverage explicitly reboots during:

- `AcceptPending` after the autonomy candidate was removed but before journal
  cleanup;
- `RollbackPending` after exact selector restore;
- rollback after candidate cleanup but before rejection persistence;
- rollback after rejection persistence but before journal cleanup.

Every case must preserve the correct authoritative generation, resume the
idempotent journal stage, retain the rollback write barrier until cleanup is
complete and leave no autonomy-owned candidate behind.


## 0023e setup and disabled-autonomy regression

For a remote-console setup run, the complete setup header and the first
`Press Enter` prompt must appear without sending a second newline. Gyro acceptance
prints raw standard deviation, mean standard error, held-out standard deviation,
held-out mean standard error and each quality gate. A high-rate capture may pass
with raw noise above the former 0.20 dps threshold only when the long-window mean
is precise and the independent validation mean agrees.

For the disabled-autonomy baseline:

```text
cal autonomy 0022 off save
cal autonomy 0023 off save
reboot
cal autonomy status
```

Require:

```text
autonomy_imu_hotpath_enabled=no
autonomy_deferred_service_required=no
```

`perf status` may still show the profiler section when profiling is enabled, but
its work count must remain zero and no periodic 6-7 ms storage/service spike may
appear from passive `suspended_storage`.

## 0023f full guided-setup regression

`test_gyro_temp_calibration_capture` replays the measured high-rate gyro noise
from hardware and proves that accurate stationary window means are accepted,
while excessive vibration and a later constant slow rotation are rejected.
`test_gyro_temp_static_fit` carries accepted temperature bins through robust
fit, leave-one-bin-out validation and RAM application, and rejects an
inconsistent middle bin. `test_setup_output_verifier` additionally proves that
valid quaternion/linear-acceleration output cannot pass while the calibrated
input is rotating.

`test_calibration_0023f_policy.py` guards the setup cancellation/diagnostic
contract, cold admission of the inactive setup capture, unchanged storage
formats, copy-free temperature validation, and stack ceilings of 768 bytes for
the main fit, 1024 bytes for the persistence helper and 384 bytes for
leave-one-bin-out validation.


## 0023g guided magnetic coverage regression

`test_calibration_0023g_policy.py` guards the fixed-memory reservoir contracts, fit-set/full-capture diagnostic separation, guided setup ownership fixes, schema compatibility, and stack ceilings. `test_mag_calibration` includes a long-tail hard/soft fixture where early full-sphere coverage is followed by a much longer single-axis sweep; the final bounded fit set must remain calibratable. `test_mag_heading_reliability` feeds long sequential X/Y/Z motion into the guided-axis reservoir and requires all dominant-axis and train/validation strata to remain represented after replacement.


## 0023ga cross-ABI guided-axis stack regression

A Windows/MSYS2 `-fstack-usage` build measured `setupRunAxisAlignment()` at 1056 bytes against the 1024-byte per-function ceiling even though Linux measured 880 bytes. The function previously kept dynamic solver output, static fallback output, printing state, and the manual input buffer in one compiler-visible frame. `0023ga` isolates static solve, dynamic solve, dynamic/static reporting, application, and the manual prompt behind explicit GCC/Clang no-inline boundaries. The orchestrator ceiling is now 512 bytes and each isolated phase has its own conservative ceiling. `test_calibration_0023ga_policy.py` also reruns the complete `0023g` reservoir policy so stack hardening cannot silently remove the magnetic coverage fix.

Hardware acceptance must run `setup calibration full` on the real LSM6DSV/QMC6309 tracker. During mag motion, verify that `dynamic_axis_candidates_seen` continues increasing after `dynamic_axis_intervals` reaches its bound, `dynamic_axis_reservoir_active=yes`, replacements/skips increase, at least two `dynamic_axis_excited_axes` and two `dynamic_axis_partition_confirmed_axes` are retained, both window parities have stored data, and hard/soft diagnostics show adequate `mag_cal_fit_span_xyz` rather than relying only on `mag_cal_capture_span_xyz`. FIFO overrun/full, tracking recovery, output-delivery, and UDP failure deltas must remain zero or at the accepted baseline.

## 0023gb magnetometer fit-metric regression

`test_calibration_0023gb_policy.py` guards centered algebraic residual normalization, rejected-fit metric retention and all three status paths. `test_mag_calibration` compares the same quantized ellipsoid at zero and large hard-iron translations and requires equal normalized algebraic/geometric errors. A mildly non-ellipsoidal translated fixture must pass the unchanged physical thresholds, while a strongly non-ellipsoidal fixture must still fail `GeometricResidualTooHigh` and retain its diagnostic fit. The policy reruns `0023ga`, so metric hardening cannot regress the guided-axis reservoir or cross-ABI stack fix.


## 0023gc cross-ABI mag-fit stack/profile regression

`test_calibration_0023gc_policy.py` requires the raw and inlier ellipsoid passes to reuse one large `FitAccumulator`, forbids the two simultaneously-live workspaces that exceeded the Windows/MSYS2 stack ceiling, and applies a stricter 1792-byte limit to `MagCalibrationCollector::compute()`. It also compiles the minimal fit-quality reporter under both Production and Slim feature profiles and verifies that the app composition includes it outside the detailed-mag-status gate. The policy reruns `0023gb`, which in turn reruns `0023ga` and `0023g`, so the stack/profile fix cannot weaken residual normalization, rejected-fit diagnostics, guided-axis stack separation or reservoir coverage. Mandatory PlatformIO Production, Production-Diag and Slim builds remain the authoritative ESP32 link gates.

## 0023gd full magnetometer mathematics regression

`tools/test_calibration_0023gd_policy.py` guards affine-centered hard/soft fitting, solver conditioning/stages, exact retained-dataset coverage, shared timestamp-coherent interval construction, right-handed QMC6309 frame ownership, near-rail saturation, chronological FIFO magnetic dispatch and cross-ABI stack ceilings. It compiles and runs `test_mag_calibration`, `test_mag_heading_reliability`, `test_fifo_runtime_processor` and `test_fifo_pair_coherency`.

The hard/soft suite includes the hardware-shaped near-origin case, translated-fit invariance, a deterministic rotation/anisotropy sweep, a valid 4:1 ellipsoid and bad non-ellipsoidal rejection. Alignment coverage includes all 24 proper signed permutations, reflection ambiguity, invalid hard/soft transforms, exact shuffled-window counting, late-axis runtime/guided reservoir replacement, calibrated raw-zero admission and timestamp endpoint construction. FIFO coverage requires each magnetic callback to see a raw endpoint within one IMU period in the synthetic 960/60 Hz stream.

Hardware acceptance must additionally verify `gyro_endpoint_valid=yes`, bounded `gyro_endpoint_skew_us`, finite hard/soft solver stage/normalization/pivot diagnostics, matching alignment training/validation winners, `calibration_valid=yes`, `axis_alignment_valid=yes`, and no new FIFO overrun/full, tracking recovery, output delivery or UDP failure deltas.

## 0023ge magnetometer post-audit realtime regression

Run `python3 tools/test_calibration_0023ge_policy.py`. The policy reruns 0023gd
and additionally guards expensive due-mag bursts against the cooperative slice
budget, prevents raw-timeline advancement while an older mag backlog remains,
checks retained frame-validation reuse, verifies early fit diagnostics, and
rechecks the affected stack ceilings.

## 0023gf magnetic callback cross-ABI regression

Run `python3 tools/test_calibration_0023gf_policy.py`. The policy checks the no-inline phase boundaries, requires one wall-clock observation and one runtime-config construction per magnetic sample, forbids the unchecked inverse helper from the common frame API, and compiles both `-O2` and `-Os` stack-usage variants. The orchestrator ceiling is 512 bytes; field and yaw phases each have a 640-byte ceiling. The policy reruns the full 0023ge chain.

## 0023gg magnetic timestamp and setup acceptance regression

Run `python3 tools/test_calibration_0023gg_policy.py`. It checks per-frame IMU/FIFO anchoring of sensor-hub magnetic timestamps, nominal fallback ownership, same-coarse conservative alignment fallback, adaptive bounded setup verification, checkpoint-honest rollback wording, and cross-ABI stack ceilings. It compiles and runs FIFO timestamp, magnetic alignment, and setup output-verifier regressions. The aggregate `check_all.py` run owns the complete predecessor policy chain to avoid another layer of recursive recompilation.

## 0023gh SlimeVR Wi-Fi provisioning compatibility regression

Run `python3 tools/test_slimevr_wifi_provisioning_0023gh_policy.py`. The policy locks all six upstream `WiFiReconnectionStatus` numeric values, the saved-attempt/server-attempt distinction plus `Backoff -> Failed` and `Connected -> Success` mappings, exact `SET WIFI`/`SET BWIFI` acknowledgement strings, and the non-blocking persistence/reconnect path. It also host-compiles the serial compatibility translation unit in native and Production-profile compositions. The aggregate `check_all.py` run owns predecessor policies.

## 0023gi SlimeVR Connect Trackers handshake/build-date regression

Run `python3 tools/test_slimevr_connect_trackers_0023gi_policy.py`. The policy locks healthy serial `status: 0`, keeps Wi-Fi progress exclusively in `WiFiReconnectionStatus`, requires every discovery retry to serialize packet number zero without consuming the session sequence, verifies the build-dated firmware string used by serial and UDP handshakes, and tests UTC/`SOURCE_DATE_EPOCH` build-date generation. It compiles the serial compatibility and output-runtime translation units and runs the focused packet-writer/build-identity regressions. The aggregate `check_all.py` owns predecessor policies.


## 0023gj already-connected Connect Trackers session-restart regression

Run `python3 tools/test_slimevr_connect_trackers_0023gj_policy.py`. The policy requires one typed shared SlimeVR runtime-apply path, maps ordinary `slime start` to session preservation and CLI reconnect plus successful `SET WIFI`/`SET BWIFI` to an explicit session restart, forbids recursive CLI dispatch, and enforces commit-before-live credential activation. Its native runtime test establishes and acknowledges an initial session while Wi-Fi remains connected, restarts only the SlimeVR session, verifies endpoint/feature/`SensorInfo` invalidation, emits a new packet-number-zero discovery, accepts the same server again, and completes a fresh `SensorInfo` acknowledgement. Affected serial units are compiled natively and in the Production profile.


## 0023gk magnetometer robust-fit acceptance regression

Run `python3 tools/test_calibration_0023gk_policy.py`. The policy requires an effective algebraic backstop compatible with the geometric gate, a bounded sigma cap tied to the physical residual limit, at most three no-heap refit passes, exact inlier-membership convergence, the single reused `FitAccumulator`, and a no-inline candidate helper that preserves the 1792-byte cross-ABI `compute()` ceiling. Native regressions reproduce the second hardware log where geometric quality passes but the old algebraic limit failed, recover a 15% moderately disturbed population, reject 20% through the unchanged inlier budget, and preserve a direct strong non-ellipsoidal geometric rejection. The policy compiles and runs the hard/soft suite; the aggregate `check_all.py` run owns the complete predecessor policy chain without recursive recompilation.


## 0023gl SlimeVR UDP TX pressure/recovery regression

Run `python3 tools/test_slimevr_udp_tx_recovery_0023gl_policy.py`. The policy requires errno-aware physical sends, bounded 20-160 ms stale-pose backoff, four-consecutive and exact 8-of-32 recovery gates, session-preserving local socket rebind, bounded full-reopen escalation, and the replacement diagnostics. Native tests reproduce consecutive and intermittent TX pressure, prove that backoff avoids an extra physical send, preserve FeatureFlags/bundle mode across rebind, cover stale-RX direct reopen and failed-rebind escalation, and run with ASan/UBSan. The source policy also locks that `perf on` only enables/resets the profiler and has no Wi-Fi, UDP or SlimeVR recovery side effect.

## pre-0024 hotpath headroom/freshness regression

Run `python3 tools/test_pre_0024_hotpath_headroom_policy.py`. The policy locks unchanged tracking equations, ODR, sample order and 5 ms tap polling; requires one absolute FIFO budget covering SPI drain and raw/magnetic callbacks; isolates the nested SlimeVR tick to rotation-only delivery; proves deterministic bounded battery sampling; verifies exact event counters with sampled expensive motion metrics; and requires exact deferred runtime-bias/autonomy evidence. Focused native tests cover profiler histograms and micros rollover, FIFO chronology/yield, numerical runtime-bias equivalence, autonomy lifecycle, battery estimator equivalence and the critical SlimeVR path. ASan/UBSan and stack ceilings guard the changed deferred and scheduling boundaries.

## pre-0024a tracking deadline regression

Run `python3 tools/test_pre_0024a_tracking_deadline_policy.py`. The policy requires the side-effect-free 100 Hz rotation-deadline gate before nested network service, forbids the old activity-signature reduction in the critical path, checks that the FIFO budget is evaluated before dequeuing a fifth callback, locks 40 ms age-based urgency without sample dropping, and requires background mag-axis evidence to use a bounded eight-entry FIFO with calibration-epoch rejection and the same FIFO/output admission gate as solver/storage work. Focused native tests prove no-op/due SlimeVR behavior, FIFO order and bounded overshoot, timestamp-span urgency, deferred-axis numerical/order equivalence, bounded overflow, admission deferral, and stale-epoch rejection. The compact FIFO boundary runs with ASan/UBSan, the actual magnetic controller is linked and executed with production host optimization, and stack ceilings cover the changed FIFO, SlimeVR and magnetic boundaries.

## pre-0024ab transform-cache regression

Run `python3 tools/test_pre_0024ab_hotpath_transform_cache_policy.py`. The policy proves that the validated sensor-to-device frame is cached by the authoritative tracker-config CRC, unchanged revisions take the constant-time path, changed invalid matrices are rejected fail-closed, and direct standalone magnetic callers retain validation fallback. It also requires the magnetic runtime controller to cache its immutable runtime configuration instead of rebuilding and copying it for every magnetic sample. Fixed-memory latency histograms cover overloads beyond 100 ms and report the observed maximum instead of `UINT32_MAX` for the open-ended bucket. Focused regressions preserve IMU/magnetic numerical transforms, full-rate gyro prediction, accel correction cadence, output cadence and packet semantics; stack ceilings cover the changed IMU and magnetic boundaries.

## pre-0024ac IMU hotpath/slack regression

Run `python3 tools/test_pre_0024ac_imu_hotpath_slack_policy.py`. The policy requires exactly one lightweight temperature-compensation evaluation in the main IMU sample path, shared current-bias use for calibration/quality/runtime-bias evidence, and a stream-mode check before the disabled serial path reads `micros()`. It locks FIFO/rotation-slack admission for optional services, permits at most one completed background worker per ordinary loop, requires fixed-memory 1/64 sampled IMU-stage telemetry and optional-service admission-skip counters, and forbids sample-history loss or AHRS/output-cadence changes. Focused native tests cover temperature snapshot equivalence, runtime-bias overload equivalence, profiler counters and admission boundaries; ASan/UBSan and stack ceilings cover the changed bias, IMU and app-loop boundaries.


## pre-0024ad network-pressure pacing/recovery regression

Run `python3 tools/test_pre_0024ad_network_pressure_policy.py`. The policy separates transient `ENOMEM`/`ENOBUFS`/`EAGAIN` pressure from non-pressure socket errors, retains the exact 8-of-32 bitmap as diagnostics only, requires 500 ms without successful motion before a local rebind, and permits full discovery only after a failed 1000 ms post-rebind interval or an equivalent stale-server failure. Rebind/full-reopen cooldowns, stable episode closure, explicit timestamp-validity state, background-control backoff, rotation-only capability negotiation, deterministic MAC phase distribution and all new diagnostics are source-gated. Native tests prove intermittent pressure does not churn the socket/session, local rebind preserves bundle negotiation, reconnect does not emit packet-4 acceleration before negotiation, and failed recovery remains fail-closed. Optimized, ASan/UBSan and stack-usage gates cover the changed runtime.
