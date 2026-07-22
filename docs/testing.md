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

Mandatory firmware builds are Production, Production Diagnostic and Slim. The
normal Debug image may exceed the wearable application partition. To avoid
hiding real Debug regressions, `check_all` first builds the internal
`BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK` environment with the same source set and
flags but a larger no-OTA partition. Compile, type and unresolved-symbol errors
there are fatal. A size-only overflow remains advisory even if the larger
link-check partition is also exceeded; any non-size compiler/linker diagnostic
still makes the gate fail. The normal Debug wearable size overflow is reported
as `PASS WITH WARNINGS`.

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
SlimeVR packet writer and paired packet 17/4 output-runtime behavior
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

The firmware now exposes SlimeVR Server serial-compatibility commands for initial provisioning: `SET WIFI`, `SET BWIFI`, `GET INFO`, `GET CONFIG`, `GET TEST`, `GET WIFISCAN`, `REBOOT`, `FRST`, `DELCAL`, and temperature-only `TCAL`. `SET WIFI`/`SET BWIFI` save credentials into the separate network NVS config, enable Wi-Fi/discovery, and restart SlimeVR discovery. Blocking Wi-Fi scans suppress expected FIFO-recovery console noise for a short grace window without disabling recovery, counters, or machine-log events. `TCAL SAVE` is intentionally scoped to gyro temperature compensation so host-side temperature-calibration commands cannot accidentally capture unrelated runtime output/accel state. `TCAL RESET` changes only the in-RAM temperature compensation slope/quality metadata and does not mutate the persistent config object unless a later explicit temperature save is requested.

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
