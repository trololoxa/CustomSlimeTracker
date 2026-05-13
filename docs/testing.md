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

The script always runs native tests unless `--skip-native` is passed. It also runs PlatformIO builds when `pio`/`platformio` is available in `PATH`. If PlatformIO is not installed, ESP32 builds are skipped by default so host-only development machines can still run the native gate. To make missing PlatformIO a failure, use:

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

Git Bash/WSL:

```bash
PIO=/c/Users/you/.platformio/penv/Scripts/platformio.exe python tools/check_all.py --require-pio
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
warning profile:

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
to test app, SPI, GPIO, Preferences/NVS, Wi-Fi or Serial transport behavior. If a
module needs real Arduino framework semantics, it remains firmware-test only
until the pure decision rule is isolated behind a host-safe helper.

## Running firmware diagnostics

Firmware diagnostics still need PlatformIO and the ESP32-C3. You can run them directly or through the quality gate:

```bash
python tools/check_all.py --require-pio
```

Direct commands:

```bash
pio run -e BOARD_LOLIN_C3_MINI
pio run -e BOARD_LOLIN_C3_MINI_DIAG
pio run -e BOARD_LOLIN_C3_MINI -t upload
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
output status
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
output status
test status
cal temp print
```

If IMU/FIFO live reconfiguration was touched, verify that the magnetometer path
is re-armed correctly after the change:

```text
imu odr 240 save
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
  config/* schema/detail headers
  the minimal tests/native/Arduino.h stub for Stream/millis-only helpers

Avoid in native tests:
  app/*
  serial/* command dispatcher
  Preferences/NVS
  SPI/GPIO/Serial/Wi-Fi
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
