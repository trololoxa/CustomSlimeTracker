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
- `runtime/runtime_bias_types.hpp` state reset semantics.
- `sensor/mag_yaw_correction.hpp` gate/reject/cooldown behavior.
- `config/tracker_config_detail.hpp` CRC helpers and schema constants.
- `config/tracker_config_schema.hpp` default schema layout expectations.

Native tests are intentionally not a replacement for firmware tests. They do
not verify SPI, GPIO interrupts, LSM6DSV FIFO timing, QMC6309 sensor-hub traffic,
NVS/Preferences, Serial output, or CPU timing on the ESP32-C3.

## Running standalone tests

From the project root:

```bash
python tools/run_standalone_tests.py --clean
```

The runner compiles every `tests/native/test_*.cpp` as a separate executable
into:

```text
build/native_tests/
```

It uses `CXX` when set, otherwise tries `g++`, `clang++`, then `c++`.

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

Do not add Arduino headers or ESP32-only APIs to native tests. If a header needs
Arduino to compile, it is not a native-test target until that dependency is
isolated behind an interface or guarded with `#ifdef ARDUINO`.

## Running firmware diagnostics

Firmware diagnostics still need PlatformIO and the ESP32-C3:

```bash
pio run -e BOARD_LOLIN_C3_MINI
pio run -e BOARD_LOLIN_C3_MINI_DIAG
pio run -e BOARD_LOLIN_C3_MINI -t upload
pio device monitor
```

Recommended smoke-test commands after an architectural change:

```text
status
health
config nvs
fifo stats
quality stats
ahrs status
bias status
mag status
stream quat
stream off
test static 120
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
- Window accept/reject criteria when extracted into pure helpers.
- Clamp/gain rules when they become host-safe.

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
  selected runtime/* pure state/types
  config/* schema/detail headers

Avoid in native tests:
  app/*
  serial/* command dispatcher
  Arduino-dependent runtime modules
  Preferences/NVS
  SPI/GPIO/Serial
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
static-test scalar/vector stats
static temp bin indexing
runtime gyro bias reset state
mag yaw gate/reject/cooldown behavior
config detail CRC/schema defaults
```

This is a starting point, not complete coverage. The next useful native-test
expansions are:

```text
config validate/sanitize helpers once host-safe
mag runtime processing after decoupling from FIFO raw sample type
runtime bias decision windows after extracting pure evaluator helpers
replay-driven AHRS regression tests from saved machine logs
```
