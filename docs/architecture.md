# Tracker firmware architecture

This document describes the current project layout, ownership rules, and extension points for the ESP32-C3 + LSM6DSV tracker firmware.

It is written for future maintainers and agents. Before adding code, read this file and keep the layer boundaries intact.

## Current high-level goal

The firmware is responsible for the physical sensor side of tracking:

- read IMU/magnetometer data reliably;
- reconstruct stable timestamps;
- apply local calibration;
- run local sensor fusion;
- expose diagnostics and calibration commands;
- eventually output tracker orientation to a host/server.

The firmware should **not** own body model logic, SlimeVR mounting/body calibration, skeleton offsets, or recenter semantics. Those belong to the host/server side. The firmware should produce a good sensor/device orientation and robust health information.

## Directory map

```text
src/
  main.cpp                         Arduino entrypoint only
  defines.h                        compile-time board/default constants

  app/                             top-level composition/wiring layer
  config/                          persisted config schema, store, print, runtime apply
  connection/                      low-level hardware/protocol drivers
  core/                            shared math primitives
  runtime/                         firmware runtime controllers and diagnostics
  sensor/                          sensor math, calibration, fusion, quality models
  serial/                          serial CLI parser and command domains

docs/
  architecture.md                  this document
  code_quality_roadmap_review.md   historical review / execution plan

tools/
  logs/                            host-side log parsing/debug helpers
```

## Layer responsibilities

### `main.cpp`

`main.cpp` must stay tiny.

Allowed:

```cpp
#include <Arduino.h>
#include "app/tracker_app_context.hpp"

void setup() {
    trackerAppContextSetup();
}

void loop() {
    trackerAppContextLoop();
}
```

Not allowed in `main.cpp`:

- hardware objects;
- globals/counters;
- CLI handlers;
- config logic;
- FIFO processing;
- AHRS or calibration logic;
- status printing;
- output formatting.

If `main.cpp` grows beyond a small entrypoint, something is being added to the wrong layer.

### `app/`

`app/` is the composition layer. It wires modules together, owns firmware-wide singletons, and defines the setup/loop orchestration.

Files:

```text
app/tracker_app.hpp
app/tracker_app_context.hpp
app/tracker_hardware_context.hpp
app/tracker_runtime_context.hpp
app/tracker_app_hooks.hpp
app/tracker_bootstrap.hpp
app/tracker_command_wiring.hpp
```

#### `tracker_app.hpp`

Owns setup/loop orchestration:

- serial startup;
- config/bootstrap sequence;
- LSM/FIFO initialization sequence;
- calibration IO setup;
- mag runtime startup from config;
- CLI setup;
- runtime loop order.

This file decides **when** things happen, not low-level details of **how** they work.

The loop order is intentionally simple:

```text
CLI poll
FIFO/runtime process
CLI poll
heartbeat/output maintenance
```

Do not put sensor math or command parsing here.

#### `tracker_hardware_context.hpp`

Owns board/hardware singletons and ISR-owned state:

- pin/default constants imported from `defines.h`;
- SPI transport;
- LSM6DSV driver;
- FIFO reader;
- LSM sensor hub;
- QMC6309 magnetometer;
- FIFO raw buffers;
- mag raw buffers;
- INT1 ISR counters and `onFifoInt1()`.

This is the only app-layer file that should directly own hardware driver instances.

#### `tracker_runtime_context.hpp`

Owns logical runtime singletons:

- `TrackerConfig` and `TrackerConfigStore`;
- `ImuCalibration`;
- `GyroTempCompensator`;
- `ImuQualityMonitor`;
- AHRS instance;
- serial CLI state;
- static test runner;
- runtime gyro-bias state;
- mag runtime state;
- prepared output runtime;
- tracking state controller;
- machine-log counters;
- performance counters.

Hardware objects do not belong here. Runtime logic can reference hardware only through dependency structs built in `tracker_app_hooks.hpp`.

#### `tracker_app_hooks.hpp`

This is the glue edge between app-owned objects and domain modules.

It contains:

- dependency builders;
- command hooks;
- FIFO/sample pipeline callbacks;
- mag runtime callbacks;
- status/log callbacks;
- `TrackerAppDeps` builder.

This file may be relatively large, because it is the explicit boundary where modules are connected. Keep it as wiring, not domain logic. If a hook grows into an algorithm, move the algorithm into `runtime/`, `sensor/`, `connection/`, or `serial/`.

#### `tracker_bootstrap.hpp`

Owns boot/init helpers:

- config load and runtime apply;
- product calibration validity enforcement;
- legacy runtime config migration;
- SPI runtime frequency apply;
- LSM initialization;
- FIFO initialization;
- calibration IO setup.

This file may call config/runtime/hardware APIs, but should not contain long-running runtime processing.

#### `tracker_command_wiring.hpp`

Owns mechanical construction of `TrackerSerialCommandContext`.

It should contain assignments and hook wiring only. Command behavior belongs in `serial/*_commands.hpp` files.

### `config/`

`config/` owns persistent configuration and NVS storage.

Files:

```text
config/tracker_config.hpp            umbrella include
config/tracker_config_detail.hpp     constants, CRC helpers, small utilities
config/tracker_config_schema.hpp     persistent schema structs only
config/tracker_config_runtime.hpp    runtime apply/capture/sanitize/validate
config/tracker_config_store.hpp      NVS storage and inspection
config/tracker_network_config.hpp    future Wi-Fi/SlimeVR network config storage
config/tracker_config_print.hpp      config summary printers
```

#### Rules for config changes

1. **Do not put runtime counters in persisted config.**

   Examples that must stay runtime-only:

   - current quaternion;
   - FIFO counters;
   - quality counters;
   - recovery counters;
   - mag heading auto-reference runtime state;
   - runtime gyro-bias trim unless explicitly promoted through a safe calibration workflow.

2. **Do not put Wi-Fi secrets in the main tracker config blob.**

   Main tracker config stores IMU/FIFO/AHRS/calibration/output/frame policy. Future network settings live in `TrackerNetworkConfig` under a separate namespace/key.

3. **Schema changes must be intentional.**

   If `TrackerConfigBlob` layout changes, update schema/block versions and consider NVS compatibility. A raw size/CRC mismatch can invalidate stored calibration.

4. **Separate persistent config from runtime application.**

   Persistent fields go in `tracker_config_schema.hpp`. Runtime conversion belongs in `tracker_config_runtime.hpp`.

5. **Printing is not validation.**

   `tracker_config_print.hpp` only reports state. Validation/sanitize must stay in runtime/config logic.

### `connection/`

`connection/` owns low-level hardware/protocol code.

Files include:

```text
connection/lsm6dsv_driver.hpp
connection/lsm6dsv_fifo.hpp
connection/lsm6dsv_sensorhub.hpp
```

Rules:

- no app-layer dependency;
- no CLI command dependency;
- no body tracking semantics;
- no SlimeVR semantics;
- no long status reports;
- no global application state.

This layer may depend on Arduino/SPI/Stream only where necessary for hardware access or direct low-level diagnostics.

### `sensor/`

`sensor/` owns sensor-domain math and models.

Examples:

```text
sensor/ahrs_6dof.hpp
sensor/calibration.hpp
sensor/accel_6pos_calibration.hpp
sensor/fifo_calibrations.hpp
sensor/gyro_temperature_compensation.hpp
sensor/imu_quality.hpp
sensor/mag_calibration.hpp
sensor/mag_heading.hpp
sensor/mag_runtime.hpp
sensor/mag_yaw_correction.hpp
sensor/qmc6309.hpp
```

Rules:

- sensor math should be deterministic and testable;
- avoid serial printing unless the file is explicitly a command/progress helper;
- do not depend on app wiring;
- do not own global firmware objects;
- prefer input/output structs over hidden global state.

This layer is where calibration math, AHRS math, quality checks, mag heading, and mag yaw correction algorithms belong.

### `runtime/`

`runtime/` owns live firmware controllers built from sensor/config/connection primitives.

Examples:

```text
runtime/fifo_runtime_processor.hpp
runtime/imu_sample_pipeline.hpp
runtime/runtime_gyro_bias_controller.hpp
runtime/static_test_runner.hpp
runtime/output_runtime.hpp
runtime/mag_runtime_controller.hpp
runtime/runtime_status_reporter.hpp
runtime/machine_log_runtime.hpp
runtime/tracking_state_controller.hpp
runtime/gyro_temp_static_fit.hpp
```

Rules:

- runtime modules should receive dependencies through explicit `Deps` structs;
- no hidden hardware globals inside runtime modules;
- no direct include of the full CLI dispatcher;
- status/report modules may print, processing modules should minimize formatting;
- hot-path modules should avoid heap allocation and `String`;
- maintain current timing invariants unless a patch explicitly targets performance.

Important hot-path modules:

```text
fifo_runtime_processor.hpp   drains FIFO and dispatches raw samples
imu_sample_pipeline.hpp      raw IMU sample -> calibration -> quality -> AHRS -> output hooks
mag_runtime_controller.hpp   mag sample processing, heading reference, yaw correction glue
```

Changes to these files require `test static` regression checks.

### `serial/`

`serial/` owns the command-line interface.

Current domain split:

```text
serial/tracker_serial_commands.hpp          dispatcher + line parser
serial/tracker_serial_context.hpp           context/types
serial/tracker_serial_stream.hpp            stream emit helpers
serial/tracker_system_commands.hpp          help/status/health/system
serial/tracker_output_commands.hpp          stream/log/output
serial/tracker_bias_commands.hpp            runtime bias
serial/tracker_test_commands.hpp            static test
serial/tracker_config_commands.hpp          config/NVS
serial/tracker_imu_fifo_commands.hpp        IMU/FIFO/quality
serial/tracker_ahrs_commands.hpp            AHRS
serial/tracker_calibration_commands.hpp     gyro/accel/temp calibration
serial/tracker_mag_commands.hpp             mag/yaw
```

Rules:

- new commands go into the matching domain file;
- `tracker_serial_commands.hpp` should stay a dispatcher and parser;
- command handlers should call hooks or domain APIs, not implement long algorithms inline;
- avoid `String` and heap allocation;
- keep command output stable unless intentionally changing the CLI contract.

When adding a new command:

1. Add the handler to the correct `tracker_*_commands.hpp` domain.
2. Add a hook to `TrackerSerialCommandContext` only if the command needs app/runtime behavior that does not already exist.
3. Wire that hook in `app/tracker_command_wiring.hpp` / `app/tracker_app_hooks.hpp`.
4. Add the command to `help` in `tracker_system_commands.hpp`.
5. Smoke-test with Serial Monitor.

## Runtime data flow

### Boot flow

```text
main.cpp
  -> trackerAppContextSetup()
    -> TrackerApp::begin(makeTrackerAppDeps())
    -> TrackerApp::setup()
      -> Serial startup
      -> config load/defaults/sanitize/apply
      -> LSM init
      -> FIFO init
      -> calibration IO setup
      -> mag startup from config
      -> command context wiring
      -> runtime reset/ready
```

### Main loop flow

```text
main.cpp
  -> trackerAppContextLoop()
    -> TrackerApp::loop()
      -> CLI poll
      -> FIFO runtime process
          -> raw IMU samples -> imu_sample_pipeline
          -> raw mag samples -> mag_runtime_controller
      -> CLI poll
      -> heartbeat/output maintenance
```

### IMU sample flow

```text
Lsm6dsvFifoReader::RawSample
  -> ImuSamplePipeline
    -> scaled accel/gyro conversion
    -> gyro bias and temp compensation
    -> accel calibration
    -> quality monitor
    -> AHRS update
    -> runtime bias estimator
    -> static test runner
    -> prepared output / serial stream / machine log
```

### Mag sample flow

```text
LSM sensorhub FIFO mag sample
  -> MagRuntimeController
    -> calibration / axis alignment
    -> trust/reject gates
    -> heading calculation
    -> heading reference / auto-reference
    -> yaw correction gate
    -> optional AHRS yaw correction
    -> status/static-test/machine-log hooks
```

## Persistent vs runtime state

Persisted in NVS:

- hardware/IMU/FIFO defaults;
- AHRS config;
- gyro bias;
- gyro temp compensation model and metadata;
- accel calibration and quality metadata;
- mag calibration and quality metadata;
- mag yaw correction config;
- output policy;
- frame/device identity placeholders;
- future network config in separate storage.

Runtime only:

- live quaternion;
- tracking state transitions;
- FIFO/quality/performance counters;
- static test in-progress stats;
- mag heading auto-reference runtime state;
- prepared output snapshot;
- runtime gyro-bias trim;
- recovery state.

Rule of thumb: if a value is derived from the current boot/session, do not persist it unless there is a deliberate calibration/save command.

## Calibration ownership

Calibration is split intentionally:

- math/model structs live in `sensor/`;
- FIFO-compatible capture helpers live in `sensor/fifo_calibrations.hpp`;
- persistent storage lives in `config/`;
- command entrypoints live in `serial/tracker_calibration_commands.hpp`;
- long static-test temp fitting lives in `runtime/gyro_temp_static_fit.hpp`;
- app wiring lives in `app/tracker_app_hooks.hpp`.

Do not put calibration algorithms into CLI files. CLI files should only parse arguments, call the right module/hook, and print results.

## Output and SlimeVR boundary

Current production-safe output is serial/debug-oriented. Fake SlimeVR/binary output modes should not be enabled without a real transport/backend.

Future SlimeVR support should be added as separate modules, for example:

```text
net/wifi_manager.hpp
net/slimevr_protocol.hpp
net/slimevr_udp_transport.hpp
runtime/slimevr_output_runtime.hpp
```

The firmware should send local sensor/device orientation and health. It should not bake in server/body/mounting calibration semantics unless there is a clear protocol-level reason.

## Dependency rules

Use these rules when deciding where to put code:

```text
main -> app only
app -> config/runtime/serial/connection/sensor/core
runtime -> sensor/config/connection/core/serial context or stream helpers only
serial -> config/sensor/runtime types through context/hooks
sensor -> core and small dependencies only
connection -> hardware/core only
config -> sensor/connection types only where needed for runtime conversion
```

Avoid these dependencies:

```text
sensor -> app
sensor -> serial commands
connection -> app
connection -> serial commands
runtime -> app
runtime -> full tracker_serial_commands.hpp
config schema -> runtime controllers
```

`serial/tracker_serial_context.hpp` is allowed as a small shared context/types file. Do not include `serial/tracker_serial_commands.hpp` from runtime modules.

## Performance and hot-path rules

Hot-path code includes:

- FIFO drain;
- IMU sample pipeline;
- mag sample processing;
- AHRS update;
- runtime bias estimator.

Rules:

- no heap allocation;
- no `String`;
- no blocking waits;
- no verbose printing in per-sample paths;
- preserve hardware timestamp usage;
- preserve bounded FIFO drains;
- preserve recovery behavior unless explicitly changing it.

After changing hot-path code, run at minimum:

```text
status
fifo stats
quality stats
test static 120
```

Watch:

```text
estimated_dropped_samples
fifo_overrun_delta
fifo_full_delta
fallback_timestamp_samples
bad_timestamp_samples
accel_correction_disabled_samples
sample_process_avg_us
fifo_process_avg_us
yaw_drift_rate_deg_min
```

## Build and diagnostics expectations

Use both builds:

```bash
pio run -e BOARD_LOLIN_C3_MINI
pio run -e BOARD_LOLIN_C3_MINI_DIAG
```

The diagnostic build should stay warning-clean. If a warning appears, fix it before continuing feature work.

Do not hide warnings by broad suppression unless the warning comes from external framework code and cannot be fixed locally.

## File size and responsibility guidance

These are soft limits, not hard rules, but they capture the intended architecture:

```text
main.cpp                         tiny entrypoint
serial/tracker_serial_commands   dispatcher/parser only
config/tracker_config.hpp        umbrella include only
app/tracker_app_context.hpp      top-level context entrypoints only
```

Large files are acceptable only when they own a coherent domain, for example:

```text
runtime/static_test_runner.hpp
runtime/mag_runtime_controller.hpp
serial/tracker_mag_commands.hpp
config/tracker_config_runtime.hpp
```

If a file contains multiple unrelated domains, split it before adding more logic.

## How to add common changes

### Add a new CLI command

- Domain command logic: `serial/tracker_*_commands.hpp`.
- Context fields/hooks: `serial/tracker_serial_context.hpp`.
- Wiring: `app/tracker_command_wiring.hpp` and/or `app/tracker_app_hooks.hpp`.
- Help text: `serial/tracker_system_commands.hpp`.

### Add a new persisted setting

- Schema: `config/tracker_config_schema.hpp`.
- Defaults/sanitize/apply/capture: `config/tracker_config_runtime.hpp`.
- NVS/version considerations: `config/tracker_config_detail.hpp` and `config/tracker_config_store.hpp`.
- Printout: `config/tracker_config_print.hpp`.
- CLI command if needed: `serial/tracker_config_commands.hpp` or a domain-specific CLI file.

### Add new sensor math

- Pure math/model: `sensor/`.
- Live firmware orchestration: `runtime/`.
- Configuration: `config/`.
- Command exposure: `serial/`.
- Wiring: `app/`.

### Add new network/SlimeVR support

- Do not put Wi-Fi secrets in `TrackerConfigBlob`.
- Use `TrackerNetworkConfig` or a future `net/` layer.
- Keep protocol encoding separate from transport.
- Keep output runtime separate from sensor fusion.

## Current known follow-up areas

Architecture is now much cleaner, but these areas remain future work:

1. **Host-side tests/replay.**
   Add tests for config validation, timestamp reconstruction, runtime bias acceptance, AHRS invariants, mag yaw gates.

2. **Dependency cleanup.**
   Remove unnecessary includes and replace heavy includes with forward declarations where practical.

3. **Mag/FIFO performance.**
   Mag-enabled runs increase FIFO processing time. Optimize only after preserving current diagnostics and behavior.

4. **Config migration policy.**
   Current schema is versioned, but future changes should consider block-level migrations instead of invalidating all calibration.

5. **Network/SlimeVR backend.**
   Add only after local tracking, calibration, and health reporting remain stable.

## Final rule

When in doubt, keep the boundary clean:

```text
sensor code computes
runtime code runs
serial code commands
config code persists
connection code talks to hardware
app code wires
main.cpp enters
```
