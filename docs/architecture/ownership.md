# Tracker firmware architecture

This document describes the current project layout, ownership rules, and extension points for the ESP32-C3 + LSM6DSV tracker firmware.

Read the affected layer and its dependencies before adding code; keep these
boundaries intact. Use [implementation](implementation.md) for the current runtime
baseline and [status](../status.md) for acceptance gaps when relevant to the task.
Historical roadmaps are planning records and do not override current source.

## Current high-level goal

The firmware is responsible for the physical sensor side of tracking:

- read IMU/magnetometer data reliably;
- reconstruct stable timestamps;
- apply local calibration;
- run local sensor fusion;
- expose diagnostics and calibration commands;
- output tracker orientation through local serial developer streams and the SlimeVR UDP backend.

The firmware should **not** own body model logic, SlimeVR mounting/body calibration, skeleton offsets, or recenter semantics. Those belong to the host/server side. The firmware should produce a good sensor/device orientation and robust health information.

## Directory map

```text
src/
  main.cpp                         Arduino entrypoint only
  defines.h                        compatibility umbrella
  build_config/                    canonical profiles, feature flags and tuning

  app/                             top-level composition/wiring layer
  config/                          persisted config schema, store, print, runtime apply
  connection/                      low-level hardware/protocol drivers
  core/                            shared math primitives
  runtime/                         firmware runtime controllers and diagnostics
                                   includes status LED, tap accumulator and SlimeVR output runtimes
  sensor/                          sensor math, calibration, fusion, quality models
  serial/                          serial CLI parser and command domains
  network/                         real Wi-Fi station management and UDP transport primitives
  output/                          host-safe SlimeVR packet writer/protocol helpers

docs/
  README.md                        documentation index
  architecture/ownership.md        this document
  architecture/implementation.md   current runtime baseline
  status.md                        current readiness and limitations
  architecture/module_inventory.md ownership map
  development/testing.md           host/firmware test strategy
  reference/cli.md                 serial command behavior and side effects
  reference/configuration.md       persisted config policy
  architecture/tracking_pipeline.md runtime data flow
  development/replay.md            machine-log replay/metrics workflow

tools/
  logs/                            host-side log parsing/debug helpers
  replay/                          replay/metrics helpers for machine logs
  validate_*.py                    profile/source/documentation contracts
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
app/tracker_app.hpp / app/tracker_app.cpp
app/tracker_app_context.hpp
app/tracker_hardware_context.hpp
app/tracker_runtime_context.hpp
app/tracker_app_hooks.hpp
app/hooks/tracker_app_common_hooks.hpp
app/hooks/tracker_app_mag_hooks.hpp
app/hooks/tracker_app_command_hooks.hpp
app/hooks/tracker_app_runtime_hooks.hpp
app/tracker_bootstrap.hpp / app/tracker_bootstrap.cpp
app/tracker_command_wiring.hpp / app/tracker_command_wiring.cpp
```

#### `tracker_app.hpp` / `tracker_app.cpp`

The header declares `TrackerApp` and its dependencies. `TrackerApp::setup()`
and `TrackerApp::loop()` in the `.cpp` own orchestration:

- serial startup;
- config/bootstrap sequence;
- LSM/FIFO initialization sequence;
- calibration IO setup;
- mag runtime startup from config;
- CLI setup;
- runtime loop order.

This module decides **when** things happen, not low-level details of **how** they work.

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

#### `tracker_app_hooks.hpp` and `app/hooks/`

This is the glue edge between app-owned objects and domain modules.

`tracker_app_hooks.hpp` is now a small umbrella that includes focused hook sections:

```text
app/hooks/tracker_app_common_hooks.hpp   counters, log glue, FIFO wait glue, bootstrap deps, runtime-bias hooks
app/hooks/tracker_app_mag_hooks.hpp      mag runtime callbacks, mag status hooks, tracking-state glue
app/hooks/tracker_app_command_hooks.hpp  command/status/static-test hook wiring
app/hooks/tracker_app_runtime_hooks.hpp  sample-pipeline callbacks and final TrackerAppDeps builder
```

These hook files are intentionally still include-only because they bind together app-level static singletons owned by `tracker_hardware_context.hpp` and `tracker_runtime_context.hpp`. The split is for readability and ownership clarity; it must not introduce runtime allocation, virtual dispatch, or new module ownership.

Keep these files as wiring, not domain logic. If a hook grows into an algorithm, move the algorithm into `runtime/`, `sensor/`, `connection/`, or `serial/`.

#### `tracker_bootstrap.hpp` / `tracker_bootstrap.cpp`

The header declares boot/init helpers; the `.cpp` implements:

- config load and runtime apply;
- product calibration validity enforcement;
- legacy runtime config migration;
- SPI runtime frequency apply;
- LSM initialization;
- FIFO initialization;
- calibration IO setup.

This module may call config/runtime/hardware APIs, but should not contain long-running runtime processing.

#### `tracker_command_wiring.hpp` / `tracker_command_wiring.cpp`

The `.cpp` owns mechanical construction of `TrackerSerialCommandContext`.

It should contain assignments and hook wiring only. Command behavior belongs in `serial/*_commands.cpp`; matching headers declare the API.

### `config/`

`config/` owns persistent configuration and NVS storage.

Files:

```text
config/tracker_config.hpp            umbrella include
config/tracker_config_detail.hpp     constants, CRC helpers, small utilities
config/tracker_config_schema.hpp     persistent schema structs only
config/tracker_config_runtime.hpp    runtime apply/capture/sanitize/validate
config/tracker_config_store.hpp      NVS storage and inspection
config/tracker_network_config.hpp    Wi-Fi/SlimeVR network config storage
config/tracker_config_print.hpp      config summary printers
```

Runtime/store/print/network headers declare APIs; their matching `.cpp` files
own behavior. Schema/detail headers retain types and small helpers.

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

   Main tracker config stores IMU/FIFO/AHRS/calibration/output/frame policy. Network credentials and SlimeVR endpoint settings live in `TrackerNetworkConfig` under a separate namespace/key.

3. **Schema changes must be intentional.**

   If `TrackerConfigBlob` layout changes, update schema/block versions and consider NVS compatibility. A raw size/CRC mismatch can invalidate stored calibration.

4. **Separate persistent config from runtime application.**

   Persistent fields go in `tracker_config_schema.hpp`. Runtime conversion is implemented in `tracker_config_runtime.cpp`.

5. **Printing is not validation.**

   `tracker_config_print.cpp` only reports state. Validation/sanitize must stay in runtime/config logic.

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
sensor/ahrs_6dof.hpp / sensor/ahrs_6dof.cpp
sensor/calibration.hpp
sensor/accel_6pos_calibration.hpp
sensor/fifo_calibrations.hpp
sensor/gyro_temperature_compensation.hpp
sensor/imu_quality.hpp / sensor/imu_quality.cpp
sensor/mag_calibration.hpp
sensor/mag_heading.hpp
sensor/mag_runtime.hpp / sensor/mag_runtime.cpp
sensor/mag_yaw_correction.hpp / sensor/mag_yaw_correction.cpp
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

These headers declare APIs; read their matching `.cpp` implementations. Select
checks through [test map](../development/test_map.md) and the
[hardware test budget](../development/device_smoke.md#hardwareruntime-test-budget).
A long `test static` capture is not required for every edit; target timing and
hardware evidence remain required when the changed contract needs them.

### `serial/`

`serial/` owns the command-line interface.

Current domain split:

```text
serial/tracker_serial_commands.hpp          line parser template + dispatcher declaration
serial/tracker_serial_commands.cpp          command router implementation
serial/tracker_serial_context.hpp           context/types
serial/tracker_serial_stream.hpp            stream emit helpers
serial/tracker_system_commands.hpp/.cpp      help/status/health/system
serial/tracker_output_commands.hpp/.cpp      stream/log/output
serial/tracker_bias_commands.hpp/.cpp        runtime bias
serial/tracker_test_commands.hpp/.cpp        static test
serial/tracker_config_commands.hpp/.cpp      config/NVS
serial/tracker_imu_fifo_commands.hpp/.cpp    IMU/FIFO/quality
serial/tracker_ahrs_commands.hpp/.cpp        AHRS
serial/tracker_calibration_commands.hpp/.cpp gyro/accel/temp calibration
serial/tracker_mag_commands.hpp/.cpp         mag/yaw
```

Rules:

- new commands go into the matching domain pair; put declarations in `*.hpp` and implementation in `*.cpp`;
- `tracker_serial_commands.hpp` should stay fixed-buffer parser/template glue only; routing lives in `tracker_serial_commands.cpp`;
- command `.cpp` files must include the concrete headers for every forwarded type they dereference; do not rely on transitive includes from the old header-only layout;
- helpers shared between command domains should be declared intentionally in the owning domain header, for example mag re-arm helpers used by IMU/FIFO reconfiguration;
- command handlers should call hooks or domain APIs, not implement long algorithms inline;
- avoid `String` and heap allocation;
- keep command output stable unless intentionally changing the CLI contract.

When adding a new command:

1. Declare the handler/helper in the correct `tracker_*_commands.hpp` domain only if other translation units need it.
2. Implement command behavior in the matching `tracker_*_commands.cpp`.
3. Add a hook to `TrackerSerialCommandContext` only if the command needs app/runtime behavior that does not already exist.
4. Wire that hook in `app/tracker_command_wiring.cpp` / `app/tracker_app_hooks.hpp`.
5. Add the command to `help` in `tracker_system_commands.cpp`.
6. Run affected host/CLI checks and builds; select a focused board smoke when
   the hardware/transport integration changes, following the device guide above.
