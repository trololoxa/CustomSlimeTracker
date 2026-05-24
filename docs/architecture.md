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
- output tracker orientation through local serial developer streams and the SlimeVR UDP backend.

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
                                   includes status LED, tap accumulator and SlimeVR output runtimes
  sensor/                          sensor math, calibration, fusion, quality models
  serial/                          serial CLI parser and command domains
  network/                         real Wi-Fi station management and UDP transport primitives
  output/                          host-safe SlimeVR packet writer/protocol helpers

docs/
  architecture.md                  this document
  project_status.md                current structural baseline
  module_inventory.md              ownership map
  testing.md                       host/firmware test strategy
  cli_reference.md                 serial command behavior and side effects
  config_schema.md                 persisted config policy
  tracking_pipeline.md             runtime data flow
  replay.md                        machine-log replay/metrics workflow

tools/
  logs/                            host-side log parsing/debug helpers
  replay/                          replay/metrics helpers for machine logs
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
app/hooks/tracker_app_common_hooks.hpp
app/hooks/tracker_app_mag_hooks.hpp
app/hooks/tracker_app_command_hooks.hpp
app/hooks/tracker_app_runtime_hooks.hpp
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
config/tracker_network_config.hpp    Wi-Fi/SlimeVR network config storage
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

   Main tracker config stores IMU/FIFO/AHRS/calibration/output/frame policy. Network credentials and SlimeVR endpoint settings live in `TrackerNetworkConfig` under a separate namespace/key.

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

Changes to these files require `test static` regression checks.

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
4. Wire that hook in `app/tracker_command_wiring.hpp` / `app/tracker_app_hooks.hpp`.
5. Add the command to `help` in `tracker_system_commands.cpp`.
6. Smoke-test with Serial Monitor.

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
- frame/device identity fields;
- network/SlimeVR config in separate storage.

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

There are two output paths with separate ownership:

- local serial developer output: `stream ...`, `output ...`, and machine logs;
- SlimeVR UDP output: `net ...` / `slime ...`, using prepared quaternion snapshots.

The SlimeVR backend is real Wi-Fi/UDP transport, not a fake packet mode. Custom binary output is still not implemented and should remain disabled until it has a real backend.

Current SlimeVR-related modules:

```text
network/wifi_manager.hpp
network/udp_transport.hpp
output/slimevr_packet_writer.hpp
runtime/slimevr_output_runtime.hpp
runtime/status_led_runtime.hpp
```

`src/network/` owns Wi-Fi/UDP transport primitives only. `src/output/` owns packet encoding, and `runtime/slimevr_output_runtime.*` owns discovery/session/output scheduling. `runtime/status_led_runtime.*` owns only GPIO LED pattern timing and status display; it must consume high-level runtime states, not sensor samples. Do not place AHRS/FIFO logic in transport modules, and do not place transport state in AHRS or mag-yaw code.

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

`serial/tracker_serial_context.hpp` is allowed as a small shared context/types file. Do not include `serial/tracker_serial_commands.hpp` from runtime modules. Keep `tracker_serial_context.hpp` lightweight: use forward declarations for concrete config/driver/runtime classes, and include concrete headers only in the command domain files that dereference those objects. Shared token parsing helpers belong in `serial/tracker_serial_parse.hpp`; shared formatting helpers belong in `serial/tracker_serial_print.hpp`.

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

Use the explicit profile builds:

```bash
pio run -e BOARD_LOLIN_C3_MINI_DEBUG
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_SLIM
```

The Debug build should stay warning-clean. If a warning appears, fix it before continuing feature work.

Do not hide warnings by broad suppression unless the warning comes from external framework code and cannot be fixed locally.

## File size and responsibility guidance

These are soft limits, not hard rules, but they capture the intended architecture:

```text
main.cpp                         tiny entrypoint
serial/tracker_serial_commands   parser/template glue only; router in .cpp
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

- Domain command declarations: `serial/tracker_*_commands.hpp`.
- Domain command behavior: `serial/tracker_*_commands.cpp`.
- Context fields/hooks: `serial/tracker_serial_context.hpp`.
- Wiring: `app/tracker_command_wiring.hpp` and/or `app/tracker_app_hooks.hpp`.
- Help text: `serial/tracker_system_commands.cpp`.

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

### Change network/SlimeVR support

- Do not put Wi-Fi secrets in `TrackerConfigBlob`.
- Use `TrackerNetworkConfig` for Wi-Fi credentials and SlimeVR endpoint settings.
- Keep protocol encoding separate from transport.
- Keep output runtime separate from sensor fusion.

## Header and implementation split policy

The project started as mostly header-only firmware code. That made early refactors easy, but large runtime/domain modules should not stay header-only forever. Use this rule:

- Keep small type-only files header-only: `*_types.hpp`, `tracker_config_schema.hpp`, `tracker_config_detail.hpp`, tiny math helpers.
- Move large behavior/reporting/controller implementations to `.cpp` once their public API is stable.
- Do not split a file only to chase line counts; split when it reduces compile dependencies or hides implementation details.
- `.hpp` files should expose domain APIs and data needed by callers. `.cpp` files should own printing, formatting, state-machine internals, and heavy includes.
- Arduino/PlatformIO automatically compiles `.cpp` files under `src/`; no separate build registration is needed.

Current split status:

```text
runtime/static_test_runner.hpp        -> static_test_runner.cpp       done
runtime/machine_log_runtime.hpp       -> machine_log_runtime.cpp      done
runtime/runtime_status_reporter.hpp   -> runtime_status_reporter.cpp  done
runtime/mag_status_reporter.hpp       -> mag_status_reporter.cpp      done
runtime/output_runtime.hpp            -> output_runtime.cpp           done
runtime/gyro_temp_static_fit.hpp      -> gyro_temp_static_fit.cpp     done
runtime/runtime_gyro_bias_controller.hpp -> runtime_gyro_bias_controller.cpp done
runtime/fifo_runtime_processor.hpp       -> fifo_runtime_processor.cpp       done
runtime/imu_sample_pipeline.hpp          -> imu_sample_pipeline.cpp          done
runtime/tracking_state_controller.hpp    -> tracking_state_controller.cpp    done
runtime/mag_runtime_controller.hpp       -> mag_runtime_controller.cpp       done
config/tracker_config_runtime.hpp        -> tracker_config_runtime.cpp        done
config/tracker_config_store.hpp          -> tracker_config_store.cpp          done
config/tracker_config_print.hpp          -> tracker_config_print.cpp          done
config/tracker_network_config.hpp        -> tracker_network_config.cpp        done
app/tracker_app.hpp                      -> tracker_app.cpp                     done
app/tracker_bootstrap.hpp                -> tracker_bootstrap.cpp               done
app/tracker_command_wiring.hpp           -> tracker_command_wiring.cpp          done
sensor/gyro_temperature_compensation.hpp -> gyro_temperature_compensation.cpp done
sensor/mag_yaw_correction.hpp            -> mag_yaw_correction.cpp            done
sensor/accel_6pos_calibration.hpp        -> accel_6pos_calibration.cpp        done
sensor/mag_calibration.hpp               -> mag_calibration.cpp               done
sensor/mag_heading.hpp                   -> mag_heading.cpp                   done
sensor/mag_runtime.hpp                   -> mag_runtime.cpp                   done
sensor/ahrs_6dof.hpp                     -> ahrs_6dof.cpp                    done
sensor/imu_quality.hpp                   -> imu_quality.cpp                  done
connection/lsm6dsv_driver.hpp            -> lsm6dsv_driver.cpp             done
connection/lsm6dsv_fifo.hpp              -> lsm6dsv_fifo.cpp               done
connection/lsm6dsv_sensorhub.hpp         -> lsm6dsv_sensorhub.cpp          done
sensor/qmc6309.hpp                       -> qmc6309.cpp                    done
```

Command-domain modules are now split into `.hpp/.cpp` pairs. Keep only declarations and tiny parser/template glue in headers. The app hook glue has been split into smaller include-only files, but should not move to `.cpp` until app globals are represented by an explicit owned context object.

```text
serial/*_commands.hpp                    -> command .cpp files             done
serial/tracker_serial_commands.hpp       -> parser template glue only       done
sensor/fifo_calibrations.hpp             -> calibration I/O .cpp, optional after more tests
sensor/calibration.hpp                   -> partial .cpp, optional after more tests
```

Hardware-facing connection drivers are split into `.cpp` files now. Keep their headers as public driver APIs only; avoid adding high-level tracking, calibration, or CLI logic to `connection/`.

Keep these app ownership/context headers header-only for now:

```text
app/tracker_app_context.hpp
app/tracker_hardware_context.hpp
app/tracker_runtime_context.hpp
```

They define the firmware composition layer and currently own static singletons for the single Arduino translation unit that includes `main.cpp`. Moving `tracker_app_hooks.hpp` or `app/hooks/*` to `.cpp` should wait until those globals are represented by an explicit `TrackerAppContext` object instead of header-level static ownership.

Avoid moving tiny structs or schema definitions to `.cpp`; they are useful as lightweight shared declarations and native-test inputs.

## Current known follow-up areas

Architecture is now much cleaner, but these areas remain future work:

1. **Host-side tests/replay.**
   Add tests for config validation, timestamp reconstruction, runtime bias acceptance, AHRS invariants, mag yaw gates.

2. **Dependency cleanup.**
   Continue replacing umbrella includes with domain-specific includes where practical. The first pass made the shared serial context lightweight; future passes should focus on app/runtime dependency builders without obscuring ownership.

3. **Mag/FIFO performance.**
   Mag-enabled runs increase FIFO processing time. Optimize only after preserving current diagnostics and behavior.

4. **Config migration policy.**
   Current schema is versioned, but future changes should consider block-level migrations instead of invalidating all calibration.

5. **Network/SlimeVR hardening.**
   Keep runtime tests, reconnect behavior, and packet diagnostics current as Wi-Fi/server behavior changes.

## Final rule

When in doubt, keep the boundary clean:

```text
sensor code computes
runtime code runs
serial code commands
config code persists
connection code talks to hardware
network code transports
output code encodes packets
app code wires
main.cpp enters
```


## Sensor calibration implementation split status

The sensor calibration layer now follows the public-header/private-implementation rule:

- `sensor/calibration.hpp` declares IMU calibration, stationary detection, startup gyro calibration, and online gyro bias APIs.
- `sensor/calibration.cpp` implements those algorithms.
- `sensor/fifo_calibrations.hpp` keeps the FIFO drain template in the header but moves the non-template calibration runners to `sensor/fifo_calibrations.cpp`.

This keeps command/config/app code from recompiling the calibration implementation in every translation unit while preserving the templated FIFO drain helper where it belongs.

### RC1 serial provisioning compatibility


Battery ADC runtime reads the RC1 divider `BAT+ -> R_TOP -> GPIO -> R_BOTTOM -> GND` with defaults `GPIO4`, `R_TOP=180 kΩ`, and `R_BOTTOM=180 kΩ`. GPIO4 is ESP32-C3 ADC1_CH4, the supported ADC path for battery telemetry. The runtime samples sparsely (`TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS`, default 10000 ms), takes a small median-filtered ADC burst, maps 3.30-4.20 V to 0-100%, and reports safe 0.000 V / 0.0% when the divider is absent, below the present threshold, invalid, or unreadable. The value is exposed through `battery status`, `GET INFO`, `slime status`, and periodic SlimeVR BatteryLevel telemetry.

The firmware now exposes SlimeVR Server serial-compatibility commands for initial provisioning: `SET WIFI`, `SET BWIFI`, `GET INFO`, `GET CONFIG`, `GET TEST`, `GET WIFISCAN`, `REBOOT`, `FRST`, `DELCAL`, and temperature-only `TCAL`. `SET WIFI`/`SET BWIFI` save credentials into the separate network NVS config, enable Wi-Fi/discovery, and restart SlimeVR discovery. Blocking Wi-Fi scans suppress expected FIFO-recovery console noise for a short grace window without disabling recovery, counters, or machine-log events. `TCAL SAVE` is intentionally scoped to gyro temperature compensation so host-side temperature-calibration commands cannot accidentally capture unrelated runtime output/accel state. `TCAL RESET` changes only the in-RAM temperature compensation slope/quality metadata and does not mutate the persistent config object unless a later explicit temperature save is requested.
