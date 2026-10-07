# Changing firmware code

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

Select host regression/policy checks and affected builds through [test map](test_map.md).
For changed hardware integration or a timing claim, follow the
[hardware test budget](device_smoke.md#hardwareruntime-test-budget). A long static
capture is not mandatory for every hot-path edit; required target evidence cannot
be replaced by host PASS. When a static board capture is selected, inspect:

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

Select the affected profiles through [test map](test_map.md) and
[build profiles](../reference/build_profiles.md); full/release scope retains its
complete matrix. Use the explicit PlatformIO executable from [session](session.md).
Examples in an initialized PowerShell session:

```powershell
& $TrackerPio run -e BOARD_LOLIN_C3_MINI_DEBUG
& $TrackerPio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
& $TrackerPio run -e BOARD_LOLIN_C3_MINI_SLIM
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
runtime/static_test_runner.cpp
runtime/mag_runtime_controller.cpp
serial/tracker_mag_commands.cpp
config/tracker_config_runtime.cpp
```

If a file contains multiple unrelated domains, split it before adding more logic.

## How to add common changes

### Add a new CLI command

- Domain command declarations: `serial/tracker_*_commands.hpp`.
- Domain command behavior: `serial/tracker_*_commands.cpp`.
- Context fields/hooks: `serial/tracker_serial_context.hpp`.
- Wiring: `app/tracker_command_wiring.cpp` and/or `app/tracker_app_hooks.hpp`.
- Help text: `serial/tracker_system_commands.cpp`.

### Add a new persisted setting

- Schema: `config/tracker_config_schema.hpp`.
- Defaults/sanitize/apply/capture: `config/tracker_config_runtime.cpp`.
- NVS/version considerations: `config/tracker_config_detail.hpp` and `config/tracker_config_store.cpp`.
- Printout: `config/tracker_config_print.cpp`.
- CLI behavior if needed: `serial/tracker_config_commands.cpp` or the owning domain.
- Matching `.hpp` files declare the APIs used by callers.

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
- PlatformIO discovers `.cpp` files under `src/`, subject to each environment's
  `build_src_filter`. Verify affected profile inclusion/exclusion through
  [source filters](../reference/source_filters.md). Native runner source lists are
  separate; discovery by PlatformIO does not prove host linkage or test coverage.

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
