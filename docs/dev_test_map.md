# DEV-04: owners and test selection

This is a routing aid, not an exhaustive dependency graph or automatic acceptance.
Start with the changed owner; add callers and tests for the actual risk. Exact
available IDs come from `check_all.py --list-checks` and
`run_standalone_tests.py --list-tests` (no compilation).

| Changed owner / location | Native test IDs (starting set) | Policy / further evidence |
| --- | --- | --- |
| `src/sensor/ahrs_6dof.*`, frame transforms | `test_core_math_ahrs`, `test_sensor_to_device_alignment`, `test_slimevr_motion_frame` | `test_0028a_transaction_and_quaternion_hotpath_policy`; existing replay and frame conventions; target timing if hot path changed |
| `src/connection/lsm6dsv_fifo.*`, `src/runtime/fifo_runtime_processor.*`, tracking recovery | `test_fifo_runtime_processor`, `test_fifo_pair_coherency`, `test_sensor_liveness_recovery`, `test_tracking_state_controller` | 0027 policies from registry; IRQ/FIFO/reset smoke requires board |
| `src/config/`, calibration transaction/autonomy | `test_config_hardening`, `test_config_storage`, `test_calibration_transaction`, `test_calibration_autonomy`, `test_storage_real_scenarios`, `test_factory_reset_coordinator` | 0028 policies; failure injection and recoverable old-good; board persistence/reboot where affected |
| `src/sensor/mag_*`, `src/runtime/mag_runtime_controller.*` | `test_mag_calibration`, `test_mag_heading_reliability`, `test_mag_yaw_runtime_bias` | magnetic replay; 0026b stack/hot-path policy; do not accept disturbed field to raise pass rate |
| gyro bias/temperature runtime | `test_runtime_bias_controller`, `test_gyro_temp_compensation`, `test_gyro_temp_calibration_capture`, `test_gyro_temp_static_fit` | stationary/moving and stale-temp evidence; no new dataset unless existing evidence cannot test the changed claim |
| `src/network/`, `src/output/`, SlimeVR runtime | `test_slimevr_packet_writer`, `test_slimevr_output_runtime`, `test_wifi_manager`, `test_tracker_network_config` | existing session/packet policies; wire changes need current Server interoperability |
| CLI, sleep, tap, battery | corresponding `test_serial_parse_helpers`, `test_cli_transport_parity`, `test_motion_light_sleep_controller`, `test_tap_runtime_diagnostics`, `test_battery_runtime` | relevant command/transport tests; physical wake/tap/ADC needs target smoke |
| `tools/run_standalone_tests.py`, `check_all.py`, reporting | `test_core_math_ahrs` as a real focused native smoke when execution changes | `test_dev04_workflow`, `test_dev03_runners`, `test_run_standalone_tests_policy`, `test_check_all_aggregation_policy`, `test_quality_gate_runtime`; `test_dev03a_reporting` if report/process handling changes |
| documentation / instructions | none for prose-only changes | `validate_documentation`; review commands and claims against current source |
| profiles/build filters/toolchain | affected firmware environments | `validate_source_filters`, `validate_profile_matrix`; full relevant target build matrix, not a host-only claim |

Policy names above are IDs passed to `check_all.py --check`; native IDs go to
`run_standalone_tests.py --test`. Use multiple selectors in one invocation.
All policies are Python scripts; this does not make their child work Python-only.

## Expand checks with the changed contract

- Local contract fix: affected regression/negative cases + relevant policies.
- Timing/state/integration change: affected sequences, fault paths and build;
  baseline/target timing when changed or claimed. Inspect caller effects.
- Fusion/threshold/numerical change: existing replay, independent physical/math
  checks and comparison to baseline; new data only for an uncovered hypothesis.
- Release/platform/schema: complete required build/release/persistence/Server
  and relevant hardware evidence. Preserve rollback and explicit exclusions.

This routing aid never waives the existing L0-L3/subsystem acceptance contract.
Runtime fault budgets stay bounded: a recoverable transport/FIFO transient alone
must not invalidate a good calibration; invalid data/partial commit remain hard failures.

## Timeout and navigation commands

Native defaults: compile/link 180 s without sanitizer, 600 s with sanitizer;
execution 180 s in every mode. `--build-timeout-s` and `--test-timeout-s` override
stages separately. Legacy `--timeout-s` sets both, with explicit stage options
winning regardless of argument order. No automatic retry or sanitizer fallback.
`check_all.py` forwards `--native-build-timeout-s`, `--native-test-timeout-s`;
legacy `--native-command-timeout-s` still sets both. Outer suite deadline remains
`--native-suite-timeout-s`; increasing an inner deadline does not bypass it.
Each effective stage deadline is recorded in native report metadata/commands.

After a successful native run, export its actual compiler arguments:

```sh
.venv-dev-linux/bin/python tools/export_native_compdb.py --report build/gate_runs/native-EXAMPLE/summary.json
```

Replace EXAMPLE with the real run. Output: `build/clangd-native/compile_commands.json`.
Point clangd at that directory (`--compile-commands-dir=/absolute/repo/build/clangd-native`).
This is navigation metadata, not a test cache or proof that current edits compiled.
It includes only compiled units, including host Arduino stubs; focused runs are
partial. Regenerate after flags/includes/profile changes. Never execute commands
from an uploaded report; exporter only copies argument arrays from local records.

For target navigation, PlatformIO supports a separate database, one profile at a time:

```powershell
& $TrackerPio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG -t compiledb
```

This writes root `compile_commands.json` (ignored), with Windows target paths.
Do not use it for Linux native files or mistake it for a linked firmware build.
Do not blindly allow clangd to query every executable from a third-party database.
[PlatformIO reference](https://docs.platformio.org/en/latest/integration/compile_commands.html).

## DEV-05 accuracy changes

- Math/quaternion changes: `test_dev05_math_reference`, `test_core_math_ahrs`.
- AHRS/mag correction changes: `test_dev05_algorithm_scenarios`, then compare
  identical-input reports. Known roadmap failures are explicit, never full PASS.
- Evidence export/comparison or scenario schema: `test_dev05_algorithm_accuracy`.
- Native fingerprint integration: add DEV-04/runner policies and one native run.
- Commands, metrics, scope and optional features: [DEV-05](dev05_algorithm_accuracy.md).

For DEV-05a observation metrics, run `test_dev05_observation_contracts` and
`test_dev05_algorithm_scenarios`. Review [measurement fixes](dev05a_report.md) and
the staged [math coverage plan](dev05_math_verification_plan_ru.md).
