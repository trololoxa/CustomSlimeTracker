# Project status

This document replaces the completed code-quality roadmap notes. It records the current structural state of the firmware after the architecture cleanup patches.


## Source of truth

Use Git for the exact firmware identity:

```bash
git branch --show-current
git rev-parse --short HEAD
git status --short
```

`docs/current_implementation.md` is the canonical short runtime summary.
`docs/production_firmware_roadmap.md` is retained as a historical planning
record and must not be used to infer that an item is still missing or already
implemented.

## Current structural baseline

- `main.cpp` is an Arduino entrypoint only.
- `app/` owns setup/loop orchestration and app-level singleton wiring.
- `app/hooks/` contains include-only glue sections; hooks must stay wiring-only.
- `runtime/` owns FIFO processing, sample pipeline, static tests, machine logs, output snapshots, runtime bias, mag runtime, and tracking state.
- `sensor/` owns math/model code for AHRS, calibration, quality, magnetometer heading, and yaw correction.
- `serial/` command domains are split into `.hpp/.cpp` pairs; parser glue remains fixed-buffer/no-heap.
- `config/` owns persisted schema, runtime apply/capture/sanitize, NVS store, and printing.
- `connection/` owns low-level hardware/protocol drivers.
- `src/network/` owns real Wi-Fi station management and UDP transport; SlimeVR packet formatting/output runtime lives outside low-level network primitives.

## Deliberate developer conveniences

- The boot serial settle delay is Debug-profile only (`TRACKER_ENABLE_BOOT_DELAY`). Production and Slim do not keep the old unconditional `sleep(2)`.
- `defines.h` is now a compatibility umbrella over `src/build_config/*`; new profile/config defaults should go into the focused build-config headers.
- `BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG` is the committed default build/upload environment on the `Upgrades` branch. It is a service environment built from `TRACKER_PROFILE_PRODUCTION` plus the live runtime profiler; Debug, Production and Slim remain separate committed environments.

## Current quality gate

Run host tests and all profile PlatformIO builds:

```bash
python tools/check_all.py --clean --require-pio
```

If PlatformIO is not on `PATH`, pass `--pio-bin` or set `PIO`.

Host-only check:

```bash
python tools/check_all.py --clean --skip-pio
```

## Remaining structural work

1. Keep `serial/tracker_serial_context.hpp` lightweight by moving reusable parse/print helpers to focused headers.
2. Maintain `docs/cli_reference.md` when commands or side effects change.
3. Maintain `docs/config_schema.md` when persisted schema changes.
4. Maintain replay/log tooling and add fixtures before major tracking-filter changes.
5. Keep SlimeVR UDP output decoupled from AHRS/FIFO: it must consume prepared output snapshots only, never low-level sensor state directly.

## Current SlimeVR network baseline

The tracker now has a working SlimeVR UDP MVP:

- non-blocking Wi-Fi station manager;
- ESP32-C3 TX power workaround through `TRACKER_WIFI_TX_POWER`;
- UDP discovery and server reconnect;
- SlimeVR protocol v22 metadata with coherent rotation/linear-acceleration output;
- `SensorInfo`, `RotationData`, heartbeat, ping/pong, RSSI and temperature telemetry;
- LSM6DSV embedded physical tap runtime on INT1 with firmware-side 2..10 tap aggregation, `FUNCTIONS_ENABLE.INTERRUPTS_ENABLE` gating, masked register verification, SlimeVR Tap packet output and serial `tap ...` diagnostics;
- non-blocking GPIO status LED runtime for ESP32-C3 SuperMini (`TRACKER_STATUS_LED_PIN=8`, active-low by default), SlimeVR-style status/error blink patterns and serial `led ...` diagnostics;
- incoming `SetConfigFlag` handling with transactional mag/yaw persistence and acknowledgement;
- local serial output decoupled from SlimeVR UDP;
- `slime status` compact view and `slime debug` full counter dump;
- `test runtime <seconds>` for full Wi-Fi/server/FIFO loop-load measurement.

Optimization work should keep using a `test runtime 600` baseline rather than `test static` alone, and firmware-size deltas should be checked with `python tools/report_firmware_size.py`.


## Known baseline gaps before the upgrade series

The following limitations are confirmed in the current code and are deliberately
recorded before behavior-changing patches begin:

- persisted `sensorToDevice` is validated as a proper rotation, applied to
  calibrated gyro, accel and IMU-aligned magnetometer data; mounting/body offsets
  remain server-side;
- gyro/temperature clear and replacement have explicit model invalidation
  semantics, and the guided temperature capture now accepts only validated
  contiguous stationary windows while preserving earlier progress across brief
  touches or quality faults;
- recovery is reason-aware: bounded FIFO full/overrun uses non-blocking soft
  recovery and resumes output on the next integrated sample, while corrupted
  timestamps, explicit resets and blocking operations still block stale output,
  keep post-gap gyro active and perform heading-preserving tilt reacquisition;
- runtime FIFO processing is cooperative across app-loop passes, preserving all
  samples while bounding one pass so the 100 Hz SlimeVR scheduler can run;
- prepared output contains one timestamp-coherent quaternion and device-frame
  linear-acceleration snapshot; negotiated packet-100 output bundles float32
  packet 17 followed by float32 packet 4 into one datagram, while older servers
  receive packet 17 at pose rate and coherent packet 4 at a 50 Hz fallback rate;
- SlimeVR protocol 22 advertises the tested corrected acceleration contract:
  rotation and acceleration share `+X right, +Y forward, +Z top/outward`, with no
  server legacy acceleration-only local-Z correction;
- firmware FeatureFlags and packet-100 bundle negotiation are implemented;
  packet 23 remains an explicitly disabled experimental option. Negotiation and
  control traffic are bound to the selected server endpoint, while malformed
  empty FeatureFlags cannot lock the runtime into a false capability state.
  SensorInfo uses explicit dirty/waiting/acknowledged state, UserAction packet 21
  is available, and protocol switching is intentionally unsupported rather than
  partially applied;
- all ESP32-C3 profiles use one committed 4 MiB no-OTA layout with a 3 MiB
  factory app; the previous Arduino default 1.25 MiB OTA slot could produce a
  boot loop once the motion-bundle image crossed its real bootable boundary;
- SignalStrength packet 19 now preserves signed RSSI dBm instead of the previous
  incorrect 0-100 normalization.

These gaps define the next implementation patches. They are not reasons to
weaken the existing FIFO timestamp, calibration, AHRS or mag-yaw quality path.

## Patch 0020 session completeness

Patch 0020 completes the firmware-side SlimeVR session contract without changing
the motion-frame or packet-23 policy:

- special six-byte SensorInfo acknowledgement parsing and resend-until-ACK state;
- explicit firmware/server FeatureFlags negotiation and bundle gating;
- strict endpoint validation and liveness updates only from validated packets;
- transactional SetConfigFlag apply/persist/ACK with idempotent retries;
- UserAction packet 21 plus optional persistent physical-tap mapping, default off;
- malformed/unknown/control diagnostics;
- boot-time QMC start from the FIFO configuration already established by bootstrap,
  avoiding the redundant successful-path FIFO reconfigure/recovery.

Packet 23 remains disabled and TX-stage instrumentation remains outside this patch.



## Patch 0020a boot FIFO epoch hardening

Patch 0020a is a focused hotfix for a hardware-only startup regression found
after 0020 acceptance. Bootstrap configured and started the 960 Hz FIFO before
blocking QMC6309 sensor-hub initialization. Although the duplicate mag/FIFO
reconfiguration was already removed in 0020, the hardware FIFO could still fill
before the first runtime loop.

The runtime startup sequence now keeps INT1 detached, pauses FIFO collection
without destroying watermark/BDR configuration, completes QMC setup, performs
one final FIFO reset/timestamp/queue/quality reset, and attaches INT1 last. A
failure in this finalization enters the existing non-blocking sensor startup
recovery instead of declaring the runtime ready. Expected cold-boot diagnostics:

```text
fifo_overrun_delta=0
fifo_full_delta=0
tracking_recovery_bootstrap_bypass_delta=0
tracking_recovery_reconfigure_delta=0
```

## Pre-0020 and calibration-storage hardening

The pre-0020 audit closed transactional runtime/config defects while retaining the
legacy single blob. Patch 0021 and its corrective hotfixes then replaced that blob
with the dual-slot/candidate framework:

- config and network NVS saves now return the exact sanitized bytes to the live mirror only after a successful write; failed writes leave active state unchanged;
- CLI commands with an explicit `save` use candidate/save/commit ordering, while commands without `save` remain intentional RAM-only changes;
- core calibration/config persistence uses dual active slots with generation, read-back verification, a CRC-protected selector and storage-v2 per-slot commit markers;
- valid legacy `tracker/cfg` data migrates automatically without deleting the old blob before the new selected slot verifies;
- calibration candidates have separate storage, quality/coverage/provenance metadata, sensor signatures, calibration-revision freshness gates, wear gates and two-phase runtime/selector/commit-marker promotion;
- candidate promotion composes calibration-owned fields onto the current active config, preserves measured quality, and does not restart IMU/FIFO hardware;
- aborted prepared slots are restored from the previous active record, and newer uncommitted slots cannot become fallback;
- boot distinguishes an empty store from degraded NVS/read/allocation errors instead of silently presenting both as a clean defaults boot;
- genuinely empty NVS is distinguished from present-but-corrupt active storage; transient boot read errors latch active/candidate writes until the authoritative config has also been applied successfully to hardware/runtime;
- no-op config saves do not write flash, increment generation or invalidate an unrelated calibration candidate; legacy v1 slots/candidates remain readable and upgrade without NVS erase;
- `config load` and `config defaults` apply LSM6DSV/FIFO hardware transactionally and roll back on reconfiguration failure;
- temperature-model replacement/reset also resets the volatile residual gyro trim, and the final persisted slope is bounded at every entry point, including boot-time config application;
- FIFO startup fallback timestamps anchor to the real drain time, and completed-sample queue loss has its own counter and recovery reason instead of being misreported as waiting-for-timestamp overflow;
- magnetic sample ages and yaw integration are wrap-safe across the 32-bit `millis()` rollover;
- native-test extra compiler flags are also passed to the linker, enabling ASan/UBSan audit runs.

## Current setup baseline

The firmware now has a compact user-facing `setup` layer for first-run preparation. It no longer exposes manual `setup rest/accel/mag/axis/temp` wrappers; those jobs belong either to the full guided setup flow or to the lower-level service commands.

Current setup coverage:

- `setup guide` prints the recommended first-run sequence.
- `setup status` reports production, 6DoF, mag-yaw, temperature-model, Wi-Fi, and SlimeVR readiness plus next commands.
- `setup wifi` is an interactive serial Wi-Fi provisioner: scan visible networks, choose one by number, enter password, connect, save successful credentials to NVS, start SlimeVR discovery, and leave Wi-Fi/SlimeVR autostart enabled. If Wi-Fi succeeds but the server is not found in the setup timeout, Wi-Fi remains saved and discovery continues in normal runtime.
- `setup calibration [resume|full] [nomag|6dof] [axis <bodyX> <bodyY> <bodyZ>]` runs the guided production flow: rest/gyro, Wi-Fi heat warm-up, validated stationary-window temperature capture, accel 6-position calibration, sensor-to-device case-frame alignment, optional magnetometer hard/soft and mag-axis stages, and production feature enable/save. Full mode reuses the first two accel captures as top/+Z and forward/+Y frame observations, so it still requires only six accel positions. Resume mode captures only those two positions when frame data alone is missing. Failed/aborted stages restore the previous RAM calibration/config and do not overwrite the last saved NVS state.
- SlimeVR `SensorInfo.hasCompletedRestCalibration` follows local gyro/rest validity instead of being hardcoded true.

The guided calibration command services FIFO, magnetometer runtime, Wi-Fi and SlimeVR internally while blocking the CLI. Temperature fitting now uses a dedicated setup temperature capture instead of the developer `test static` runner, while reusing the same fit quality gates. Mag hard/soft apply now uses robust full-ellipsoid quality gates: bounded sample reservoir, raw-norm prefilter, geometric outlier rejection, inlier ratio, box coverage, directional coverage, algebraic residual and corrected-norm residual checks. The final setup save captures runtime calibration/config to NVS after production features are enabled. Magnetic axis inference now prefers gyro-assisted motion scoring from the mag motion stage, cross-checks against static accel-face/mag inclination samples when available, and keeps explicit `axis ...` tokens as an override/fallback.


## Calibration implementation notes

- Magnetometer calibration now uses a full ellipsoid fit and stores hard-iron plus a full 3x3 soft-iron matrix when coverage/residual quality gates pass.
- `setup calibration` remains transaction-safe: failed late calibration stages must not overwrite the last saved NVS calibration.

### RC1 serial provisioning compatibility


Battery ADC runtime reads the RC1 divider `BAT+ -> R_TOP -> GPIO -> R_BOTTOM -> GND` with defaults `GPIO4`, `R_TOP=180 kΩ`, and `R_BOTTOM=180 kΩ`. GPIO4 is ESP32-C3 ADC1_CH4, the supported ADC path for battery telemetry. The divider is high impedance and has no hardware capacitor, so the runtime samples sparsely (`TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS`, default 10000 ms in Debug and 30000 ms in Production), takes a larger ADC burst (`TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT`, default 64), discards the first settle reads (`TRACKER_BATTERY_ADC_DISCARD_COUNT`, default 4), sorts the remaining burst, averages the trimmed center, maps 3.30-4.20 V to 0-100%, applies a slow EMA (`TRACKER_BATTERY_ADC_EMA_ALPHA`, default 0.12), and rejects impossible voltage steps. It reports safe 0.000 V / 0.0% when the divider is absent, below the present threshold, invalid, or unreadable. It also rejects BAT+ values above `TRACKER_BATTERY_PRESENT_MAX_VOLTAGE` (default 4.35 V) and preserves the previous filtered estimate on one-off low/high ADC glitches. CLI/status percentages are 0-100%, while SlimeVR BatteryLevel telemetry is converted to the protocol's 0.0-1.0 fraction at send time. The value is exposed through `battery status`, `GET INFO`, `slime status`, and periodic SlimeVR BatteryLevel telemetry.

The firmware now exposes SlimeVR Server serial-compatibility commands for initial provisioning: `SET WIFI`, `SET BWIFI`, `GET INFO`, `GET CONFIG`, `GET TEST`, `GET WIFISCAN`, `REBOOT`, `FRST`, `DELCAL`, and temperature-only `TCAL`. `SET WIFI`/`SET BWIFI` save credentials into the separate network NVS config, enable Wi-Fi/discovery, and restart SlimeVR discovery. Blocking Wi-Fi scans suppress expected FIFO-recovery console noise for a short grace window without disabling recovery, counters, or machine-log events. They are still tracking interruptions: gyro motion during the scan is not reconstructable, but FIFO/AHRS recovery must resume integration afterwards. `ahrs status` exposes `large_dt_rebase_count`, `fifo_rebase_count`, `last_rebase_t_us`, and `post_fifo_recovery_samples` for post-scan diagnostics. FIFO timestamp resets also clear the magnetometer sensor-hub timestamp baseline so mag samples restart from the new IMU stream. `TCAL SAVE` is intentionally scoped to gyro temperature compensation so host-side temperature-calibration commands cannot accidentally capture unrelated runtime output/accel state. `TCAL RESET` changes only the in-RAM temperature model, resets the RAM-only residual gyro trim learned against the previous model, and does not mutate the persistent config object unless a later explicit `TCAL SAVE` is requested.

- 0021d separates configuration persistence, calibration snapshots and calibration events; keeps no-op saves/provenance stable; scopes component saves; uses v3 model-only candidate freshness with v1/v2 compatibility; guards unsaved runtime calibration; resets dependent adaptive state on load/promotion/clear; and refreshes SensorInfo when advertised calibration or sensor capability changes.


## 0021e calibration epoch and field-ownership hardening

0021e finishes the patch-21 integration audit without changing the persisted blob
layout. Calibration models, their evidence, runtime policy and unfinished capture
workspaces now have explicit owners and epoch boundaries:

- invalid gyro, temperature, accel, magnetic and frame models sanitize to canonical
  fail-honest values and clear only their own evidence; independent enable policy is
  preserved;
- a disabled `sensorToDevice` frame is logically identity and dormant matrix bytes no
  longer create false sensor-signature mismatches;
- candidate promotion is rejected when the live unsaved ODR/full-scale/FIFO/frame
  contract differs from the active signature, because calibration-only promotion does
  not reconfigure hardware;
- full config apply, promotion, setup commit/rollback and full calibration erase clear
  incompatible accel, gyro-temperature and magnetic collection workspaces;
- real gyro/temperature correction changes start a new AHRS/runtime-bias epoch, while
  no-op toggles do not disturb tracking;
- guided accel+frame calibration keeps acceleration unavailable until both new models
  are valid, and guided production policy is applied to live AHRS/quality state before
  readiness and persistence;
- manual hard/soft-iron replacement invalidates the previous mag-to-IMU alignment and
  magnetic-yaw apply state; candidate promotion remains atomic because it transfers the
  field and axis models together;
- `cal clear_all` and `ERASE CALIBRATION` also clear the device frame, and persistent
  erase discards any staged candidate that could otherwise resurrect the old model.

Compact and detailed status now report `sensor_to_device_valid` and
`motion_frame_config_ready` separately from `accel_cal_valid`.


## Patch 0022 continuous magnetic alignment and heading reliability

Patch 0022 closes the planned magnetic reliability wave without changing the
persisted config schema, IMU/FIFO baseline, SlimeVR protocol, or active calibration
automatically:

- temporal norm/dip/heading consistency with explicit acquiring/trusted/suspect/disturbed/recovering states;
- long-dwell new-environment recovery and bounded reference adaptation;
- slow large-innovation yaw reacquisition without permanent cooldown lockout;
- continuous gyro-assisted mag-axis candidate collection with timestamp coherence,
  independent-window/axis coverage and only 24 proper rotations;
- rate-limited solve/storage checks so the 60 Hz mag path does not repeatedly run
  expensive solves or NVS inspection;
- background results stage only through the 0021 candidate lifecycle and never
  flush, promote, reconfigure FIFO or mutate active calibration automatically;
- LOGVER 3 and expanded CLI/machine diagnostics for field state, re-entry,
  reacquisition and candidate progress.

The next planned main stage is 0023 safe background calibration autonomy.

## Hotfix 0022a magnetic candidate, reacquisition and realtime hardening

The post-0022 integration review found that measured axis quality was not
comparable to the older binary alignment-valid score, instantaneous 60 Hz heading
rate could keep reacquisition closed forever, a stable shifted field could become
trusted against the old heading reference, and candidate storage inspection still
ran from the magnetic sample path. 0022a fixes those defects and adds a two-stage
axis solver:

- 24 proper signed permutations provide the coarse axis/sign mapping;
- a bounded local `SO(3)` refinement represents real residual mechanical rotation
  without introducing scale, shear or reflection;
- finite interval prediction uses `Exp(-omega*dt)` instead of a first-order
  derivative approximation;
- the active matrix is scored on the same intervals and only a proven improvement
  receives measured candidate quality;
- noisy large-error reacquisition uses filtered signed heading rate;
- changed magnetic environments remain fail-closed until the old field returns or
  the reference is explicitly reacquired;
- solve and lightweight candidate-slot/storage work are deferred until FIFO is
  neither pending nor urgent, with explicit timing/deferral counters.

0022a keeps config/candidate formats unchanged and does not add automatic flush,
promotion, rollback or candidate cleanup. Those autonomous lifecycle operations
remain the scope of 0023 after hardware acceptance of this hotfix.

## Hotfix 0022b magnetic environment and solver confidence hardening

The post-0022a quality audit found three remaining integration defects: moderate
5-19 degree stationary field jumps could clear the one-sample suspect gate and
pull tracker yaw, fixed-degree solver separation rejected correct ordinary-speed
motion, and the same observations were used to fit and prove a candidate. 0022b:

- latches abrupt cumulative stationary heading discontinuities above the normal
  bounded yaw-correction rate and remains fail-closed until the original field
  returns or reference acquisition is explicitly restarted;
- splits independent temporal windows into training and validation partitions;
- normalizes winner separation by observable magnetic angular motion, allowing
  valid 30-120 deg/s datasets without weakening ambiguity rejection;
- compares the refined winner and active alignment only on held-out evidence and
  rejects training-only fits or mismatched validation winners;
- permits axis-independent finite/in-range observation collection so a
  bad-but-valid active mapping can be repaired;
- gates deferred solve/storage work on both software state, actual LSM FIFO status
  and SlimeVR rotation-deadline slack.

0022b retains the same persistent config and candidate formats. Automatic flush,
promotion, probation, rollback and cleanup remain the scope of 0023.

## Hotfix 0022c promotion stack hardening

Windows/MSYS2 GCC reported a 1040-byte stack frame for
`TrackerConfigStore::prepareCandidatePromotion`, exceeding the 1024-byte storage
policy even though Linux/GCC measured 1008 bytes. The cause was an ABI-dependent
756-byte `TrackerConfig` return temporary from `trackerComposeCalibrationCandidate`.
0022c composes the calibration candidate directly into the heap-backed promotion
workspace and adds a source policy plus a 768-byte promotion-specific ceiling.
Persistent schemas, candidate semantics, magnetic behavior, FIFO, network, and
tracking math are unchanged.
