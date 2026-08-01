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

The guided calibration command services FIFO, magnetometer runtime, Wi-Fi and SlimeVR internally while blocking the CLI. Temperature fitting now uses a dedicated setup temperature capture instead of the developer `test static` runner. Short contiguous windows use hard vibration/accel/FIFO gates, standard-error-of-the-mean quality, bounded thermal consistency and a robust leave-one-bin-out final fit; no-progress diagnostics stop an impossible capture instead of silently rejecting samples for the full timeout. Mag hard/soft apply uses robust full-ellipsoid quality gates: a deterministic Algorithm-R reservoir over the complete accepted capture, raw-norm prefilter, geometric outlier rejection, inlier ratio, fit-set box coverage, directional coverage, algebraic residual and corrected-norm residual checks. Full-capture and actual fit-set diagnostics are reported separately so discarded extrema cannot falsely satisfy a coverage gate. Magnetic axis inference prefers gyro-assisted motion scoring, cross-checks static accel-face/mag inclination samples when available, and keeps explicit `axis ...` tokens as an override/fallback.

`0023g_magnetometer_coverage_reservoir_hardening` repairs the failed guided magnetic run in which the first 160 dynamic intervals froze the axis dataset and more than ten thousand later intervals were discarded. The guided-axis collector now uses a fixed-memory reservoir stratified by dominant gyro axis and train/validation window parity, with deterministic replacement and a 75 ms admission cadence. Late rotations around a missing axis remain eligible without heap allocation, FIFO reconfiguration, or unbounded work. Independent-window and excitation metrics are maintained incrementally rather than rescanned in the blocking setup loop. The same patch fixes guided-flow ownership defects found during the audit: temperature-model readiness is re-evaluated after a new rest/gyro checkpoint, inconclusive static mag-face samples no longer suppress dynamic-axis collection, hard/soft collection and static axis-face observation have separate ownership so an already-valid hard/soft model is not needlessly reset or collected during accel faces, contradictory `nomag` plus axis options and trailing manual-axis tokens are rejected, and failure prompts point to the exact magnetic reason/fit metrics. Config schema 2 and candidate format 3 are unchanged.


## Calibration implementation notes

- Magnetometer calibration now uses a full ellipsoid fit and stores hard-iron plus a full 3x3 soft-iron matrix when coverage/residual quality gates pass.
- `setup calibration` remains transaction-safe: failed late calibration stages must not overwrite the last saved NVS calibration.
- `0023a` hardens setup/autonomy ownership: setup and all mutating calibration commands synchronously resolve provisional promotions, invalidate background evidence, and keep manual/setup candidates authoritative.
- `0023b` removes the cross-ABI accel-proposal stack regression, aggregates all quality-gate failures, and adds reboot coverage for accept/rollback cleanup boundaries.
- Initial rest-gyro uses a continuous train window plus held-out validation; each of the same six accel faces gets an automatic held-out tail; temperature capture validates statistically precise independent windows and temperature fitting uses copy-free leave-one-bin-out validation. The final verifier also checks that the calibrated input itself was stationary, not only that the output looked smooth.
- `setup verify` and the final setup stage validate the actual coherent quaternion/linear-acceleration output on any stable ordinary face. No diagonal or precise-angle pose was added.
- `cal erase_all confirm` completely removes saved calibration slots/candidate/autonomy journal/rejection state while preserving ordinary product policy.

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

`0023ga_axis_alignment_stack_hardening` fixes the concrete Windows/MSYS2 stack-policy regression reported after `0023g`: `setupRunAxisAlignment()` used 1056 bytes against a 1024-byte ceiling. The orchestration function no longer exposes dynamic/static solver results and the manual parsing buffer to one compiler frame. Explicit no-inline phase boundaries keep the dynamic solver, static cross-check/fallback, matrix application and manual prompt separate; Linux `-O2 -fstack-usage` drops the orchestrator from 880 to 144 bytes while preserving the exact `0023g` reservoir, train/validation and setup behavior. Config schema 2, candidate format 3, ODR, FIFO and network policy remain unchanged.


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

`0023gb_magnetometer_fit_metric_normalization` fixes the hardware-reproduced hard/soft rejection that remained after reservoir coverage succeeded: `algebraic_residual_too_high` was computed from the raw equation `x^T A x + b^T x = 1`, so its magnitude changed with hard-iron translation through `k = 1 + c^T A c`. The residual is now divided by `abs(k)` and describes the centered dimensionless ellipsoid equation. The existing threshold is not relaxed. Finite rejected fits retain and report algebraic/geometric residuals, limits, coverage and inlier metrics. Translation-invariance, mild-disturbance acceptance and strong non-ellipsoidal rejection regressions preserve physical quality gates. Config schema 2, candidate format 3, FIFO, ODR, AHRS and networking are unchanged.

`0023gc_mag_fit_stack_and_profile_build_hardening` fixes two concrete regressions reported by Windows/MSYS2 and mandatory PlatformIO builds after `0023gb`. `MagCalibrationCollector::compute()` kept two 9x9-double `FitAccumulator` objects live at once and measured 2240 bytes against the 2048-byte policy ceiling on MSYS2. The raw and inlier passes now reuse one accumulator workspace while preserving the same initial fit, robust inlier selection, optional refit and quality gates; Linux `-O2 -fstack-usage` drops the function from 2016 to 1296 bytes and a stricter 1792-byte ceiling preserves cross-ABI margin. Compact Production/Production-Diag/Slim hooks also called `magStatusPrintCalibrationFitQuality` while its definition was only included under `TRACKER_ENABLE_DETAILED_MAG_STATUS`, causing mandatory profile compilation to fail. The helper now lives in a minimal always-available header shared by compact hooks and the detailed reporter, without restoring the excluded `mag_status_reporter.cpp` source. Calibration math, normalized residual thresholds, config schema 2, candidate format 3, FIFO, ODR, AHRS and networking are unchanged.

## 0023gd magnetometer math and alignment hardening

`0023gd_magnetometer_math_and_alignment_hardening` performs the full mathematical audit requested after hardware retained complete three-axis reservoir coverage but hard/soft calibration still stopped at `ellipsoid_fit_failed`. The ellipsoid solver now uses affine-centered and per-axis-scaled coordinates, equilibrated normal equations, explicit conditioning/stage diagnostics, translation-independent filtering and double-precision mapping back to raw coordinates. The physical inlier, geometric residual, directional coverage, axis-ratio and algebraic gates remain fail-closed.

The same audit removes independent `magToImu` non-convergence defects: exact unique-window accounting, dataset-owned axis/partition coverage, bounded runtime reservoir replacement, hard/soft-corrected direction admission, raw-origin acceptance after calibration, shared trapezoidal endpoint interval construction, exact gyro endpoint retention at each magnetic callback, chronological FIFO raw/mag dispatch, QMC6309 near-rail saturation detection, a right-handed driver contract and a fail-closed static/dynamic coarse-mapping cross-check. Exhaustive tests cover all 24 proper signed-permutation mountings; reflections remain forbidden persistent mappings.

Config schema 2, candidate format 3, SlimeVR protocol 22, IMU ODR, FIFO configuration, AHRS product policy, networking and NVS migration remain unchanged.

## 0023ge magnetometer post-audit realtime hardening

The acceptance audit of 0023gd found that an already-due sensor-hub backlog could
record a chronological deferral and still advance the raw timeline, and that up
to eight mag callbacks could bypass the cooperative callback-time budget.
0023ge gates raw advancement on complete due-mag dispatch, applies the same
slice budget to magnetic callbacks, reports count-vs-time deferrals, reuses the
single sensor-to-device validation decision for the gyro endpoint, and preserves
normalization diagnostics for early physical fit rejection. Calibration math,
thresholds, schema 2, candidate format 3, protocol 22, ODR and hardware FIFO
configuration remain unchanged.

## 0023gf magnetometer callback cross-ABI hardening

Windows/MSYS2 measured `MagRuntimeController::processRawSample()` at 1040 bytes against the 1024-byte ceiling although Linux measured 944 bytes. `0023gf` separates endpoint, heading, field-reliability and yaw phases behind explicit no-inline boundaries; the orchestrator drops to 336 bytes on Linux `-O2` with independent conservative ceilings for every phase. One coherent `millis()` value and one runtime config are now used per mag sample. The unchecked inverse helper added to the common frame header by 0023ge is removed and kept controller-local. Calibration math, FIFO scheduling, schemas, ODR, AHRS, networking and NVS semantics are unchanged.

## 0023gg magnetic timestamp and setup acceptance hardening

The first complete hardware guided-calibration run after 0023gf accepted hard/soft iron but exposed 4255 gyro/mag skew rejections and a final verification false negative at 207 healthy input samples. 0023gg anchors every sensor-hub magnetic frame to the current IMU/FIFO time domain instead of free-running forever at nominal 60 Hz. If independent partitions recover the same proper coarse axis/sign mapping but disagree only on small continuous refinement, the disputed refinement is discarded and the unchanged physical quality gates evaluate the shared coarse rotation. Final setup verification keeps its 256-input-sample requirement but extends its capture from a minimum four seconds to a bounded maximum eight seconds and reports every stationarity sub-gate separately.

## 0023gh SlimeVR Wi-Fi provisioning compatibility hardening

`0023gh_slimevr_wifi_provisioning_compat_hardening` corrects the serial compatibility contract used by SlimeVR Server's setup wizard. The reported field is upstream `WiFiReconnectionStatus`, not Arduino `WiFi.status()`: ordinary saved-credential `Connecting` reports `SavedAttempt=1`, a successful serial credential submission latches `ServerCredAttempt=3` for its reconnect, `Backoff` reports `Failed=4`, and `Connected` remains `Success=5`. `SET WIFI` and `SET BWIFI` emit the exact upstream-compatible success strings. Credential persistence, immediate non-blocking reconnect, normal Wi-Fi state-machine behavior, UDP tracking, schemas and queues are unchanged.

## 0023gi SlimeVR Connect Trackers handshake and build-date hardening

`0023gi_slimevr_connect_trackers_handshake_and_build_date_hardening` completes the compatibility audit after 0023gh allowed the setup wizard to observe `Success=5` but the tracker still timed out before appearing in the server. Healthy serial tracker status is now `0` and no longer changes to `1/2` for Wi-Fi/server stages. Every discovery handshake uses the official fixed packet number `0` and does not consume the established-session packet sequence, so a lost first broadcast cannot make every later retry protocol-incompatible. The UDP handshake and serial compatibility output now report `<feature-version>+build.YYYYMMDD`; UTC build date and actual Git/worktree identity are exposed through `GET INFO`/`version`. `SOURCE_DATE_EPOCH` preserves reproducible builds. Wi-Fi credential persistence, UDP port/broadcast address, discovery cadence, tracking packets, schemas and queues are unchanged.


## 0023gj SlimeVR Connect Trackers session-restart hardening

`0023gj_slimevr_connect_trackers_session_restart_hardening` fixes the remaining already-connected onboarding case left by 0023gi. Successful server-style credential provisioning is now transactional and explicitly restarts the SlimeVR UDP session after the committed Wi-Fi config becomes live. This guarantees a fresh packet-number-zero discovery and `SensorInfo` registration even when identical credentials reconnect too quickly for the runtime to observe Wi-Fi loss. A shared typed helper now owns normal start versus forced restart semantics, removing recursive CLI dispatch and preventing command-path drift. Tracking, FIFO, AHRS, protocol 22, storage schema, calibration and normal `slime start` behavior are unchanged.


## 0023gk magnetometer robust-fit acceptance hardening

`0023gk_magnetometer_robust_fit_acceptance_hardening` fixes two hardware-reproduced hard/soft-iron acceptance defects. The centered algebraic residual is approximately twice normalized radial error, so its effective numerical ceiling is now guaranteed not to be stricter than the authoritative geometric gate; the second tracker log (`0.146809` algebraic, `0.074308` geometric) therefore no longer fails a contradictory duplicate threshold. Robust inlier selection now caps contaminated sigma at 1.5 times the geometric limit and performs at most three exact-membership-convergent refits using the existing single accumulator workspace. A 15% moderate-disturbance fixture is recovered, while 20% exceeds the unchanged 82% inlier requirement and remains fail-closed. New diagnostics expose robust refit passes and threshold factor. Config schema 2, candidate format 3, ODR, FIFO, AHRS, magToImu, networking and calibration ownership are unchanged.


## 0023gl SlimeVR UDP TX recovery hardening

`0023gl_slimevr_udp_tx_recovery_hardening` fixes the long-lived low-TPS cascade reproduced on two ESP32-C3 trackers with strong RSSI and an otherwise live server session. ESP-IDF/lwIP TX-buffer pressure is now reported through errno-aware transport results, stale motion traffic enters a bounded 20-160 ms backoff, and recovery triggers on either four consecutive failures or eight failures in the exact last 32 physical attempts. Recent heartbeat/ping now selects a session-preserving local UDP rebind rather than suppressing recovery; a failed rebind, stale RX, or a second burst within two seconds escalates to the existing full discovery lifecycle. New counters expose backoff, TX-pressure classification, local rebind and escalation. FIFO, ODR, AHRS, packet formats, protocol 22, calibration, persistent schema and candidate format are unchanged.

## pre-0024 hotpath headroom foundation

`pre-0024_hotpath_headroom_foundation` is a standalone performance wave before AHRS `0024`. It makes hardware SPI drain and raw/magnetic callbacks share one absolute FIFO slice budget, reserves an armed pose deadline, replaces full nested network service with a rotation-only tick, spreads the unchanged battery trimmed-mean estimator across bounded ADC services, caches slow Wi-Fi diagnostics, reduces remote-console/LED idle taxes, samples only expensive motion aggregates, and moves runtime-bias/autonomy window finalization outside the 960 Hz callback while preserving exact evidence and update ordering. Fixed-memory DIAG telemetry now reports section percentiles, 10 ms frame headroom, profiler overhead and honest software-only queue/prepared/rotation ages. No raw samples are dropped or reordered; ODR, AHRS math, accel/mag correction cadence, output target rate, protocol 22, calibration models and persistent schemas are unchanged.

## pre-0024a tracking deadline hardening

`pre-0024a_tracking_deadline_hardening` follows the telemetry foundation after a clean one-tracker hardware log still showed zero frame-headroom p05, 79% late rotation events, 8.11% wall time in repeated nested network calls, 14.365 ms FIFO callback bursts and 86,427 magnetic budget deferrals. It adds an O(1) due gate before the rotation-only nested UDP path, checks the absolute FIFO slice budget before a fifth raw callback, activates existing urgent service at 40 ms queue timestamp span, and moves only background axis-alignment observation into a bounded deferred evidence FIFO. Evidence preserves exact ordering and config epoch; overflow or stale epochs drop only uncommitted background evidence and are observable. ODR, sample order, AHRS equations, current magnetic yaw correction, output rate, packet formats, active calibration and persistent schemas are unchanged.

## pre-0024ab hotpath transform-cache hardening

`pre_0024ab_hotpath_transform_cache_hardening` follows hardware logs from `pre-0024aa` where clean UDP delivery and zero FIFO overruns still coexisted with 150-370 ms live queue age, a 512-sample high-water mark and roughly 0.69-0.72 ms sampled raw-callback cost at a 1.054 ms IMU period. The patch removes repeated validation of the immutable sensor-to-device rotation from every IMU and magnetic sample, caches the complete immutable magnetic runtime configuration by tracker-config CRC, and extends fixed-memory age histograms so overload percentiles remain readable above 100 ms. Changed config revisions are still revalidated fail-closed, direct callers retain the old validation path, and no tracking equation, cadence or packet payload changes are made.

## pre-0024ac IMU hotpath and slack-admission hardening

`pre-0024ac_imu_hotpath_and_slack_admission_hardening` follows hardware evidence from `pre-0024ab`: the persistent 150-550 ms backlog was removed, but each IMU callback still averaged roughly 0.66 ms and optional hooks could collide with short 47-54 ms queue bursts. The patch evaluates gyro temperature compensation/current bias once per sample and reuses it for calibration, quality and runtime-bias evidence; avoids the disabled-stream `micros()` read; adds fixed-memory 1/64 sampled IMU-stage telemetry; and admits battery, LED, magnetic deferred work, autonomous calibration and remote console only when FIFO and rotation-deadline slack allow it. Only one completed background worker is admitted per loop. Sensor order, AHRS/correction cadence, packet rates and persistent formats are unchanged.


## pre-0024ad network-pressure pacing and recovery hardening

`pre_0024ad_network_pressure_pacing_and_recovery_hardening` addresses a five-hour 13-tracker session that accumulated 28,883 ESP/lwIP TX-pressure failures, 1,261 local UDP rebinds and 815 full session reopens despite zero Wi-Fi disconnects and zero FIFO overrun/full events. Transient pressure now uses bounded 10-80 ms pacing and successful-motion age instead of failure-density recovery: intermittent successes preserve the socket/session, 500 ms without motion can request one local rebind, another 1000 ms without post-rebind success can request full discovery, and 5/10 s cooldowns suppress churn. During capability renegotiation packet-4 acceleration is withheld so reconnect cannot create a 150-datagram/s motion fallback. The unchanged 100 Hz schedule receives a deterministic MAC-derived phase. ODR, FIFO order, hardware timestamps, AHRS, correction cadence, packet layouts, calibration and persistent schemas are unchanged.
