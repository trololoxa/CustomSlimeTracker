# Runtime hot-path optimization notes

This document tracks the safe runtime optimizations that do not change IMU ODR,
LSM6DSV performance mode, AHRS math, calibration application or SlimeVR
RotationData quaternion quality.

## SlimeVR runtime configuration policy

`SlimeVROutputRuntime::configure()` is a static-configuration operation. It may
reset UDP/session state when endpoint, device identity, magnetometer support,
telemetry policy or output rate changes. It must not be called on every main
loop iteration.

The app layer separates:

- static SlimeVR config refresh: cached and profile-limited;
- live runtime state update: temperature, battery and rest-calibration status;
- runtime update: Wi-Fi/UDP state machine and packet emission.

This avoids repeated config rebuilds and potential state churn in the hot path.

## Telemetry intervals

Telemetry is split by packet type:

| Macro | Purpose |
|---|---|
| `TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS` | RSSI/signal packet cadence |
| `TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS` | IMU temperature packet cadence |
| `TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS` | Battery packet cadence |
| `TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS` | Legacy/common fallback |

Default profile policy:

| Profile | Signal | Temperature | Battery |
|---|---:|---:|---:|
| Debug | 5 s | 5 s | 5 s |
| Production | 15 s | 15 s | 30 s |
| Slim | off | off | off |

This only changes service telemetry cadence. It does not alter quaternion
calculation, FIFO processing or AHRS behavior.

## Live SlimeVR state refresh

Live non-tracking state refresh is throttled with
`TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS`:

| Profile | Default |
|---|---:|
| Debug | 0 ms |
| Production | 1000 ms |
| Slim | 5000 ms |

At worst this delays non-tracking metadata such as battery percentage or the
SensorInfo rest-calibration bit by the configured interval.

## Work and idle-candidate metrics

Debug runtime tests collect coarse work counters:

```text
work_loop_count
idle_candidate_loop_count
idle_candidate_ratio
fifo_work_count
battery_work_count
network_work_count
tap_work_count
led_work_count
heartbeat_work_count
```

These counters are measurement-only unless an explicit idle/yield experiment is
enabled. Work counters must count real work, not merely that a hook was called:

- `network_work_count` increments only when Wi-Fi/SlimeVR state machines run
  after their profile update budget.
- `heartbeat_work_count` increments only when a boot heartbeat line is printed.
- `battery_work_count` increments when a battery sample/update actually occurs.

## Hot-path perf counters

`TRACKER_HAS_HOTPATH_PERF` is the single contract for expensive per-sample and
per-FIFO timing instrumentation. It is enabled when a profile needs hot-path
measurements, static tests or runtime tests.

| Profile | `TRACKER_HAS_HOTPATH_PERF` | Reason |
|---|---:|---|
| Debug | on | runtime/static tests and performance diagnostics |
| Production | off | no test runners; avoid per-sample `micros()` timing |
| Slim | off | smallest/lowest-overhead runtime path |

When this contract is off, the firmware does not call `micros()` just to measure
sample processing or FIFO batch duration. Timestamping required for correctness,
such as FIFO drain timestamps or non-blocking FIFO fallback polling, remains
enabled.


## SlimeVR rotation/service split

The SlimeVR runtime now separates fast RotationData scheduling from slower
service-path work. `TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS` controls how
often the app asks the network runtime to run at all. Inside the SlimeVR
runtime, `TRACKER_SLIMEVR_SERVICE_UPDATE_INTERVAL_MS` throttles slower work:

- incoming packet polling;
- heartbeat / SensorInfo refresh;
- telemetry packets;
- discovery while the server is not found;
- server-silence checks.

RotationData remains checked on the normal network runtime cadence, so this
change does not reduce FIFO/AHRS update rate or intentionally lower the
quaternion send scheduler. Default service intervals are:

| Profile | Service interval |
|---|---:|
| Debug | 5 ms |
| Production | 10 ms |
| Slim | 20 ms |

`test runtime` reports:

```text
slime_service_updates_delta
slime_service_skips_delta
```

These counters help confirm that the slow service path is throttled while
RotationData stays fresh (`slime_rotation_sample_lag_end: 0`).

## Rotation scheduler diagnostics

`test runtime` reports SlimeVR rotation scheduler counters:

```text
slime_rotation_send_due_delta
slime_rotation_rate_limited_delta
slime_rotation_no_snapshot_delta
slime_rotation_duplicate_snapshot_delta
slime_rotation_snapshot_age_us_end
slime_rotation_sample_lag_end
slime_rotation_send_failures_delta
slime_control_send_failures_delta
slime_telemetry_send_failures_delta
slime_discovery_send_failures_delta
slime_tap_transport_send_failures_delta
slime_consecutive_send_failures_end
```

`slime_rotation_sample_lag_end` is the trustworthy end-of-test freshness metric:
it compares the last sent RotationData sample with the current runtime sample
count. `slime_rotation_snapshot_age_us_end` is kept for compatibility, but the
IMU FIFO timestamp does not necessarily share the same epoch as `millis()`, so a
large value there should not be interpreted as a real multi-second quaternion
latency unless the sample lag also confirms it.

## Current output policy

- Debug keeps the 100 Hz target output for diagnostics.
- Production keeps the 100 Hz target output unless future runtime tests prove a
  lower product default is acceptable.
- Slim intentionally clamps SlimeVR output to 50 Hz to reduce Wi-Fi work while
  preserving the local FIFO/AHRS cadence.

A measured RotationData rate around 68-73 Hz with zero `no_snapshot`, zero
`duplicate_snapshot` and zero `sample_lag` means the sender is limited by the
prepared-output cadence, not by missing AHRS data.

## Idle/yield experiment

Wave 6 adds an opt-in idle/yield mechanism. It is disabled by default in every
profile and must be enabled only for A/B tests:

```ini
-DTRACKER_ENABLE_IDLE_YIELD=1
-DTRACKER_IDLE_YIELD_MODE=TRACKER_IDLE_YIELD_MODE_DELAY0
-DTRACKER_IDLE_YIELD_EVERY_N_IDLE_LOOPS=16
```

Available modes:

| Macro | Behavior | Risk |
|---|---|---|
| `TRACKER_IDLE_YIELD_MODE_DELAY0` | Calls `delay(0)` on selected idle-candidate loops | lower; scheduler/Wi-Fi yield without intentional 1 ms sleep |
| `TRACKER_IDLE_YIELD_MODE_DELAY1` | Calls `delay(1)` on selected idle-candidate loops | higher; may add latency and should be tested only after `delay(0)` |

`TRACKER_IDLE_YIELD_EVERY_N_IDLE_LOOPS` reduces how often the firmware yields
when many idle candidates are available. `N=1` was rejected as too aggressive: it
kept FIFO/AHRS stable but reduced observed SlimeVR RotationData rate. The current
manual candidate is throttled `delay(0)` with `N=16`; it remains disabled by
default because a product thermal/current benefit is not proven.

## Cooperative FIFO scheduling

Production runtime no longer processes every decoded sample from a large FIFO
event in one app-loop pass. Hardware reads keep the existing configured drain
ceiling, while pending callbacks resume in slices of at most 12 callbacks or
about 4.5 ms. This keeps the single-owner network scheduler responsive without
creating another FreeRTOS task, dropping IMU samples, or invoking UDP from inside
the AHRS sample callback. Manual, network-scan and magnetometer-triggered FIFO
resets clear the local pending batch so pre-reset samples cannot leak into the
new stream.

Runtime tests report:

```text
idle_yield_enabled
idle_yield_mode
idle_yield_every_n_idle_loops
idle_yield_count
temp_slope_c_per_min
temp_recent_slope_c_per_min
```

Acceptance for an idle/yield candidate:

```text
tracking_recovery_delta: 0
quality_estimated_dropped_delta: 0
fifo_overrun_delta: 0
fifo_full_delta: 0
wifi_disconnects_delta: 0
slime_send_failures_delta: 0
slime_unknown_packets_delta: 0
slime_rotation_sample_lag_end: 0
sample_rate_hz remains near baseline
```

## Acceptance checklist

Build checks:

```bash
pio run -e BOARD_LOLIN_C3_MINI_DEBUG
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_SLIM
python tools/report_firmware_size.py
python tools/check_all.py --skip-native --skip-tool-smoke
```

Debug runtime check:

```text
test runtime 600
```

Expected pass conditions:

```text
wifi_connected_end: yes
slime_server_found_end: yes
wifi_disconnects_delta: 0
slime_send_failures_delta: 0
slime_unknown_packets_delta: 0
tracking_recovery_delta: 0
quality_estimated_dropped_delta: 0
fifo_overrun_delta: 0
fifo_full_delta: 0
```

Phone hotspots are not a final acceptance environment. They can show UDP send
failures or reopen requests even while RSSI is good. Use them for rough
thermal/loop testing only; final Wi-Fi policy must be validated on the target
router/AP.
