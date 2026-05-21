# Runtime hot-path optimization notes

This document tracks the safe performance/power optimizations that do not
change IMU ODR, LSM6DSV performance mode, AHRS math, calibration application or
SlimeVR RotationData payload quality.

## SlimeVR runtime configuration policy

`SlimeVROutputRuntime::configure()` is a static-configuration operation. It may
reset UDP/session state when endpoint, device identity, magnetometer support,
telemetry policy or output rate changes. It must not be called on every main
loop iteration.

The app layer now separates:

- static SlimeVR config refresh: profile-limited and cached;
- live runtime state update: temperature, battery and rest-calibration status;
- runtime update: Wi-Fi/UDP state machine and packet emission.

This keeps config changes responsive while avoiding repeated config rebuilds and
potential state churn in the hot path.

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

## Runtime work / idle candidate metrics

Debug runtime tests collect coarse work counters:

- `work_loop_count`
- `idle_candidate_loop_count`
- `idle_candidate_ratio`
- `fifo_work_count`
- `battery_work_count`
- `network_work_count`
- `tap_work_count`
- `led_work_count`
- `heartbeat_work_count`

These counters are measurement-only. They are intended to decide later whether a
safe `delay(0)`/`vTaskDelay(1)` experiment is worth testing. No sleep/yield
policy is enabled by these metrics.

## Live SlimeVR state refresh

`SlimeVROutputRuntime::configure()` is a static configuration operation. The
application now updates live state separately and can throttle that refresh with
`TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS`:

```text
Debug:      0 ms    // immediate, easiest to inspect
Production: 1000 ms // enough for battery/temp/rest-calibration UI
Slim:       5000 ms // no product telemetry, lowest polling pressure
```

This does not change FIFO, IMU ODR, AHRS or quaternion generation. At worst it
delays non-tracking metadata such as battery percentage or the SensorInfo
rest-calibration bit by the configured interval.

## Rotation scheduler diagnostics

`test runtime` now reports SlimeVR rotation scheduler counters:

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

These counters separate network/output scheduling from AHRS quality. A low
observed RotationData rate can now be traced to rate limiting, missing fresh
snapshots, duplicate snapshots, or actual UDP send failures.

`slime_rotation_sample_lag_end` is the trustworthy end-of-test freshness metric:
it compares the last sent RotationData sample with the current runtime sample
count. `slime_rotation_snapshot_age_us_end` is kept for compatibility, but the
IMU FIFO timestamp does not necessarily share the same epoch as `millis()`, so a
large value there should not be interpreted as a real multi-second quaternion
latency unless the sample lag also confirms it.

Send failures are split by packet purpose. This is important for Wi-Fi power
work: if only RotationData fails, output pacing/TX pressure is suspect; if
control or discovery packets fail, the AP/hotspot or UDP socket state is more
likely involved.

## Hot-path perf counters

`TRACKER_HAS_HOTPATH_PERF` is the single contract for expensive per-sample and
per-FIFO timing instrumentation. It is enabled when a profile needs hot-path
measurements, static tests, or runtime tests.

Profile policy:

| Profile | `TRACKER_HAS_HOTPATH_PERF` | Reason |
|---|---:|---|
| Debug | on | runtime/static tests and performance diagnostics |
| Production | off | no test runners; avoid per-sample `micros()` timing |
| Slim | off | smallest/lowest-overhead runtime path |

When this contract is off, the firmware does not call `micros()` just to measure
sample processing or FIFO batch duration. Timestamping that is required for data
correctness, such as FIFO drain timestamps or non-blocking FIFO fallback polling,
remains enabled.

## Wave 3 acceptance checklist

Wave 3 is considered complete when all profile builds pass and the Debug runtime
report proves that the new scheduling does not hide work or break networking.

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

New fields to inspect:

```text
idle_candidate_ratio
fifo_work_count
network_work_count
slime_rotation_send_due_delta
slime_rotation_rate_limited_delta
slime_rotation_no_snapshot_delta
slime_rotation_duplicate_snapshot_delta
slime_rotation_snapshot_age_us_end
```

A real observed RotationData rate around 73 Hz is not a failure by itself. With a
100 Hz target it means the scheduler is close enough for normal tracking tests,
but the counters above should explain where the remaining 27 Hz are lost. For a
battery-oriented Slim build, 50 Hz is an intentional default unless later runtime
A/B tests show that 100 Hz is worth the extra Wi-Fi work.

## What Wave 3 intentionally does not do

Wave 3 does not change:

- LSM6DSV ODR;
- LSM6DSV high-performance mode;
- AHRS math;
- gyro/accel/mag calibration application;
- SlimeVR RotationData quaternion payload.

Wi-Fi modem sleep, TX power experiments, CPU frequency changes and idle/yield
policy belong to later waves because they require hardware A/B measurements.


## Wave 3 follow-up: real work vs scheduled checks

`test runtime` work counters must count real work, not merely that a hook was
called. In particular:

- `network_work_count` increments only when the Wi-Fi/SlimeVR runtime passes its
  profile update budget and actually runs the network state machines.
- `heartbeat_work_count` increments only when a boot heartbeat line is printed.
- `idle_candidate_loop_count` is allowed to become non-zero in Debug. A zero
  value means at least one hook is still claiming work every loop and must be
  fixed before testing any yield/sleep policy.

Default network update budgets after wave 3:

| Profile | `TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS` | Notes |
|---|---:|---|
| Debug | 2 ms | Keeps diagnostics responsive while exposing idle candidates. |
| Production | 2 ms | Below the 10 ms period of 100 Hz RotationData. |
| Slim | 5 ms | Below the 20 ms period of the 50 Hz Slim default. |

Acceptance for closing wave 3:

- Wi-Fi and SlimeVR stay connected for `test runtime 600`.
- `wifi_disconnects_delta`, `slime_send_failures_delta`, FIFO errors and recovery
  counters remain zero.
- `idle_candidate_loop_count` is non-zero, proving the firmware can now measure
  whether future idle/yield experiments are safe.
- `network_work_count` is lower than `loop_count`; it should roughly reflect the
  configured network update budget, not the raw loop rate.

## Wave 4 network-output policy

Wave 4 tunes telemetry and SlimeVR output scheduling without changing tracking
quality. The current policy is:

- Debug keeps 100 Hz target output for diagnostics.
- Production keeps 100 Hz target output unless runtime tests prove that a lower
  product default is acceptable.
- Slim intentionally clamps SlimeVR output to 50 Hz to reduce Wi-Fi work while
  preserving the local FIFO/AHRS cadence.

A measured RotationData rate around 68-73 Hz with zero `no_snapshot` and zero
`duplicate_snapshot` means the sender is limited by when fresh prepared output
snapshots become visible to the network scheduler, not by missing AHRS data.
That is acceptable for moving to Wi-Fi power A/B tests, but the runtime report
must still show zero persistent UDP failures on the final router test.

Phone hotspots are not a final acceptance environment. They can show UDP send
failures or reopen requests even while RSSI is good. Use them for rough
thermal/loop testing only; final Wi-Fi policy must be validated on the target
router/AP.

The UDP reopen threshold is deliberately higher than the original debug value.
Transient `WiFiUDP::endPacket()` failures should not cause repeated socket
reopen churn, because the reopen itself costs time and can increase packet
loss. Consecutive failures still reopen the socket quickly enough when the UDP
state is genuinely stuck.

## Wave 4 closure criteria

Wave 4 is considered complete when the runtime report shows that SlimeVR output
and telemetry scheduling are stable on the normal home router/AP, not only on a
phone hotspot. Phone hotspots are useful for stress testing, but they are not the
acceptance baseline because they can add UDP loss, aggressive power management
or background channel changes outside the firmware's control.

A passing home-router runtime report should show:

```text
wifi_connected_end: yes
slime_server_found_end: yes
wifi_disconnects_delta: 0
slime_send_failures_delta: 0
slime_rotation_send_failures_delta: 0
slime_control_send_failures_delta: 0
slime_telemetry_send_failures_delta: 0
slime_discovery_send_failures_delta: 0
slime_udp_reopen_requests_delta: 0
slime_rotation_no_snapshot_delta: 0
slime_rotation_duplicate_snapshot_delta: 0
slime_rotation_sample_lag_end: 0
```

The observed RotationData rate can remain below the configured target when it is
bounded by the prepared-snapshot cadence. The important tracking-quality checks
are that fresh snapshots are available, no duplicate/no-snapshot counters grow,
no UDP send failures occur on a stable AP, and local FIFO/AHRS sample processing
continues without drops or recovery events.
