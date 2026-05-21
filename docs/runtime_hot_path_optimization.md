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
