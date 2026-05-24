# Firmware optimization wave status

This document records the current accepted state of the optimization roadmap.
Temporary A/B environments and rejected candidates should not be kept in the
committed PlatformIO matrix.

## Wave 1 — build profiles and guardrails

Status: **closed**.

Done:

- Debug / Production / Slim profiles.
- `src/build_config/*` split from the monolithic `defines.h`.
- Build/profile contract checks.
- Source-filter validation.
- Firmware size report tooling.
- Boot delay limited to Debug.

## Wave 2 — compile-time slicing

Status: **closed for the safe/product path**.

Done:

- Production removes test, machine-log, full status and full config dump modules.
- Slim removes serial CLI, setup/calibration UI, tests, battery/tap/LED runtime,
  stream/log diagnostics and calibration capture modules.
- App/runtime wiring is null-safe for missing profile subsystems.

Deferred:

- Splitting very large CLI files further (`tracker_setup_commands.cpp`,
  `tracker_mag_commands.cpp`) is possible but is a separate refactor because it
  can affect setup/calibration UX.

## Wave 3 — hot-path cleanup

Status: **closed**.

Accepted result:

- FIFO/AHRS sample rate remains about 931 Hz.
- No FIFO overrun/full, no tracking recovery and no estimated dropped samples in
  accepted Debug runtime runs.
- Runtime work/idle classification is meaningful.
- SlimeVR static config and live telemetry updates are separated from the normal
  packet/runtime hot path.
- SlimeVR RotationData scheduling is separated from slower service-path work
  (incoming packets, heartbeat, SensorInfo, telemetry and discovery).
- Production/Slim avoid Debug-only per-sample/per-FIFO timing overhead.

## Wave 4 — telemetry/rate tuning

Status: **closed on the home-router baseline**.

Accepted result:

- Home-router runtime test was clean: zero Wi-Fi disconnects, zero UDP send
  failures, zero UDP reopen requests, zero unknown packets and zero server
  silence resets.
- Rotation send rate around 68-73 Hz is accepted for the current prepared-output
  cadence because `no_snapshot`, `duplicate_snapshot` and `sample_lag` counters
  stay clean.
- Slim intentionally caps SlimeVR output at 50 Hz for the battery profile.

Phone hotspot behavior is treated as a stress environment, not the acceptance
baseline.

## Wave 5 — Wi-Fi power and thermal A/B

Status: **diagnostics complete; no new default accepted**.

Accepted default:

```text
TRACKER_WIFI_POWER_SAVE_MODE = TRACKER_WIFI_POWER_SAVE_NONE
TRACKER_WIFI_TX_POWER        = WIFI_POWER_8_5dBm
```

Current conclusion:

- Wi-Fi power-save modes are correctly applied and reported.
- TX power overrides are correctly applied and reported.
- Tested Wi-Fi power-save/TX-power variants did not show a convincing
  steady-state thermal improvement in Debug runtime tests.
- 5 dBm TX had rotation send failures in one run.
- 2 dBm TX was clean in one run but is not adopted as a product default because
  it did not prove a thermal benefit and may be AP/RSSI dependent.

Next useful work:

- Compare Production and Slim with external power/temperature measurement, or
  add a temporary local Slim diagnostic overlay later.
- Continue Wi-Fi A/B only if a candidate shows a measurable steady-state
  temperature or current reduction without transport failures.

## Wave 6 — idle/yield

Status: **framework ready; disabled by default**.

Accepted default:

```text
TRACKER_ENABLE_IDLE_YIELD = 0
```

Current conclusion:

- Runtime tests report work/idle candidate counts and idle-yield counters.
- `delay(0)` with `TRACKER_IDLE_YIELD_EVERY_N_IDLE_LOOPS=1` was rejected as too
  aggressive: it kept FIFO/AHRS stable but reduced observed SlimeVR RotationData
  rate.
- Throttled `delay(0)` with `TRACKER_IDLE_YIELD_EVERY_N_IDLE_LOOPS=16` is the
  current safe manual A/B candidate: it kept FIFO/AHRS stable and preserved the
  normal home-router RotationData rate range.
- It is not enabled in default profiles because thermal/current benefit is not
  proven in Production/Slim.

Experimental items still not accepted as defaults:

- `delay(1)` idle sleep;
- CPU-frequency changes;
- light sleep;
- IMU low-power or ODR reduction.
