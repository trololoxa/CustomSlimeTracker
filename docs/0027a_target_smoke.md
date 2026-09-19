# 0027a ESP32-C3 target smoke

This is the minimum board check for 0027a. It does not replace the host fault
matrix. Use a test tracker that can be reflashed and keep a copy of its current
config. Do not disconnect or short live SPI/power wiring unless the board has a
purpose-built switch or interposer.

## 1. Build and flash

From the repository root:

```sh
pio run -e BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
pio run -e BOARD_LOLIN_C3_MINI_SLIM
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG -t upload
pio device monitor -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
```

The monitor speed is already 921600 in `platformio.ini`. A normal firmware
upload does not erase NVS, but save the current config before testing if the
tracker is important.

Hard fail:

- any profile does not compile/link;
- image exceeds the configured partition;
- boot loops, panics or enters safe mode on an ordinary cold boot.

## 2. Healthy baseline, 5 minutes

Place the tracker still for 10 seconds, then move it normally for at least one
minute. Run:

```text
health
perf tracking reset
```

After five minutes run:

```text
health
perf tracking
ahrs status
slime status
```

Required:

- `tracker_fault_active=no`;
- `tracker_degraded_no_imu=no`;
- `sensor_recovery_active=no`;
- `safe_mode_active=no`;
- gyro/AHRS timestamps continue advancing;
- rotation packets continue while moving;
- no continuously increasing timestamp-backward, queue-overflow or recovery
  counters;
- no new burst, duplicate pose or permanent output-rate reduction.

Retain both `perf tracking` blocks for baseline comparison. A single bounded
network transient is not a sensor failure.

## 3. Verified manual FIFO recovery

Keep the tracker moving gently and run:

```text
perf tracking reset
fifo reset
```

Wait three seconds, then run:

```text
health
perf tracking
ahrs status
```

Required:

- `fifo reset queued` is printed;
- recovery count increases once and recovery-success count increases once;
- `sensor_recovery_active` returns to `no`;
- `tracker_fault_code=none`;
- AHRS/rotation resumes automatically;
- no stale linear acceleration is published during the recovery blackout;
- the tracker does not require stillness before gyro-driven pose resumes.

Repeat `health` after another three seconds. The recovery count must remain
unchanged. Any second `imu_no_progress` episode without a new fault reproduces
the stale-event bug and is a hard fail.

## 4. Blocking Wi-Fi scan regression

With Wi-Fi configured, start another baseline and run:

```text
perf tracking reset
net scan visible limit 16
```

The scan is an intentional tracking interruption. Do not judge motion lost
during the scan itself. After the result returns, wait three seconds and run:

```text
health
perf tracking
ahrs status
```

Wait another three seconds and run `health` again.

Required:

- exactly one scan-owned recovery episode may occur;
- recovery succeeds and tracking resumes;
- the second `health` does not show another recovery-count increment;
- `last_integrated_t_us` advances;
- `bad_dt_rejects` does not grow continuously;
- no watchdog reboot occurs during the scan.

This is the primary physical regression for 0027a.

## 5. Intentional light sleep and wake

ProductionDiag uses production scheduling while retaining the diagnostic CLI.
Put the tracker on a stable surface and run:

```text
health
sleep
```

Wake it with a normal physical movement, reconnect the console if necessary,
then run:

```text
health
ahrs status
slime status
```

Required:

- no task-watchdog reset while asleep;
- one movement wakes the device;
- FIFO, network and rotation output recover;
- `safe_mode_active=no`;
- sleep/wake does not create a repeated recovery loop.

## 6. Controlled sensor-fault smoke (only with safe hardware isolation)

If the test fixture can isolate the IMU without shorting a live bus, interrupt
the sensor long enough to exceed two seconds. Observe `health`, then restore
the sensor.

Required while isolated:

- a typed sensor fault becomes visible;
- output is marked degraded rather than presenting stale acceleration as
  fresh;
- retries are bounded; the device remains responsive over Serial/Wi-Fi;
- there is no rapid reboot loop.

Required after restoration:

- a later bounded probe reinitializes the sensor automatically (the exhausted
  retry interval is 30 seconds, so allow at least 35 seconds);
- gyro/AHRS timestamps and rotation resume;
- recovery-success count increments;
- no manual reboot or config erase is required.

If there is no proper isolation fixture, record this section as **not
verified**. Do not improvise by shorting SPI, CS, 3V3 or ground.

## 7. WDT and retained boot record (extended target gate)

This requires a temporary test-only image or debugger that deliberately stops
the app loop after setup. Do not add the fault trigger to a release image.

Verify separately:

1. one task-WDT reset is reported as `crash_or_watchdog` on the next boot;
2. three consecutive task-WDT/panic resets enter safe mode;
3. safe mode keeps CLI, Wi-Fi and bounded sensor probes available while NVS
   mutations are rejected;
4. 60 seconds of stable main-loop uptime clears safe mode and the retained
   crash count;
5. power-on, brownout and random/corrupt retained bytes do not falsely enter
   safe mode;
6. an intentional light-sleep wake is not counted as a crash.

Also measure the actual task-WDT timeout on the pinned Arduino/ESP-IDF build.
If the framework already initialized a timeout shorter than 15 seconds, the
synchronous Wi-Fi scan result is not accepted until it is proven shorter than
that timeout or the watchdog ownership is corrected.

## 8. Result record

Record:

```text
board/revision:
commit or patch sha256:
PIO versions:
profiles built:
healthy baseline: PASS/FAIL
manual FIFO reset: PASS/FAIL
blocking scan: PASS/FAIL
light sleep/wake: PASS/FAIL
controlled sensor fault: PASS/FAIL/NOT VERIFIED
WDT/noinit extended gate: PASS/FAIL/NOT VERIFIED
max FIFO queue age / high-water:
recovery blackout:
unexpected resets:
notes:
```

The patch is target-smoke ready only when sections 1-5 pass. Release readiness
also requires the relevant controlled fault and retained-state checks, with
anything unavailable stated explicitly as not verified.
