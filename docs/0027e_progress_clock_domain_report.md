# 0027e progress clock-domain report

## Scope and ordering

0027e is additive after 0027d. It fixes the target-proven cause of the remaining
sensor recovery loop. It does not change sensor ODR, FIFO geometry, AHRS math,
accel/mag gates, persistence, networking or output-rate policy.

## Target evidence

ProductionDiag remained operational and SlimeVR received pose, but the health
snapshot showed:

- 4,885 watchdog faults/recovery successes by 121 seconds;
- 21,154 by 549 seconds;
- every fault was `no_accepted_gyro`;
- zero FIFO drain, queue, capacity, overrun, full, timestamp or quality faults;
- continuous IMU/magnetometer samples and coherent pairs;
- `tracking_recovery_monotonic_gyro_samples=0`, because every false recovery
  restarted the strict-recovery proof;
- IMU progress timestamps lagging local uptime by about 0.7–0.85 seconds while
  the configured local watchdog timeout was 0.5 seconds.

LSM hardware timestamps have a sensor-owned epoch. They are valid for sample
ordering and integration but are not MCU `micros()` timestamps. Unsigned local
timeout subtraction therefore classified a healthy stream as stale.

## Changed contract

- The IMU sample path publishes a 32-bit progress sequence edge for accepted
  gyro and orientation publication. It does not read a clock.
- `SensorProgressWatchdog::evaluate()` detects sequence changes and records them
  using its `nowUs` argument, which is already in the local `micros()` domain.
- Epoch reset snapshots current sequences, so pre-recovery progress cannot be
  reused as post-recovery evidence.
- Natural 32-bit sequence wrap is safe because equality is checked at every
  bounded watchdog service interval, not after an entire counter period.
- IRQ/drain timestamps remain local observations and all existing deadlines
  remain enforced.

The hot-path operation replaces a timestamp conversion/assignment with one
fixed-width increment. There is no additional `micros()`, allocation, floating
point, logging or callback.

## Verification

- focused sensor-liveness native regression, including a deliberately offset
  producer/watchdog clock scenario;
- compile-only check of the IMU sample pipeline;
- source policy forbidding producer timestamps and hot-path clock reads;
- `-Os` host comparison: the main pipeline stack frame remains 384 bytes and
  object text decreases from 5,868 to 5,842 bytes;
- Python syntax, diff hygiene and clean patch-chain application.

## Target rerun

After flashing, capture boot and run `health` twice at least five minutes apart.
Pass requires the recovery-controller request/success counters to remain stable,
`sensor_progress_current_fault=none`, increasing IMU and FIFO interrupt counts,
zero FIFO/quality fault counters, and normal SlimeVR pose delivery. A single
manual `fifo reset` must increase request/success by one only, must not print a
new tap setup banner, and must permit recovery to complete or gyro-degraded pose
to unlock after four monotonic integrations.

At 0027e publication, target build, physical smoke, target stack high-water and
post-fix performance comparison remained unverified by host tests. The later
0027f report records the completed functional target smoke; target stack
high-water remains a separate measurement.
