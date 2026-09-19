# 0027d recovery feedback and tap report

## Scope and ordering

0027d is additive after 0027c. It addresses a target-observed recovery feedback
loop whose visible symptom was repeated `tap_runtime_enabled=yes hardware=ok`
output. It does not change sensor ODR, FIFO capacity, estimator math, accel/mag
gates, persistent schemas or SlimeVR packet cadence.

## Evidence and defect

The target continued producing samples and reported zero FIFO drain, overflow
and whole-batch capacity failures, with shallow empty queues. Nevertheless it
remained in strict `RECOVERING`, its FIFO interrupt counter repeatedly restarted,
and tap setup was printed continuously. Two contracts were crossed:

- progress watchdog treated AHRS initialization as proof that pose publication
  was currently allowed, although strict recovery deliberately withholds pose
  until a bounded monotonic-gyro proof;
- every successful FIFO-only reset reran tap setup even though that transaction
  does not reset the LSM embedded-function registers.

The initial unconditional interrupt detach also asked the ESP32 GPIO layer to
remove a handler before one had been attached, producing a boot-time driver
error despite later successful attachment.

## Changed contract

- Orientation-publication liveness starts only when AHRS is initialized and
  normal publication is active, or strict recovery has unlocked degraded
  gyro-only output. IRQ/drain and accepted-gyro liveness are never relaxed.
- FIFO-only recovery retains tap configuration. Full sensor reinitialization,
  startup and light-sleep resume continue to configure tap hardware.
- FIFO interrupt attachment is idempotent and explicitly owned by the runtime
  hook.
- Progress watchdog preserves the most recent triggered fault across epoch
  reset. `health` reports watchdog timestamps/faults, recovery counts, monotonic
  gyro proof and degraded-output permission.

## Host verification

- focused sensor-liveness native test;
- compile-only checks for runtime status and Arduino app composition;
- 0027d source/behavior policy;
- Python syntax and patch-chain apply check.

## Target smoke

Build and flash the ProductionDiag environment. Capture boot plus at least ten
minutes of ordinary tracking, including one manual FIFO reset if the command is
available. Pass requires:

- no boot-time `gpio_isr_handler_remove` error;
- exactly one tap setup banner at ordinary startup, plus one only after a real
  full sensor reinitialization;
- no repeating tap banner after FIFO-only recovery;
- increasing `runtime_samples` and `fifo_int_count` between two `health` calls;
- zero drain/capacity/queue-overflow counters;
- `sensor_recovery_request_count` stable during an undisturbed five-minute
  window;
- after strict recovery, `tracking_recovery_monotonic_gyro_samples` reaches four
  and `tracking_degraded_gyro_output_allowed=yes` even if motion prevents tilt
  reacquisition;
- SlimeVR pose continues without a new long blackout.

A persistent `RECOVERING` state while the device is continuously moving is not
itself a failure: gravity may be unobservable. It is a failure if degraded gyro
output remains disabled after four clean monotonic gyro integrations, or if the
sensor recovery request counter continues increasing with no typed FIFO fault.

Target build, ESP32-C3 timing/stack observation and physical fault injection are
not host-verified and remain required before merge.
