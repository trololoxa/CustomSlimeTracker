# 0027a sensor liveness and bounded recovery report

## 0027a adversarial-review revision

This lettered patch is a delta applied after 0027. The first 0027 candidate
correctly rebased `FifoInterruptEventSource` during a full sensor reinit, but
its shorter verified FIFO-reset path did not call `fifoEvents->reset()`. After
a long blocking Wi-Fi scan, the old `lastEventAtUs` could therefore be copied
into the newly reset progress watchdog epoch and immediately request a second
false `ImuNoProgress` recovery. The transactional commit now rebases the ISR
counter and clears the old event timestamp after the hardware reset succeeds
and before INT1 is attached again. The focused policy test enforces that
ordering, and the native event-source regression proves that pre-detach IRQs
and the old timestamp do not survive the new epoch.

The first candidate also copied `ImuQualityResult` unconditionally in the
normal 960 Hz path even though mutation is needed only for degraded recovery
output. The normal branch now passes the original quality object directly; a
copy is constructed only inside the recovery branch. No output contract or
recovery decision changed.

Finally, prepared output is now an explicit build contract. Every current
profile already enables it; compilation fails with a clear error if a future
override disables the only orientation-publication source while the sensor
progress watchdog is active.

## Defects and evidence

The pre-0027 runtime had no owner that could prove continued sensor progress.
`g_fifoLastIrqUs` was written by the ISR but never used. A failed FIFO drain
only cleared local work, and the recovery hook cleared timestamp, quality and
software queues even when `resetFifo()` returned false. Runtime enqueue also
accepted a hardware batch one element at a time, so an unexpected capacity
failure could publish a prefix as if it were continuous. Strict recovery kept
gyro propagation internally but suppressed every prepared pose until a clean
gravity window, which made sustained physical motion indistinguishable from a
stopped tracker at the output.

There was also no task watchdog, retained crash-boot record or safe-mode write
gate. Repeated panic/watchdog boots therefore had no bounded operational state
that retained Wi-Fi/CLI and sensor recovery while disabling optional work and
persistent mutations.

## Changed contract and state owners

`SensorProgressWatchdog` is the single liveness owner. It observes last IRQ or
fallback FIFO event, successful hardware drain, accepted gyro timestamp and
successful orientation publication. Its timeout is derived from configured
sample period and watermark, bounded to 0.5..2 seconds, and all arithmetic is
32-bit wrap-safe. Boot, intentional sleep, blocking scan and sensor reinit are
explicit suppression reasons; leaving the final suppression starts a fresh
grace epoch.

`SensorRecoveryController` owns retry order and deadlines. Each episode tries
two verified FIFO reset transactions, escalates to full sensor reprobe/reinit,
and enters a 30-second exhausted backoff after five failed attempts. Exhausted
is a terminal degraded state for the current attempt sequence, not a permanent
lock: bounded full-reinit probes continue until success.

`Lsm6dsvFifoReader` now treats reset and configure as hardware transactions:

1. stop INT1/INT2 routes;
2. enter FIFO bypass;
3. write watermark, batching, mode and routes;
4. read every affected register back;
5. only then clear parser/timestamp state and publish the new software epoch.

Failure leaves the prior software epoch intact, marks hardware state unknown
and keeps collection fail-closed. Manual FIFO reset and post-scan recovery use
the same app-owned controller; low-level, compatibility and interactive setup
Wi-Fi scans share one recovery adapter. A new event epoch snapshots the current
ISR counter, so an interrupt accumulated before detach/reinit cannot masquerade
as fresh progress. The runtime processor stages a complete drained
fragment, preflights both queues, and either enqueues all of it or publishes a
typed discontinuity. A recovery request quarantines remaining old-epoch data
until a successful hardware transaction commits `resetWork()`.

Strict tracking recovery proves four monotonic integrated gyro samples before
publishing degraded gyro-only orientation. Accel correction remains disabled,
linear acceleration is invalid with `RECOVERY_DEGRADED`, and a clean gravity
window is still required for tilt reacquisition. Motion does not reset the
gyro-output proof, and no timeout accepts untrusted accel.

`TrackerApp` owns the ESP task watchdog feed. The only hardware feed operation
is reached after mandatory sensor/recovery, network and command service, both
from the ordinary loop and the equivalent bounded-command service path; it is
never called from ISR/timer code. A CRC/inverse-protected retained boot record
counts consecutive panic/watchdog resets. Three consecutive crash boots enter
safe mode: Wi-Fi, CLI and bounded sensor recovery continue; tap, deferred mag
work, calibration autonomy and config/network/autonomy persistence are gated.
The write gate is installed before config load, including best-effort storage
repair or migration writes, rather than only before later CLI mutations. A
valid legacy record remains readable in this state but is not migrated or
deleted until writes are allowed again.
After 60 seconds of main-loop uptime the crash count and safe-mode gates clear.

No estimator, sensor ODR, accel/mag gate, packet format, config schema or NVS
record format changed. The new runtime controllers are fixed-size and perform
no allocation, logging or expensive math in the sample path.

## Verification

Host acceptance covers:

- no IRQ/drain, no accepted gyro and no orientation progress;
- suppression exit and `micros()`/retry-deadline wrap;
- reset -> reinit -> exhausted backoff -> later success;
- invalid/random retained boot bytes, three crash boots and stable clearing;
- every reset transport write/read step failing, terminating and succeeding on
  a later retry;
- reset read-back failure preserving the old software timestamp epoch;
- drain failure publishing no sample;
- whole-batch capacity failure publishing no prefix;
- old-epoch quarantine until recovery commit;
- pre-reinit IRQ counts not being consumed as fresh events;
- the transactional app reset committing the event epoch after hardware proof
  and before INT1 reattach;
- the old event timestamp being cleared and only a post-reset IRQ becoming
  fresh progress;
- no unconditional `ImuQualityResult` copy in the normal sample path;
- monotonic gyro proof during strict recovery and reset on a new time fault;
- degraded pose with explicitly invalid, zeroed linear acceleration;
- safe-mode write inhibition for main, network and autonomy stores.

The focused `0027 sensor liveness/recovery policy` also enforces typed faults,
one app-owned watchdog feed operation, no ISR clock read, fixed-size recovery
controllers, batch preflight and the required regression tests.

## Not verified here

- ESP32-C3 target compilation and link with the pinned Arduino/ESP-IDF package;
- physical LSM6DSV SPI read-back behavior and injected bus faults;
- task-watchdog panic/reset behavior on the board;
- RTC `.noinit` retention across panic, task-WDT and software reset;
- a short target sensor-fault smoke, motion-light-sleep wake and Wi-Fi scan;
- CPU/WCET, stack, current and recovery-blackout measurements on hardware.

Those target checks are required before release. They are not replaced by the
host tests and no target PASS is claimed by this patch report.

The executable target procedure and pass/fail fields are documented in
`docs/0027a_target_smoke.md`.

## Compatibility and rollback

Rollback is source-only. No persistent or wire migration is required. A
rollback removes the liveness/recovery/safe-mode owners and returns to the old
unsafe reset semantics; it does not need an NVS erase.
