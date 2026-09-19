# 0028d — recovery proof and calibration acceptance

Additive patch after **0028c**. Scope: R1–R7 of the 2026-09-06 final audit of
0027–0028c. This is a source/host-tested correction, not a target-verified release.

## Defects and changed contracts

- **R1:** A successful reset/read-back enters `awaiting_progress`. Only a fresh
  drain plus accepted gyro, and orientation when expected, completes recovery.
  Failure to produce that evidence uses the same bounded episode: two FIFO
  attempts, full reinit, exhausted 30 s backoff with continuing probes. An
  additional fault cannot restart this sequence at attempt zero. Every attempt,
  including full reinit, rebases above the highest already assigned timestamp.
  Tap setup remains restricted to full sensor reinit.
- **R2/R6:** One `FifoCalibrationCaptureSession` owns direct FIFO capture and its
  held-out validation. Wait slices are at most 20 ms. After each bounded sensor
  attempt, the application services console, battery, network/status and the
  task watchdog; it excludes the ordinary FIFO consumer, tap work and optional
  calibration workers. Pose is invalidated on entry and the normal bounded
  sensor recovery owns exit. Mag samples collected against interrupted tilt
  are discarded rather than fused. No watchdog feed was added to an ISR.
- Capture has one wrap-safe absolute deadline, including validation, explicit
  cancel and final pre-accept checks. Zero/invalid bounds cannot disable it.
  Missing, incoherent, saturated, duplicate, reversed and gap samples restart
  clean evidence, not calibration authority. Two verified local drain resets
  are allowed; a failed reset terminates capture. Validation reports progress.
  Cancellation chains existing hooks, bounds input consumption to 32 bytes per
  scope/poll, and restores the previous hook even when no stream is installed.
- **R3:** The slot writer is **v4**, with the same payload layout. A named v3
  reader normalizes only previously accepted historical policy: ignored baud
  to the compile-time baud, debug/quaternion flags to the old quaternion
  precedence, and incompatible positive dt override to automatic FIFO dt.
  Full current semantics still apply after normalization. Calibration/frame,
  magnetic model, user mag enable and quality gates are not repaired or erased.
  The inactive v4 slot is written/read back before selector commit and RAM
  apply. v1/v2 migration remains available. Safe boot permits explicitly
  reported read-only volatile migration. Invalid calibration still fails.
- **R4/R7:** Quaternion corruption is fatal even in stale, invalid or duplicate
  snapshots. Repeated polling of one invalid/stale publication does not spend
  its budget repeatedly. A discontinuity interrupts quaternion comparisons;
  the contaminated window cannot pass. One fresh retry is possible for
  recoverable contamination, never for failed quaternion finiteness, norm or
  continuity. Retry uses a loop and a single 22 s verification deadline, not
  recursive stack frames. Transport diagnostics do not veto sensor acceptance.
- **R5:** Manual gyro/accel/cal save, temperature save, learned config save,
  candidate promotion and setup checkpoints share `TrackerCalibrationTransaction`.
  The magnetic CLI routes learned saves through it as well. A detached semantic
  candidate is written/read back to the inactive slot, then tested as a named
  preview while a write barrier protects old authority, then committed. A
  prepared candidate promotion reuses its existing prepared slot. Rollback
  restores exact pre-stage config, IMU calibration, temperature compensator and
  session trim. An uncertain selector retains the working preview; the store uncertainty
  latch inhibits writes; it must be reconciled by load/reboot, never guessed.
  Initial partial models use bounded fresh finite sensor probation rather than
  requiring already calibrated 6D/9D or a Server. Complete models use final
  output verification. Ordinary saves of unchanged calibration/policy do not
  require stationary calibration verification.

The asynchronous autonomy controller remains its existing journal owner, using
these same storage prepare/commit/rollback primitives. Its reboot journal,
absolute probation deadline and separate sensor/transport verdict are retained;
this patch does not replace that state machine with the synchronous CLI owner.
A manual preview predating `save` remains the pre-stage RAM rollback point; the
previous durable model remains authoritative until successful commit.

## Performance and limits

No changes to AHRS integration/correction, quaternion hot-path admission, accel
or magnetic thresholds, IMU ODR, prepared-output cadence, packet format or TPS.
New capture validation runs only during explicit interrupted calibration.
Recovery proof runs in the existing low-rate progress service. Transaction
snapshots have one static CLI/setup owner; no per-sample heap/logging/math was
added. Its host size increases from 1376 B to 2208 B (+832 B) for the immutable
candidate and exact temperature rollback. Target RAM/flash size is not measured. Config save uses a cold-path temporary for comparison with durable state.

Stack verification uses an opaque `millis()` declaration because the ordinary
native constant-zero clock optimizes away blocking verification loops. Measured
Linux/GCC `-O2` frames: verification attempt 1552 B, retry wrapper 144 B. Unlike the old recursive retry (two 1520 B attempt
frames), only one attempt frame can exist. These are host individual-frame
measurements, not an ESP32 whole-call-chain stack or WCET measurement.

## Tests run

Targeted native cases cover no-stream reset success/escalation/wrap/late proof;
shared capture deadline past 15 s, cancellation and ownership, bounded wait
slices, stale/reordered/saturated evidence; stale/duplicate corrupted quaternion;
retry precedence; v3 migration with preserved calibration/mag and torn writes;
inactive prepare before preview, sensor rejection, write/read-back failure,
selector uncertainty, immutable caller candidate, exact temperature/trim rollback.
Existing storage, autonomy and FIFO fault tests remain required by `check_all`.

Final aggregate gate and apply-check results are recorded in the delivery report.
The added policy uses an opaque clock for stack checks and native tests for
behavior. Existing numerical hot-path budgets have not been raised.

## Storage compatibility / rollback

v4 is a semantic-reader version boundary, not a payload-ABI migration. Explicit
fixed-width storage encoding remains roadmap 0036. Keep the firmware with v4
support when reverting behavior. Firmware through 0028c does not understand v4;
a raw downgrade is not a supported rollback procedure and may select an older
committed slot. Back up current configuration/diagnostics before target smoke.
The old v3 slot is retained during migration, but subsequent successful A/B
writes can reuse it. Do not rely on it as a permanent downgrade backup.

## Target smoke

1. Build Production and ProductionDiag; run `python tools/check_all.py` locally.
   Flash ProductionDiag without erasing NVS. Record build identity, boot config
   load/migration status, `health`, `config status`, `mag processed`, `mag yaw status`.
   Existing valid models/user mag enable must survive; physical interference may
   still legitimately keep yaw closed.
2. Run `fifo reset`, then `health` after 2–3 s. Expect awaiting proof to end in
   idle with one success, fresh gyro/pose, no tap spam, no timestamp regression.
   Rotate during recovery: gyro propagation must resume without forced accel.
3. Run a gyro capture on a stationary tracker; test `q` cancellation separately.
   For an accel face capture, keep moving for more than 15 s, then cancel.
   Console/network remain responsive, no watchdog reboot, no accepted moving
   calibration. On exit expect bounded recovery and fresh normal output.
4. Save an intended calibration while stationary; expect persisted candidate
   probation then commit. During a separate probation press `q`: previous NVS
   authority must remain usable after reboot. An intentional unsaved preview can
   remain the pre-stage RAM snapshot. Never erase NVS to work around rejection.
5. Check normal tracking/SlimeVR reception, reboot and the already working manual
   sleep/wake path. Compare FIFO loss/queue age and profiler/stack headroom with
   the same build profile and active workload. No new motion dataset is required.

Host fault injection covers missing IRQ/gyro, failed reset/reinit and NVS
failures. A physical stopped-source smoke, if available, must verify escalation
and recovery after restoring the source. Software manual reset alone does not
exercise absent hardware. Do not disconnect a powered SPI bus to induce faults.

## Not verified

ESP32-C3 production linking, flash/RAM size, actual watchdog/USB/network timing,
whole-call-chain stack, sleep retention and physical sensor-failure recovery.
Current SlimeVR Server interoperability and new sensor replay were not run.
Existing replay/fixture gates do not prove new before/after numerical tracking
performance. These limits prevent claiming target/release closure of 0027/0028.
