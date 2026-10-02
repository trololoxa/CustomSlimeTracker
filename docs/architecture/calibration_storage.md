# Calibration storage and candidate framework

Active configuration uses verified dual slots; calibration candidates are separate
and cannot become authoritative merely because their bytes were written.
The current writer is slot v4, with v1/v2/v3/v4 compatibility readers. Payload ABI
is unchanged. The shared transaction contract below owns preview and rollback.

## Active dual-slot layout

Core IMU/calibration config uses the NVS namespace `tracker` and these keys:

```text
cfg_a   full active-slot record A
cfg_b   full active-slot record B
cfg_s   CRC-protected active selector
cfg_ac  slot-A commit marker (slot v2/v3/v4)
cfg_bc  slot-B commit marker (slot v2/v3/v4)
cfg     legacy single-blob key, migration input only
```

Every slot contains:

- storage magic/version/size;
- monotonic generation;
- calibration-relevant sensor signature;
- active quality summary and calibration provenance;
- a complete validated `TrackerConfigBlob`;
- a record CRC covering header, signature, quality and payload.

Slots v2/v3/v4 use CRC-protected commit markers. Legacy v1 records retain their
legacy commit authority. Compatible old versions migrate to a validated v4 slot;
current semantic validation applies before persistence and RAM application.

A save always removes the target slot's old commit marker, writes the inactive
slot, reads it back byte-for-byte and validates its CRC/signature/payload. Only
then is `cfg_s` rewritten and the matching slot commit marker written. The
previous selected slot remains authoritative throughout. An interrupted first
save cannot become active merely because one complete-looking slot exists.

If the selector points to a corrupt slot, boot falls back only to a slot with
commit authority: either a matching v2/v3/v4 marker or a compatible v1 legacy record.
A newer non-selected v2/v3/v4 slot without a marker is an uncommitted prepared write
and is never promoted by fallback. With a missing selector, committed v2/v3/v4 markers
allow the newest committed generation to be recovered; two legacy v1 slots use
the conservative older-generation rule. Selector/marker repair is best-effort
and never overrides a valid authoritative payload.

## Legacy migration

When no committed dual-slot generation is available, the loader checks the old
`cfg` blob. A valid legacy blob is written to slot A, read back, selected, marked
committed, and only then removed. If a previous migration stopped after writing an
uncommitted marker-based artifact, it is cleared and migration safely retries from
the still-valid legacy blob.
Once the selector commit succeeds, legacy-key removal is best-effort and non-fatal:
the committed slot is loaded immediately and stale-key cleanup is retried. If a
dual-slot generation already exists, `config migrate` can only remove a stale
legacy key; it can never overwrite the authoritative active generation.

Commands:

```text
config slots
config verify
config migrate
```

Normal boot migration is automatic; `config migrate` is a service command for
inspection and controlled recovery.

## Sensor signature

Every active slot and calibration candidate is bound to:

- LOLIN C3 Mini / ESP32-C3 identity;
- LSM6DSV identity;
- QMC6309 magnetometer identity;
- IMU ODR;
- accel and gyro full scale/mode;
- FIFO accel/gyro BDR;
- `sensorToDevice` frame hash;
- calibration schema fingerprint.

A stored slot is invalid if the signature does not match its own payload. A
candidate cannot be promoted when its signature differs from the selected
active slot, even with `promote force`. New candidate metadata records a revision hash of the active calibration payload
at staging. Unrelated saves such as output-rate, AHRS-policy or CLI changes do not
invalidate the candidate; any gyro/accel/mag/alignment calibration change does.
Legacy v1 candidates remain readable under their original generation-based rule.
The sensor signature still blocks incompatible ODR/full-scale/frame/schema changes.

## Candidate storage

Candidate data is separate from active config:

```text
cfg_c   candidate record
```

The record contains a complete config snapshot plus:

- provenance (`manual`, `setup`, `background`, `imported_legacy`);
- creation uptime and active calibration revision at creation;
- sample/window coverage;
- overall and per-subsystem quality;
- residual metrics;
- last comparison result and reason flags;
- candidate generation and persisted-write count;
- sensor signature and CRC.

In Debug, Production and ProductionDiag, a candidate is first staged in RAM. Staging never
changes active tracking and can be evaluated repeatedly without flash writes.
Slim keeps the dual-slot active store and migration, but compiles out the
876-byte RAM candidate cache and interactive candidate API because Slim has no
calibration CLI or background collectors. Default candidate NVS policy
requires both meaningful quality improvement and a five-minute minimum interval
between non-forced writes. `force` is a service override for explicit manual
operations, not a background-learning default.

Commands:

```text
cal candidate status
cal candidate stage [manual|setup|background] [flush|force]
cal candidate flush [force]
cal candidate compare
cal candidate discard
cal candidate promote [force]
```

`stage` captures the current runtime calibration/config into the candidate
framework. The stored snapshot is useful for audit/replay, but promotion composes
only calibration-owned fields onto the current active config. Output rate, AHRS
tuning, FIFO/SPI settings, tap mapping, magnetometer enable policy and
temperature-compensation enable policy remain owned by the current active
generation. Magnetometer trust bounds are calibration-derived and move with the
field model. Future patches
0022/0023 will call the same API with their own fit, coverage and held-out quality
metrics.

### measured axis quality

Background `magToImu` candidates store the solver-derived alignment quality in
the existing `TrackerCalibrationQualitySummary` and set
`ALIGNMENT_MEASURED|SOURCE_MEASURED`. Older active records only encode valid axis
alignment as binary score `1.0`; when and only when a candidate has measured axis
quality, comparison treats that old binary value as an unmeasured neutral
baseline. Runtime staging has already required the candidate to beat active
alignment on the exact same motion intervals. Once a measured candidate is
promoted, subsequent candidates compare measured score to measured score.

The quality flag uses an existing metadata bit and does not change config version,
blob size, candidate format, NVS keys or migration behavior. A lightweight
`candidateExists()` probe supports deferred background service without scanning
active slots; structural validation and actual staging still use the normal store
transaction path.

## Atomic promotion

Promotion is deliberately two-phase:

```text
candidate validate + compare + active-calibration-revision check
  -> compose calibration fields onto current active config
  -> write/read-back inactive slot (selector unchanged)
  -> apply calibration state to runtime without IMU/FIFO restart
  -> switch selector only after successful runtime apply
```

If runtime apply is aborted, the inactive slot is restored with a copy of the
previous active record so redundancy is retained. A newer prepared slot is never
accepted as fallback. Selector write/read-back disagreement is reconciled against
the actual NVS selector; an unresolved state is reported as `CommitUncertain` and
requires reload/reboot before further writes. Active-slot writes are latched off
until a successful `load` resolves the authoritative selector. A reboot between inactive-slot
preparation and selector commit still loads the old active slot.

Manual promotion normally requires:

- matching sensor signature;
- finite quality values;
- minimum overall improvement;
- no gyro/accel/mag/alignment/coverage regression.

`promote force` may bypass quality thresholds, but never signature compatibility,
stale-calibration protection, structural validation or the terminal
`already_promoted` state. Measured candidate quality and provenance are retained
in the promoted active slot; unrelated config saves preserve both. Promotion
best-effort persists `lastComparison=promoted`; a failed metadata write leaves a
RAM-dirty retry that `cal candidate flush` can complete without creating another
active generation.

## Scope boundary

The storage owner provides comparison, wear control, diagnostics and promotion. It
does not automatically learn gyro, accel, magnetometer or alignment candidates.
Collectors and fit policies belong to magnetic alignment and calibration autonomy.

## Storage stack ownership

Storage scratch records use bounded heap workspaces outside the sample hot path.
Migration runs after the initial resolver returns, avoiding recursive
`load -> resolveActive -> migrateLegacy -> saveInternal -> resolveActive`.
CRCs do not require a candidate-sized stack copy; absent keys are probed with
`Preferences::isKey()` before their blob length is read.

`tests/contracts/test_calibration_storage_stack.py` compiles the storage units with
`-fstack-usage`, rejects recursive migration, and keeps every storage control
frame below the declared host-side stack budget.


## transaction and lifecycle hardening

A valid record must also satisfy lifecycle rules:

- stale active-generation rejection for candidates;
- calibration-only composition and runtime application;
- preservation of measured active quality;
- rollback-slot restoration after aborted promotion;
- protection against a newer uncommitted slot becoming fallback;
- non-fatal, retryable legacy-key cleanup;
- bounded selector-commit reconciliation and explicit `CommitUncertain`;
- separate boot load states: `loaded`, `migrated`, `defaults_not_found`, and
  `defaults_storage_error`;
- stack-budget coverage for the real candidate CLI/promotion call chain.

`config slots` now reports load/degraded state, legacy cleanup status and
uncertain-commit counters. A storage error may still start with sanitized defaults
to avoid a boot loop, but it is explicitly reported as degraded and must not be
confused with a genuinely empty first boot.

## real-lifecycle and recovery hardening

Empty, legacy, current, corrupt and transiently unreadable NVS have explicit
recovery semantics:

- explicit distinction between genuinely empty NVS and present-but-corrupt active keys;
- a degraded-storage write latch after boot read errors; active/candidate mutations
  stay blocked until an authoritative config is read **and successfully applied**
  to hardware/runtime; read-only `config verify` cannot clear that latch;
- per-slot commit markers, while retaining read compatibility with v1 slots;
- automatic compatible v1/v2/v3-to-v4 migration on boot;
- content-addressed no-op saves that do not write flash, change generation or stale
  a candidate;
- candidate freshness based on calibration revision rather than whole-config generation;
- interrupted legacy-migration recovery from the still-valid legacy blob;
- persisted active provenance and terminal promoted-candidate history;
- write blocking for candidate flush/discard/promotion while storage is degraded or
  an authoritative hardware apply is pending; full `config erase` remains the explicit
  destructive recovery operation.

Boot may still use sanitized defaults to avoid a reboot loop after a storage error,
but those defaults cannot overwrite the recoverable NVS generation until a successful
`config load` plus hardware/runtime apply confirms the authoritative state.

## Model and event ownership

Generic saves must not restamp `accelCalQuality.calibrationUptimeMs` or
`gyroCalMeta.tempModelUpdatedUptimeMs`: that would break no-op detection,
provenance and candidate freshness. Three operations have distinct owners:

- **authoritative configuration persistence** saves the sanitized RAM config and never
  snapshots transient stream/FIFO state or stale calibration-runner state back into it;
- **calibration snapshot** copies the currently applied model and existing evidence
  without creating a new calibration event;
- **calibration update** is called only by a successful gyro/accel/temperature/mag
  calibration event and replaces evidence while recording its uptime/sample count.

Component commands are scoped: `cal gyro save` owns gyro/temperature state,
`cal accel save` owns accel state, while `cal save`/guided setup are the explicit
full-calibration snapshots. Candidate staging transfers accel sample/window evidence
only when the runner model exactly matches the staged accel model.

Candidate format v3 binds freshness to a field-wise calibration model revision.
The revision includes applied gyro bias/temperature slope, accel bias/matrix,
mag hard/soft iron, field norm/trust bounds and mag-to-IMU alignment. It excludes
quality scores, sample counts, provenance, event timestamps and runtime policy
such as `tempCompEnabled`. Existing v1 candidates remain generation-bound and v2
candidates retain their historical metadata-inclusive revision, so no NVS erase
is required.

Promotion now applies only calibration fields to the current RAM config, preserves
unsaved runtime/product policy, resets runtime gyro trim and AHRS state, clears the
old magnetic heading reference and resets magnetic yaw correction. It does not
call the general config runtime apply path, reconfigure SPI/LSM/FIFO, or overwrite
output/network policy. Promotion is rejected with `runtime_calibration_diverged`
when RAM contains a third unsaved calibration model that matches neither persisted
active state nor the candidate.

Full `config load/defaults` still applies the complete config transaction, but then
starts a fresh adaptive epoch: runtime gyro trim, AHRS, magnetic heading reference
and yaw correction are reset, and SensorInfo is refreshed. Gyro-validity changes
and magnetometer-driver changes likewise refresh SensorInfo. Magnetic yaw cannot be
enabled without valid accel, magnetic-field and axis-alignment calibrations, and impossible blobs
with a temperature slope but no gyro bias are normalized during sanitize.


## calibration epoch and field-ownership hardening

Calibration models, their evidence, runtime policy and unfinished capture
workspaces have explicit owners and epoch boundaries:

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

### held-out axis evidence

Measured `magToImu` quality staged by the runtime is now derived from held-out
validation windows. Training windows select/refine the rotation; validation
windows must reproduce the same winner and show improvement over active alignment.
The persistent candidate record layout is unchanged. The additional evidence is
collapsed into the existing measured alignment quality and diagnostic runtime
fields rather than adding a schema revision.

## Shared transaction and recovery contracts

- **Contract:** A successful reset/read-back enters `awaiting_progress`. Only a fresh
  drain plus accepted gyro, and orientation when expected, completes recovery.
  Failure to produce that evidence uses the same bounded episode: two FIFO
  attempts, full reinit, exhausted 30 s backoff with continuing probes. An
  additional fault cannot restart this sequence at attempt zero. Every attempt,
  including full reinit, rebases above the highest already assigned timestamp.
  Tap setup remains restricted to full sensor reinit.
- **Contract:** One `FifoCalibrationCaptureSession` owns direct FIFO capture and its
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
- **Contract:** The slot writer is **v4**, with the same payload layout. A named v3
  reader normalizes only previously accepted historical policy: ignored baud
  to the compile-time baud, debug/quaternion flags to the old quaternion
  precedence, and incompatible positive dt override to automatic FIFO dt.
  Full current semantics still apply after normalization. Calibration/frame,
  magnetic model, user mag enable and quality gates are not repaired or erased.
  The inactive v4 slot is written/read back before selector commit and RAM
  apply. v1/v2 migration remains available. Safe boot permits explicitly
  reported read-only volatile migration. Invalid calibration still fails.
- **Contract:** Quaternion corruption is fatal even in stale, invalid or duplicate
  snapshots. Repeated polling of one invalid/stale publication does not spend
  its budget repeatedly. A discontinuity interrupts quaternion comparisons;
  the contaminated window cannot pass. One fresh retry is possible for
  recoverable contamination, never for failed quaternion finiteness, norm or
  continuity. Retry uses a loop and a single 22 s verification deadline, not
  recursive stack frames. Transport diagnostics do not veto sensor acceptance.
- **Contract:** Manual gyro/accel/cal save, temperature save, learned config save,
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
it remains separate from the synchronous CLI owner.
A manual preview predating `save` remains the pre-stage RAM rollback point; the
previous durable model remains authoritative until successful commit.
