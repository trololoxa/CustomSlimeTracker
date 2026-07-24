# Calibration storage and candidate framework

Patch 0021 replaces the single `tracker/cfg` blob with a power-loss-safe active
configuration store and a separate calibration-candidate store. It does not
change the tracking algorithm or collect background calibration by itself.

## Active dual-slot layout

Core IMU/calibration config uses the NVS namespace `tracker` and these keys:

```text
cfg_a   full active-slot record A
cfg_b   full active-slot record B
cfg_s   CRC-protected active selector
cfg_ac  slot-A commit marker (storage v2)
cfg_bc  slot-B commit marker (storage v2)
cfg     legacy single-blob key, migration input only
```

Every slot contains:

- storage magic/version/size;
- monotonic generation;
- calibration-relevant sensor signature;
- active quality summary and calibration provenance;
- a complete validated `TrackerConfigBlob`;
- a record CRC covering header, signature, quality and payload.

Storage-v2 slots also require a matching CRC-protected commit marker. Slots written
by clean 0021/0021a/0021b builds remain readable as legacy-committed v1 records,
but the first subsequent real save upgrades the active store to the v2 marker
lifecycle without changing calibration values or requiring an NVS erase.

A save always removes the target slot's old commit marker, writes the inactive
slot, reads it back byte-for-byte and validates its CRC/signature/payload. Only
then is `cfg_s` rewritten and the matching slot commit marker written. The
previous selected slot remains authoritative throughout. An interrupted first
save cannot become active merely because one complete-looking slot exists.

If the selector points to a corrupt slot, boot falls back only to a slot with
commit authority: either a matching v2 marker or a compatible v1 legacy record.
A newer non-selected v2 slot without a marker is an uncommitted prepared write
and is never promoted by fallback. With a missing selector, committed v2 markers
allow the newest committed generation to be recovered; two legacy v1 slots use
the conservative older-generation rule. Selector/marker repair is best-effort
and never overrides a valid authoritative payload.

## Legacy migration

When no committed dual-slot generation is available, the loader checks the old
`cfg` blob. A valid legacy blob is written to slot A, read back, selected, marked
committed, and only then removed. If a previous migration stopped after writing an
uncommitted v2 artifact, the artifact is cleared and migration safely retries from
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

In Debug and Production, a candidate is first staged in RAM. Staging never
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

0021 provides storage, comparison, wear control, diagnostics and promotion. It
does not automatically learn gyro, accel, magnetometer or alignment candidates.
Those collectors and fit policies belong to 0022 and 0023.

## 0021a stack-safety hotfix

The first 0021 boot exposed a migration-only stack overflow on ESP32-C3. The
old path nested `load -> resolveActive -> migrateLegacy -> saveInternal ->
resolveActive` while several 756-876 byte records were also local stack
objects. 0021a moves storage scratch records to bounded heap workspaces,
performs migration only after the initial resolver has returned, calculates
record CRCs without a candidate-sized stack copy, and probes absent NVS keys
with `Preferences::isKey()` before reading their blob length.

`tools/test_calibration_storage_stack_policy.py` compiles the storage units with
`-fstack-usage`, rejects recursive migration, and keeps every storage control
frame below the declared host-side stack budget.


## 0021b transaction and lifecycle hardening

The post-0021a audit found several cases where a technically valid NVS record
could violate the intended lifecycle contract. 0021b adds:

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

## 0021c real-lifecycle and recovery hardening

Scenario testing across empty, legacy, v1, current, corrupted and transiently
unreadable NVS found several gaps that method-level tests did not expose. 0021c
adds:

- explicit distinction between genuinely empty NVS and present-but-corrupt active keys;
- a degraded-storage write latch after boot read errors; active/candidate mutations
  stay blocked until an authoritative config is read **and successfully applied**
  to hardware/runtime; read-only `config verify` cannot clear that latch;
- per-slot v2 commit markers, while retaining read compatibility with v1 slots;
- automatic v1-to-v2 upgrade on the next real save;
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

## 0021d model/event integration hotfix

Hardware acceptance of 0021c exposed that generic runtime capture re-stamped
`accelCalQuality.calibrationUptimeMs` and
`gyroCalMeta.tempModelUpdatedUptimeMs` on every `config save`. That defeated
content-addressed no-op saves, changed CRC/generation, discarded active
provenance and could make a candidate stale without any calibration change.

0021d separates three contracts:

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


## 0021e calibration epoch and field-ownership hardening

0021e finishes the patch-21 integration audit without changing the persisted blob
layout. Calibration models, their evidence, runtime policy and unfinished capture
workspaces now have explicit owners and epoch boundaries:

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
