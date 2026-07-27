# Safe background calibration autonomy (0023/0023a)

`0023_safe_background_calibration_autonomy` extends the 0022 magnetic
candidate foundation into a fail-closed lifecycle. It does not change IMU ODR,
FIFO configuration, hardware timestamps, AHRS tuning, output rate, Wi-Fi or
SlimeVR protocol behavior.


## 0023a hardening

`0023a_calibration_autonomy_and_setup_hardening` keeps config schema 2 and
candidate format 3, but strengthens the lifecycle after the 0023 audit:

- a proposal is bound to the active calibration revision and sensor signature
  when the proposal is created, not when it is finally staged;
- every setup/manual calibration command acquires synchronous ownership,
  finishes any provisional rollback first, removes only autonomy-owned
  candidates, and invalidates 0022/0023 evidence before collecting new data;
- promotion, accept and rollback are idempotent write-ahead stages. Reboot
  between the inactive-slot write, journal, selector commit, probation,
  candidate cleanup or journal cleanup resumes deterministically;
- rollback restores the exact previous committed slot and generation rather
  than creating a derived replacement generation;
- motion light sleep is blocked while a calibration transaction, manual owner,
  proposal or probation is active;
- absent-candidate NVS probing is throttled.

The guided setup remains user-friendly. It still uses the same six ordinary
case faces; each face automatically collects a short held-out validation tail.
The rest-gyro stage requires one continuous stationary window plus a held-out
window, and the temperature fit uses leave-one-temperature-bin-out validation.
After enabling tracking, `setup calibration` runs a final coherent-output check
while the tracker rests on any stable ordinary face. No diagonal or precise
angle is required.

## Ownership and supported models

- `magToImu`: three physically separated, internally train/validation-proven
  0022 solve sessions before an owned background candidate may be staged.
- gyro residual bias: three independent stationary sessions at a narrow
  temperature range, strict per-axis non-regression and held-out probation.
- gyro temperature model: at least eight independent stationary sessions,
  at least 4 C coverage, alternating train/validation checks and bounded slope.
- accelerometer: only after strict natural six-face coverage and full 3x3 fit;
  probation additionally requires fresh evidence spanning all three axes.
- hard/soft-iron and `sensorToDevice` are not autonomously promoted by 0023.

Manual and guided-setup candidates always own the shared candidate slot.
Autonomy never overwrites or deletes them.

## Transaction

The lifecycle is:

```text
observe -> independent confirmation -> RAM candidate -> deferred persistence
-> no-force comparison -> write-ahead journal -> atomic promotion
-> probation -> accept/cleanup or calibration-only rollback
```

The journal is stored in two CRC-protected NVS records under `cal_auto` and
contains the previous calibration payload. Config schema remains 2 and
candidate format remains 3; no NVS migration or erase is required.

During probation a write barrier protects the rollback model. A no-op config
save is still allowed, but a changed ordinary save returns `ApplyPending` until
accept or rollback. Power loss before selector commit, after selector commit,
during probation, accept cleanup or rollback is recovered from the journal.

All solver, candidate and NVS work is deferred behind the same FIFO/output
admission gate used by 0022. The approximately 960 Hz sample path only updates
bounded RAM evidence.

## CLI

```text
cal autonomy status
cal autonomy 0022 on|off [save]
cal autonomy 0023 on|off [save]
cal autonomy rollback
cal autonomy reset
cal autonomy clear_rejections
cal erase_all confirm
setup verify
```

Without `save`, enable changes last until reboot. `0022 off` disables only
background axis-alignment learning; it does not disable the magnetometer or yaw
correction. `0023 off` first rolls back any provisional active calibration.

## Performance diagnostics

After `perf on`, `perf status` includes:

```text
calibration_0022
calibration_0023
```

`calibration_0022` covers deferred alignment solve/storage work.
`calibration_0023` covers the 4 Hz coordinator service, candidate lifecycle,
journal, probation and rollback work. Normal per-sample RAM evidence collection
remains part of the existing FIFO/sample pipeline timing.


## Complete calibration erase

`cal erase_all confirm` removes both active calibration slots, the selector,
the shared candidate, autonomy rollback journal and rejected-candidate memory.
It then saves a clean calibrationless config while preserving ordinary product
policy such as Wi-Fi/output settings, magnetometer driver policy and whether
temperature learning is enabled. The explicit `confirm` token is mandatory.

## 0023b stack and interruption hardening

Accel proposal selection uses compact indices into the bounded session ring for
both fit and held-out windows. It does not copy complete `Session` objects into
the task stack and does not add permanent controller RAM for a second six-face
workspace. The fit and validation datasets remain independent.

The transaction remains interruption-safe at every persistent boundary. A
reboot during accept or rollback cleanup reloads the two-slot CRC journal and
continues exactly one idempotent stage at a time. The old authoritative slot is
protected until rollback/accept cleanup has removed the autonomy candidate and
cleared the journal.

Project test runners aggregate failures. One stack-policy or PlatformIO error
no longer hides later policy, replay or environment failures; the final report
lists everything that failed in the same invocation.


## 0023c upgrade and destructive recovery

Journal v1 records written by the first 0023 build are read only for a
conservative rollback to their saved previous calibration payload. They are
never accepted as v2 transactions. After rollback the obsolete journal and its
owned background candidate are removed.

`cal erase_all confirm` is deliberately callable from `suspended_storage`. It
does not parse or trust a damaged journal. Before touching config slots the command writes a verified erase-recovery marker containing the calibrationless config and autonomy preferences. It then erases calibration slots/candidates, writes the clean authoritative config, clears journal/rejection metadata, and removes the marker last. A power loss at any stage is completed idempotently at boot, preserving ordinary product policy and preventing resurrection or silent acceptance of a provisional calibration.

## 0023d passive storage suspension and light sleep

A fail-closed `suspended_storage` state with no valid in-memory transaction
journal is passive. It disables autonomous promotion and preserves the damaged
or obsolete persistent record for explicit recovery, but it does not perform
calibration work and therefore no longer prevents motion-triggered light sleep.
This is important for an unattended tracker: a calibration metadata fault must
not turn into permanent battery drain.

Light sleep is still blocked while any state can mutate calibration or requires
fresh evidence: manual/setup ownership, a RAM proposal, candidate persistence,
promotion, a valid durable journal, probation, accept cleanup or rollback. If a
storage failure happened during a real transaction, the valid journal remains
the authoritative blocker until recovery completes. Light sleep preserves RAM
and never clears or accepts the suspended record.

`cal autonomy status` reports:

```text
autonomy_motion_sleep_blocked=yes|no
autonomy_motion_sleep_block_reason=none|manual_calibration|durable_transaction|proposal_pending|candidate_persistence|promotion|probation|accept_cleanup|rollback
```


## 0023e setup prompt, gyro quality and cold hot path

Blocking guided setup commands can run through the bounded Wi-Fi remote console.
The setup prompt and periodic calibration progress explicitly flush one bounded
non-blocking output chunk while the normal app loop is occupied by the command.
The prompt is therefore visible before the command starts waiting for Enter; no
second newline is required to reveal already-buffered output. Serial behavior is
unchanged.

The initial gyro calibration still requires one continuous stationary fit window
and an independent held-out window. Raw 960 Hz standard deviation is retained as
a hard vibration/motion ceiling, but final acceptance is based on the uncertainty
of the long-window mean and agreement of the held-out mean. This avoids rejecting
a precise bias estimate merely because an otherwise healthy high-rate sensor or
surface has more than 0.20 dps sample-to-sample noise. Large vibration, poor mean
precision, validation disagreement, accel instability and temperature span still
reject the result.

When both autonomy waves are disabled and no durable transaction exists, the IMU
pipeline performs only one cold boolean gate: it does not hash calibration state,
accumulate learner statistics or enter the controller. The normal loop also skips
the autonomy service before reading `millis()` or evaluating NVS. Passive
`suspended_storage` remains fail-closed for promotion but has no recurring service
or IMU hot-path work. `cal autonomy status` exposes both gates as:

```text
autonomy_imu_hotpath_enabled=yes|no
autonomy_deferred_service_required=yes|no
```

With `0022 off` and `0023 off`, both values must be `no` unless a real durable
promotion/probation/rollback transaction is being recovered. Config mutations
(`load`, `defaults`, `save`, `erase`, SPI and FIFO changes) take the same manual
ownership as calibration commands. Applying a replacement config explicitly
invalidates learner evidence; calibration revision/signature hashing is not
performed in the IMU sample path.

## 0023f full guided-setup reliability

The dedicated setup temperature capture no longer judges a 960 Hz sensor by an
unrealistically low per-sample noise threshold. Each short contiguous window is
accepted only when its mean is statistically precise, raw vibration stays below
a hard ceiling, accel/rest quality is valid, temperature is coherent, and the
window agrees with the first and previous accepted windows under a bounded
thermal-slope envelope. The final model still requires at least four temperature
bins, at least 3 C of range, robust fitting and leave-one-bin-out validation.

Temperature capture fails early with detailed rejection reasons when no valid
progress is possible, supports `q` + Enter cancellation, and stops on a real
60-second relative plateau only after sufficient accepted coverage. Its fit
workspace is setup-only static storage and leave-one-out validation no longer
copies all 48 bins onto loopTask's stack.

All guided confirmation prompts accept `q` + Enter as an explicit abort. CRLF is
drained between magnetic rounds, and aborting the optional mag-axis motion now
rolls the setup transaction back instead of falling through into a manual axis
prompt. Calibration readiness is independent from Wi-Fi/server availability and
from the current temperature merely being outside the calibrated range; those
remain runtime confidence or product-connectivity diagnostics.
