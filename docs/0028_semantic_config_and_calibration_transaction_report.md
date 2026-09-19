# 0028 semantic config and calibration transaction report

## Scope and ordering

0028 is the main patch after 0027 and all of its additive letter patches. It
hardens configuration, calibration and reset ownership without changing sensor
ODR defaults, estimator equations, accel/mag physical gates, wire packets or
normal output cadence.

## Defects and evidence

Current-source review found four classes of gap:

- command and background candidates were sanitized immediately before save, so
  invalid input could be transformed into a different accepted value;
- the fixed CLI rejected an overlong line but the quoted-argument parser could
  accept an unclosed quote, nested dispatch could drop excess arguments and
  guided setup silently discarded input after its local buffer filled;
- several save-capable paths applied RAM or hardware before the persistent
  write/read-back proof;
- `factory_reset` and SlimeVR `FRST` did not share one recoverable, scoped
  transaction.

The existing dual-slot config store, candidate store and autonomy journal were
retained. They already provide the old-good generation and promotion rollback
anchors needed by this patch.

## Changed contract

- `validateSemanticConfig()` is the non-mutating admission gate for defaults,
  loaded/migrated data, CLI candidates, setup, manual calibration and autonomy.
  It rejects non-finite values, invalid enums, incompatible IMU/FIFO state,
  singular/ill-conditioned calibration matrices, non-rotation frames, reversed
  gates, invalid rates, inactive-model residue, invalid dependencies and
  non-neutral reserved state.
- `sanitize()` remains only on the explicit legacy single-key/read-only and
  legacy-journal migration paths. Current dual-slot records, runtime previews
  and persistent stores do not repair candidates. The one recognized network
  migration changes only the exact historical default hostname.
- save-capable command paths persist and read back the immutable candidate
  before active RAM apply. Hardware failure restores the old runtime/hardware
  state and attempts to restore the durable old-good config.
- quaternion ingestion has a fallible normalization path. Invalid or near-zero
  quaternions are rejected without replacing the last valid orientation, and
  the rejection count is visible in `ahrs status`.
- command parsing is fixed-capacity and allocation-free, with explicit errors
  for line overflow, too many arguments, unclosed quotes and characters after a
  closing quote. Guided setup consumes and rejects an entire overlong line; a
  truncated suffix cannot become the next answer. Password contents are never
  echoed.
- calibration probation has one wrap-safe absolute deadline. FIFO/magnetic and
  network verdicts are counted separately; a recoverable event discards its
  contaminated window but cannot extend probation forever. Evidence that cannot
  be reconstructed after reboot is not credited as fresh.
- `FactoryResetCoordinator` owns `main`, `network`, `calibration` and `full`
  scopes. Both CLI `factory_reset` and SlimeVR `FRST` use it. A verified 16-byte
  little-endian marker checkpoints each idempotent step; the marker contains no
  raw C++ config object. A confirmed or already-pending reset can complete from
  safe mode without permanently dropping its write protections. Success always
  performs the required reboot.

No existing persistent config schema or SlimeVR wire format is changed. The new
factory-reset marker is a private fixed-width recovery record in its own NVS
namespace.

## Host verification

- changed production translation units compile with the native warning set;
- `python3 tools/run_standalone_tests.py`: 50/50 executables passed;
- native regression covers strict defaults and invalid gates/matrices/enums,
  candidate immutability and old-good retention, exact legacy normalization,
  structurally invalid stored-record rejection, parser failures, invalid
  quaternion retention, scoped reset, interrupted/safe-mode resume,
  corrupt-marker fail-closed behavior and explicit replacement;

CRC-valid current-slot semantic rejection and hardware-before-selector commit
ordering were not exercised by 0028 itself; those gaps are closed and tested by
the additive 0028a patch.
- after the final adversarial revision, focused `config_hardening`,
  `tracker_network_config`, `factory_reset_coordinator` and
  `calibration_autonomy` executables passed against the updated objects;
- `python3 tools/test_0028_semantic_config_transaction_policy.py` locks the
  source-level ownership and aggregate-gate contract.

## Target smoke

Build and flash `BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG`. Save the complete monitor
log. On an already calibrated tracker, first perform the non-destructive part:

1. Let tracking and SlimeVR stabilize for five minutes.
2. Run `health`, `config verify`, `config nvs`, `ahrs status`, `net status` and
   `slime status`.
3. Run `config save`, then `reboot`; repeat the status commands after tracking
   returns.
4. Enter `config print "` and require `# ERR unclosed quote`. Enter a command
   containing more than ten whitespace-separated arguments and require
   `# ERR too many arguments`. Send one line longer than 191 characters and
   require `# ERR line too long`. The following valid `health` command must work.
5. Confirm no credential/password text was echoed and no parser error caused a
   config, calibration, network or tracking transition.

Pass conditions after reboot: `config_valid=yes`, the same desired network and
mag settings, no new FIFO overflow/batch-capacity failure, no sensor recovery
loop, monotonic runtime samples, prepared output remaining fresh, SlimeVR seeing
the tracker, and `invalid_quaternion_rejects=0` during the clean run.

The destructive coordinator smoke is only for a spare tracker or after saving
the calibration needed to restore it. Run `factory_reset calibration confirm`.
The device must reboot once, retain network credentials/connectivity, clear all
sensor/frame calibration and autonomy metadata, and not reboot again. Re-run the
guided calibration before using that tracker. A power-cut-at-each-step reset test
requires an NVS fault-injection build and is not part of ordinary hardware smoke.

## Not verified

- ESP32-C3 target build and the target smoke above have not yet been run in this
  workspace;
- target task stack high-water, flash/RAM size delta and before/after loop timing
  still require the actual PlatformIO toolchain and board;
- physical NVS power-cut injection, low-level NVS write failure and factory-reset
  interruption have host fault-injection coverage only;
- no new motion, magnetic or thermal dataset was required because estimator,
  physical thresholds and sensor cadence were not changed.
