# 0028a transaction recovery and quaternion hot-path report

## Scope and ordering

0028a is additive and applies after 0028. It closes defects found by the
post-patch adversarial review without changing estimator choice, sensor ODR,
accel/mag trust thresholds, SlimeVR packets or output cadence.

## Defects and evidence

- A CRC-valid then-current v2 slot passed only structural validation during active
  resolution. A semantically impossible stored config could therefore reach
  bootstrap, fall back to RAM defaults and leave the store's apply-pending state
  inconsistent.
- Save-capable IMU/FIFO, SPI and mag hardware commands switched the persistent
  selector before hardware apply. A hardware failure then required a second NVS
  write to restore old-good, and failure of that compensating write was not a
  transaction.
- An interrupted full factory reset could fail during boot and normal sensor and
  network startup still continued over partially erased domains. Marker I/O and
  marker corruption were also conflated, and an existing pending scope could be
  reported as a different newly requested scope.
- A transport fault reset calibration's sensor-evidence window. Continuous
  network failure could therefore reject an otherwise healthy sensor candidate
  at the absolute probation deadline.
- Several extreme but finite calibration/FIFO values passed semantic admission;
  `smallGapFactor` was persisted but had no implementation.
- AHRS calculated quaternion norm squared after every gyro propagation and again
  after every sample even though accel correction only runs once per four
  samples and periodic normalization already owns norm proof.

## Changed contract

- 0028a originally made then-current v2 slots require full semantic validation
  while resolving authority. Target evidence later proved that v2 had already
  been deployed under the older compatibility contract. Additive 0028b therefore
  reserves strict admission for new v3 records and treats v1/v2 only as explicit
  sanitize/migrate/revalidate input; the impossible-current-slot protection is
  retained for v3.
- Hardware settings use `prepareAuthoritativeCommit → hardware apply →
  commitPreparedAuthoritative`. Prepare writes and reads back only the inactive
  slot. Hardware rejection aborts it while the selector still names old-good.
  Selector uncertainty keeps the proven working candidate active and requires a
  reboot; it never guesses which generation won.
- Boot failure to resume a reset enters a local-console-only fail-closed mode.
  Sensor, network, autonomy and tap startup are inhibited. Retry is wrap-safe,
  five seconds apart and capped at eight attempts. A corrupt marker is terminal
  until an explicit confirmed replacement; a mismatched pending scope is
  rejected and reported. While this mode is active the dispatcher permits only
  help, factory-reset recovery/replacement and reboot commands, so another
  mutable subsystem cannot be entered over a partially erased configuration.
- Sensor and transport probation evidence are separate. Sensor faults alone
  clear sensor windows. Transport smoke gets its own verified/unverified status
  and counter and cannot veto sensor-calibration acceptance.
- Finite bounds reuse the FIFO's real hardware/queue capacities and existing
  accel, gyro-startup and magnetic calibration limits. A small positive
  timestamp interval below `smallGapFactor` is now an informational flag/counter
  only: it does not reject gyro, accel, output or request recovery.
- External quaternion entry points (`reset`, setters and yaw correction) retain
  full fallible normalization. The 960 Hz propagation path still rejects
  non-finite output every sample; norm validity is checked at periodic
  normalization and independently-derived correction boundaries. A bad accel
  correction falls through an out-of-line full normalization of the gyro
  prediction and does not suppress valid propagation. A finite-but-degenerate
  prediction instead makes AHRS uninitialized until normal gravity admission;
  it is never left publishable.
- The calibration composition helper is explicitly fallible; callers cannot
  silently receive an unchanged active config after a rejected candidate.

## Verification

- `python3 tools/check_all.py --clean --host-only`: PASS. This includes all
  standalone native executables, policy gates and existing replay smokes. After
  the final failure-only quaternion recovery extraction, the affected AHRS
  Production/Slim builds, numerical test, stack measurements and 0028a/0026b
  policies were rerun rather than repeating unrelated gates.
- Full native suite: 50/50 executables passed.
- Focused 0028a source policy passed.
- Existing 0026b and pre-0024 hot-path stack policies passed without raising a
  ceiling.
- Production-profile host proxy (`-O2`): `Ahrs6Dof::update` stack decreased
  from 144 to 128 bytes and its symbol from `0x69f` to `0x697` (8 bytes).
  The out-of-line numeric-corruption recovery path adds 320 bytes to this host
  object but executes only after a rejected quaternion. Explicit
  norm checks drop from two per ordinary sample to one per four-sample accel
  window plus the unchanged configured normalization cadence.
- Slim-profile host proxy (`-Os`): stack and symbol size are exactly unchanged
  at 144 bytes and `0x3c7`; the compact original safety shape is retained for
  that size-oriented profile.
- The long-duration AHRS numerical/invariant test passed in both explicit
  Production (`-O2`) and Slim (`-Os`) builds after the final hot-path revision.
- Native regressions cover semantic current-slot rejection, inactive-slot
  prepare/commit/abort, reset scope mismatch, extreme finite values and
  informational small gaps.

## Target smoke

After the ordinary 0028 smoke, verify these 0028a-specific points on
`BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG`:

1. Record `health`, `fifo status`, `ahrs status`, `mag yaw status` and
   `config nvs` after five minutes of stable tracking.
2. Run an unsaved SPI/FIFO change and restore it, then a saved change followed
   by reboot. The saved setting must survive exactly once, tracking must recover,
   and NVS must not report degraded/uncertain state.
3. Run `fifo reset`; require one bounded recovery to tracking, continuing gyro
   output, no queue overflow and `invalid_quaternion_rejects=0`.
4. Exercise clean stillness and normal gameplay motion for at least five
   minutes. Require no new wrong-way tilt/yaw behavior, no sticky accel/mag
   rejection and no recovery loop.
5. Destructive reset-interruption testing belongs on a spare tracker or a fault
   build. Interrupt a full reset after a checkpoint: the next boot must show
   `factory_reset_recovery_mode=yes`, must not associate Wi-Fi or initialize the
   IMU, and must either complete/reboot once or stop after the bounded retry
   budget with the local console responsive.

Capture target loop/profiler, task stack high-water, firmware size and free-heap
deltas against the same 0028 build. Host object/stack measurements are only
proxies and are not acceptance evidence for ESP32-C3.

## Not verified

- PlatformIO/ESP32-C3 builds are unavailable in this workspace.
- Target flash/RAM delta, task stack high-water, loop latency and current draw
  require the board/toolchain.
- Physical NVS power-cut behavior and reset-recovery mode require a spare device
  or fault-injection build.
- No new motion/magnetic dataset was used because no estimator or physical
  fusion threshold changed.
