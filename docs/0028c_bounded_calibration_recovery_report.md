# 0028c bounded calibration recovery report

## Scope and ordering

0028c is additive and applies after 0028b. It closes the final source-audit
gaps in 0028 without changing AHRS cadence, IMU ODR, accel/mag physical trust
thresholds, SlimeVR wire behavior or storage schema.

## Defects and changed contract

- Gyro and accelerometer FIFO captures previously retried a failed FIFO wait
  forever. Each capture now has an absolute wrap-safe deadline, a bounded
  consecutive-timeout budget, cooperative `q` cancellation and an explicit
  terminal status. One timeout remains recoverable; drain failure remains a
  hard failure.
- Setup verification could observe only valid prepared snapshots because the
  normal copy API hid coherent invalid publications. `copyCoherent()` now lets
  verification count them while ordinary network/status consumers retain the
  existing valid-only `copy()` contract.
- Invalid and stale snapshots are excluded from good evidence. Small bounded
  invalid/stale/seqlock and fallback/AHRS-skip counts are tolerated. FIFO
  discontinuity, sample loss/reorder, recovery and non-finite/near-zero
  quaternions remain hard failures. Excess recoverable contamination gets one
  automatic fresh-window retry; it cannot loop.
- An autonomous candidate rejection used a persisted uptime value without an
  expiry. Rejections now have reason-aware bounded retry delays and only a
  record written in the current boot may suppress current evidence. A reboot
  cannot turn an old uptime into a permanent lock.
- Magnetic heading normalization previously converted a bad quaternion to
  identity. It now uses the existing fail-capable normalization and reports
  `MAG_HEADING_REJECT_QUAT_INVALID`. This replaces the old normalization at mag
  cadence and adds no extra square root.
- Accel face tokens are exact, variance must be finite/non-negative and
  validation thresholds must be finite, ordered and non-zero before divisions.
- A non-zero quality `expectedDtUs` must be compatible (within a deliberately
  broad factor) with the configured gyro BDR. Persisted serial baud is fixed to
  the build-time baud instead of pretending to be runtime-applicable. Debug
  text streaming remains an explicit session action and is deliberately not
  auto-started after boot/config load, because a high-rate text stream can
  starve the diagnostic console.

## Performance and technical-debt review

The capture checks run only in blocking calibration workflows. Setup
verification runs only before candidate acceptance. Autonomous rejection TTL
runs in the slow deferred service. The mag heading change replaces an existing
normalization at approximately mag cadence. The verifier now normalizes each
unique quaternion once and reuses it for continuity, avoiding repeated square
roots. No heap allocation, logger, event bus, NVS transition write, sensor-rate
change or new active estimator was added.

The new cancellation owner is a small RAII scope, so callbacks are restored on
every return path. The valid-only prepared-output API remains unchanged for
existing consumers; the broader coherent API is explicitly named and used only
by verification/tests.

## Host verification

The additive native regressions cover bounded FIFO wait termination,
cooperative cancellation, malformed face tokens, invalid variance/validation
parameters, coherent invalid snapshot visibility, bounded recoverable
verification events, stale exclusion, near-zero/non-finite heading quaternion,
expected-dt compatibility and fixed serial-baud semantics.

Run the single aggregate host gate:

```bash
python tools/check_all.py --clean --host-only
```

Result: `check_all --clean --host-only` PASS. This includes 50/50 standalone
native executables, source/profile/document validation, all 0027/0028 policy
gates, stack/hot-path policies, sanitizer-owned regressions and the 60/600
second replay gates. The aggregate correctly reports
`host-verified, target build not verified`.

## Target smoke

1. Flash 0028 + 0028a + 0028b + 0028c and boot twice. Require the same v3
   generation/calibration after the second boot and normal IMU/FIFO/mag startup.
2. Start a gyro or accel face capture, leave the sensor running and inject one
   brief harmless console/FIFO scheduling delay. Capture must continue.
3. Start another face capture and send `q` followed by Enter. It must finish
   promptly with `capture_status=cancelled`; active calibration and NVS must be
   unchanged.
4. With the IMU deliberately unavailable, start capture. It must terminate
   after the bounded timeout streak with `sensor_unavailable`, not hang.
5. Run setup verification while still. A single fallback timestamp or skipped
   AHRS publication may remain within the printed recoverable budget. Loss,
   timestamp reversal, FIFO recovery or a non-finite quaternion must still
   reject and preserve old-good calibration. If recoverable contamination
   exceeds budget, require exactly one automatic retry and no loop.
6. Enable the local debug stream, reboot and verify it does not auto-start or
   flood the console. Explicitly enabling it in the new session must still work.
7. Run five minutes still plus ordinary motion. Compare FIFO queue age,
   recovery counters, prepared-output age, mag cadence and SlimeVR visibility
   against the 0028b smoke; no new drift or throughput regression is expected.

## Not verified

- ESP32-C3 target build, task stack high-water and flash/RAM delta require the
  user's PlatformIO toolchain.
- Physical FIFO timeout/cancel behavior and the setup retry window require the
  target board and console.
- This patch intentionally does not weaken the magnetic norm/interference gate;
  a field rejected as physically implausible remains rejected.
