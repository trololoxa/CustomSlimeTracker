# DEV-05a: observation-contract corrections

Additive after the delivered DEV-05. This changes host measurement/test code and
documentation only. Firmware sources, trust gates, ODR, target packages, packet
modes and persistence are unchanged. Local Git history is reconstructed; patch
application on the user's Windows checkout is not verified remotely.

## Defects and changes

1. The old tilt probe replaced recovery time after a second clean window. It also
   estimated window onset as the last time minus 250 ms. `FirstSampleDwell` now
   preserves the actual first sample timestamp of the first complete 240-sample
   window. Interrupted candidates restart; later episodes cannot overwrite it.
   Duplicate/backward observation times fail explicitly. Counts are bounded.
   Endpoint tilt remains checked separately; no claim of permanent stability.
2. Invalid-accel gyro continuity used only the return value of `update()`. The
   probe now requires update success, a finite near-unit quaternion matching the
   independent expected gyro step, and advancement to the exact integrated sample
   timestamp. A frozen/wrong-direction/doubled step or timestamp-only success
   cannot close the contract. Quaternion sign equivalence remains valid.

The numerical step tolerance is 0.00001 degree for this particular 1 ms probe
(expected step about 0.01146 degree), not a general accuracy target or firmware
gate. Raw quaternion validity is checked before the angular metric normalizes it.

Both contracts have one test-only owner in `dev05/observation_contracts.hpp`.
`test_dev05_observation_contracts` exercises repeated/interrupted recovery,
fractional 960 Hz timestamps, crossing 2^32, the count boundary, false success,
timestamp failures, sign equivalence, bad direction, doubled motion and invalid
quaternions. Existing native discovery includes it without a parallel runner.

## Verification

- Linux GCC 13.3, ASan/UBSan: 3/3 selected native executables passed, including
  observation contracts, algorithm scenarios and math reference; report
  `native-flb8p3wo`. Capability clean/fault probes succeeded.
- Three controlled helper mutations all compiled and then failed the new test as
  intended: overwritten first recovery, bool-only propagation, omitted timestamp
  verification. Report `dev05a-sensitivity-rkqxzyl3`; child exit 1 is expected
  negative-test evidence, not successful execution of a broken firmware.
- Actual native export: 24 scenarios, the same five explicitly OPEN module
  contracts. Tilt-25 first window begins at 1408.333 ms; invalid-accel continuity
  correctly remains false with 0.0114591560734 degree missing-step error.
- Same-report comparison: no measurable changes, five OPEN retained. Old DEV-05
  versus DEV-05a is rejected for incompatible harness fingerprint, as required.
- Focused Python evidence/runner policies, documentation and clean additive
  application are validated during packaging; evidence is included in the archive.

No full gate, firmware build, target timing, hardware, standalone LSan, or user
Windows/WSL execution of this revision is claimed. Host tests do not establish
absolute device accuracy. Existing 0029/0030 gaps are not fixed by this patch.

## User acceptance and baseline

Run from the project root using variables in [dev_session.md](dev_session.md):

```powershell
& $TrackerPython tools/check_all.py --check test_dev05_algorithm_accuracy --check test_run_standalone_tests_policy --check validate_documentation
& $TrackerPython tools/run_standalone_tests.py --cxx $TrackerCxx --test test_dev05_observation_contracts --test test_dev05_algorithm_scenarios --test test_dev05_math_reference --build-timeout-s 240 --test-timeout-s 60
```

Export the printed native report using the [existing instructions](dev05_algorithm_accuracy.md).
Both sides of an AHRS implementation comparison must use the same corrected
harness. Re-exporting an old raw log does not upgrade its measurements; rerun both
firmware versions with DEV-05a. Never remove the hash check to mix old/new reports.

The [mathematical verification plan](dev05_math_verification_plan_ru.md) records
the agreed next scope. Its additional scenarios are planned, not implemented or
certified by DEV-05a. This small correction precedes that expansion.

## Local acceptance update, 2026-09-23

Local DEV-05a observation-contract/evidence acceptance is now recorded for clean
`4ac5c0c171913060582cd93d9cef2fd6665fae86`; see the
[complete local verification record](dev05_local_verification_2026-09-23.md).
All three DEV-05/DEV-05a executables ran in each full 54/54 Windows native, WSL
ASan/UBSan and WSL LSan suite. The Python evidence group passed 11 tests. Exports
of these actual reports each retain 24 scenarios and the same five OPEN contracts.
Five firmware profiles built, but full Windows check_all exited 1 on three
stack-budget policies. This closes pending local host execution, not overall
project/release acceptance, hardware, target timing, absolute accuracy, 0029/0030
or the additional mathematical-plan scenarios. The original verification above
is retained as historical evidence with its original scope.
