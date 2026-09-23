# DEV-05 implementation and review

Additive after DEV-04b on the supplied source through firmware 0028d. The working
Git history was reconstructed from uploaded bytes; local synthetic commits must
not be presented as the user's firmware source identity. User Windows identity
last reported: `f146d16d6036ee673669821432fc2cd86b5a3fe2` clean. Actual patch application
on that checkout is a user acceptance check, not something tested remotely here.

## Changed contract

- Independent test-only double rotation oracle (including matrix cross-check).
- Existing core-math accuracy metric retains small-angle resolution; no loosened
  threshold, production quaternion math or estimator replacement.
- 24 deterministic scenarios execute actual AHRS/heading/yaw controller modules.
- New exporter/comparator preserves partial/open-contract distinctions, rejects
  incomplete or incompatible evidence, and reports improvements and regressions
  individually. It does not claim real-device accuracy from synthetic inputs.
- Existing native runner adds pre/post content fingerprints only when the scenario
  test is selected. Existing flags, deadlines, lock, sanitizer probes and cleanup
  remain owned by that runner. No parallel runner or persistent build cache.
- New Python evidence tests are selectable through the existing check_all registry.
- Session workflow, scenario contract, observed gaps and feature priorities are
  documented in [DEV-05 guide](dev05_algorithm_accuracy.md).

Firmware `src/`, PlatformIO pins/partitions, packet modes and production build
flags are unchanged. Test-only allocations, trigonometry and JSON output never
enter the tracker image. Host timings are not interpreted as target performance.

## Validation here

Linux x86-64 GCC 13.3.0:

- Native selected tests (before the final extra tilt-step/end-state assertions): math reference, algorithm scenarios, existing core AHRS:
  3/3 PASS (`native-hh21mkzo`). New scenario execution ~0.17 s on this host; not a
  performance budget or board timing claim.
- The same selection with ASan/UBSan: 3/3 PASS (`native-0i0p7tnv`, final revision), including real
  clean/fault capability probes. Separate LSan and target tests were not rerun.
- Six focused tooling groups: DEV-05 evidence, DEV-04 contracts, DEV-03 runners,
  standalone runner policy, aggregation policy, quality-gate runtime: PASS
  (`check-all-wshtn9sz`). DEV-05 includes 11 Python tests with parameterized
  malformed/missing/duplicate/nonfinite inputs, coverage, mismatched config/build/
  harness, mixed improvement/regression, atomic write failure and log preservation.
- Export of the actual final ASan/UBSan native report: 24 scenarios, five explicitly OPEN contracts.
  Same-report comparison: no measurable regression, all five gaps retained.
  Strict roadmap comparison is expected to fail on those five current gaps.
- Three controlled mutations compiled only into temporary copies: wrong gyro
  multiplication order, wrong accel correction sign and bypassed mag age check.
  All three produced real scenario failures, each executable exit 1. Their logs
  are negative-test evidence, not successful firmware acceptance.
- Documentation and additive patch application are checked during packaging.

During implementation an initial magnetic test used a field-yaw reference without
this project's north = field + pi convention. It failed; the test reference was
corrected against current source. No production gate was weakened. A subsequent
review corrected the invalid-accel scenario truth to include the gyro step that
should have occurred, so implementing propagation will improve rather than
artificially worsen that metric.

## Open firmware contracts (not introduced or fixed here)

At the module entry points exercised, current results are:

| Probe | Observation | Owner / future work |
| --- | --- | --- |
| Clean gravity after 90 degree tilt error | Error remains 90 degrees after 30 s | Ahrs6Dof / 0029 |
| Invalid accel with valid gyro | Update rejected, expected gyro step missing | Ahrs6Dof / 0029 |
| One second missing accel after rest | Accel norm mean changes from 1 toward zero | Accel statistics / 0029 |
| 60 Hz yaw callback, 16 phase offsets | 12 total accel updates lost over the trials | Accumulator / yaw entry point / 0029 |
| 180 degree yaw error with clean controller inputs | No correction during 30 s | MagYawCorrectionController / 0030 |

These observations do not establish behavior of every full firmware recovery path.
No defect ID is closed and no broad tracking-quality improvement is claimed.
`--require-roadmap` is the hard gate for claiming these requirements are satisfied.

## Limits and next acceptance

Windows GCC 16.2 integration and actual user WSL execution of DEV-05 remain to be
run after patch application. Current Server, HIL/debugger, real replay golden,
ESP32 timing/size and firmware release are not certified by this DEV patch.
Source/config fingerprints detect ordinary inconsistency, not hostile report
forgery or all external toolchain changes. A single seeded noise scenario does
not support a statistical accuracy claim; frequency and noise sweeps are scoped
as optional extensions in the guide.

DEV-04 evidence update: the user supplied WSL `check-all-4cddiv3j` (2/2 focused
PASS) and `native-n2n0s3wm` (AHRS 1/1 PASS, none, 240/30 s), plus Windows doctor
`host-at_lvptl` and native `native-4w6_26ns` (1/1 PASS, GCC 16.2). WSL snippets
contain no SHA, so they are not automatically attributed to the Windows commit.
DEV-04 implementation and local workflow acceptance are closed at that scope.

## Local acceptance update, 2026-09-23

The [local verification record](dev05_local_verification_2026-09-23.md) supersedes
the pending Windows/WSL execution above for clean source
`4ac5c0c171913060582cd93d9cef2fd6665fae86`, including DEV-05a. Windows GCC 16.2.0
native and WSL GCC 13.3.0 ASan/UBSan and separate LSan each passed 54/54 executables.
The DEV-05 Python evidence checks, five replay checks and five firmware profiles
passed. Full Windows check_all completed with exit 1: three stack-budget policies
remain FAIL. Local DEV-05 host acceptance is recorded at that scope; the full
project is not accepted. All five OPEN 0029/0030 contracts and the historical
hardware/accuracy/release limits remain. The new record links complete summaries,
raw logs, source/fixture hashes and the separately retained sandbox failure.
