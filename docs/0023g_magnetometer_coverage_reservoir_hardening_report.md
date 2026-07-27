# 0023g magnetometer coverage reservoir hardening

## Source of truth

- predecessor: `Tracker_firmware.zip`
- predecessor SHA-256: `4b42bdb973124a0f565b5d41c18d776f7adf6d949f7a838cffd1afa3f2e93ad6`
- reviewed input patch: `0023h_magnetometer_coverage_reservoir_hardening.patch`
- reviewed input patch SHA-256: `7d876b01d40cf86bad857a1b32e440c050631e2e68a4ebfac1040360b8961c29`
- final patch identity: `0023g_magnetometer_coverage_reservoir_hardening`

There is no `0023g` predecessor. The supplied `0023h` is therefore renamed and rebuilt as `0023g` rather than preserved as a later suffix.

## Reported failure

The guided magnetic stage reached:

```text
axis_intervals=160
dynamic_axis_dropped=10361
collected_samples=16206
calibration_valid=no
axis_alignment_valid=no
```

The tracker had continued receiving useful motion and magnetic data, but the bounded dynamic-axis dataset stopped changing after the first 160 accepted intervals.

## Why the supplied 0023h did not apply

The patch references:

```text
tools/test_guided_setup_policy.py
```

That file does not exist in the exact predecessor. The current guided-setup policy is:

```text
tools/test_calibration_0023f_policy.py
```

Consequently:

```text
git apply --check
→ error: tools/test_guided_setup_policy.py: No such file or directory
```

and `patch --dry-run -p1` reaches the same missing-file failure after the code hunks.

## Why the supplied code change was not sufficient

The original patch improved the hard/soft-iron sample buffer, but its dynamic-axis change only renamed the old counter from `dynamic_axis_dropped` to a capacity diagnostic. The collector still used a first-N array and returned immediately after 160 intervals. Late rotation around a missing axis therefore remained invisible to the solver. The log became less alarming, but the physical failure remained.

The original hard/soft diagnostics also mixed two different populations: full-capture extrema were printed and used as coverage evidence while the ellipsoid solver fitted the bounded reservoir. A discarded extreme could therefore make coverage look adequate even when the actual fit set was inadequate.

## Implemented corrections

### Hard/soft-iron fit reservoir

- Replaced the recent-sample slot permutation with deterministic Algorithm-R admission over all accepted samples.
- Kept fixed memory and no heap allocation.
- Added accepted/stored/replacement/skip diagnostics.
- Added explicit diagnostics for both the complete capture and the exact bounded fit set.
- Made box-coverage gates use the actual fit-set spans, not extrema that may have been discarded.
- Corrected displayed inlier denominators to use stored fit samples.

### Guided mag-axis reservoir

- Replaced the frozen first-160 array with a fixed-memory stratified reservoir.
- Six strata are used: dominant gyro axis X/Y/Z × training/validation window parity.
- Each stratum uses deterministic replacement after reaching its fixed quota.
- A 75 ms admission cadence prevents highly correlated consecutive intervals from dominating the reservoir.
- Late X/Y/Z evidence remains eligible for replacement throughout the entire guided motion stage.
- Readiness now requires at least two axes with meaningful evidence in both training and validation partitions, rather than only aggregate two-axis excitation.
- Independent-window and excitation results are cached incrementally; the blocking setup loop does not perform quadratic rescans.
- Large reservoir reset operations are explicit in-place resets rather than by-value construction of a multi-kilobyte temporary.

New setup diagnostics include:

```text
dynamic_axis_intervals
dynamic_axis_candidates_seen
dynamic_axis_reservoir_active
dynamic_axis_reservoir_replacements
dynamic_axis_reservoir_skipped
dynamic_axis_cadence_skipped
dynamic_axis_windows_stored_seen
dynamic_axis_excited_axes
dynamic_axis_partition_confirmed_axes
dynamic_axis_bucket_counts
```

### Additional guided-calibration audit fixes

- Temperature-model readiness is re-read after a newly committed rest/gyro calibration, because the new gyro model may invalidate the old temperature-model epoch.
- Inconclusive static accel-face magnetic samples no longer suppress the dynamic mag-axis motion stage.
- Hard/soft collection and static mag-axis face observation now have separate ownership. A valid hard/soft model is not needlessly reset or recollected merely because mag-axis alignment is missing.
- A defensive stop remains for any temporary collector that is found active when a valid model is reused.
- `nomag`/`6dof` combined with an explicit axis mapping is rejected as contradictory input.
- Manual axis input rejects trailing fourth or later tokens instead of silently accepting the first three.
- Failure prompts point to `mag_cal_failure_reason` and the exact capture/fit metrics instead of only saying that coverage is missing.

## Compatibility and scope

Unchanged:

```text
persistent config schema = 2
candidate format = 3
SlimeVR protocol = 22
ODR / FIFO / SPI configuration
output rate and packet policy
mag driver policy ownership
NVS erase or migration requirements
```

The patch does not auto-promote hard/soft calibration, change `sensorToDevice`, reconfigure FIFO, or place solver/NVS work into the IMU callback.

## Resource direction

Host `-Os` object comparison for the five directly affected translation units:

```text
predecessor: text=112411, data=2364, bss=7168
0023g:       text=117294, data=2364, bss=7424
delta:       text=+4883, data=0, bss=+256 bytes
```

The guided reservoir object is static and bounded. `MagAxisIntervalReservoir<160>` is 7296 bytes; it replaces the previous 7040-byte interval array plus collector fields, producing the measured net BSS increase rather than adding another full dataset.

Relevant host stack usage at `-O2`:

```text
setupRunMagMotionAndApply       128 bytes
setupRunAxisAlignment           880 bytes
MagCalibrationCollector::compute 2016 bytes
```

The magnetic fit function was already a roughly 2 KiB setup/deferred operation in the predecessor; the new policy fixes its ceiling at 2048 bytes. No 7 KiB reservoir temporary is returned or reset by value.

Actual ESP32-C3 flash/RAM percentages still require PlatformIO on the user machine.

## Validation performed

Passed:

```text
test_mag_calibration
test_mag_heading_reliability
ASan + UBSan: test_mag_calibration
ASan + UBSan: test_mag_heading_reliability
validate_source_filters.py
validate_profile_matrix.py
validate_documentation.py
all build/check/storage/autonomy policies through 0023g
calibration integration policy
mag heading reliability policy
host compile-only: mag calibration, mag axis, mag status/runtime, guided setup
Arduino host composition: tracker_app.cpp with -DARDUINO
```

The new native regressions cover:

- one early full-sphere hard/soft dataset followed by a much longer single-axis tail;
- thousands of sequential X, then Y, then Z dynamic intervals;
- retention of all six axis/partition strata;
- exact cached independent-window accounting;
- reset without a large by-value temporary;
- refusal to call an axis independently confirmed until both train and validation contain sufficient evidence.

The aggregate native runner was attempted with both GCC and Clang. In this environment it exceeded the execution window while compiling the very large pre-existing native matrix; no failure from an affected test was reported before timeout. PlatformIO is not installed here, so `check_all.py --clean --require-pio` remains a user-machine gate.

## Required hardware acceptance

Run:

```text
version
status
perf on
perf reset
setup calibration full
```

During magnetic motion verify:

```text
dynamic_axis_candidates_seen continues increasing
dynamic_axis_reservoir_active=yes after saturation
replacements/skips increase after the bounded set fills
dynamic_axis_excited_axes >= 2
dynamic_axis_partition_confirmed_axes >= 2
both parities are represented in dynamic_axis_bucket_counts
mag_cal_fit_span_xyz has real three-axis coverage
mag_cal_failure_reason=none after apply
```

After completion:

```text
setup status
setup verify
mag status
mag heading
perf status
reboot
setup status
setup verify
```

Acceptance requires valid hard/soft and axis alignment, persistence after reboot, no FIFO overrun/full regression, no repeated tracking recovery, no UDP send failures, and no visible pose hitch during deferred solve/apply.
