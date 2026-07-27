# 0023ga axis alignment stack hardening

## Confirmed defect

Scenario: `python3 tools/check_all.py --clean --require-pio` on Windows/MSYS2 compiled `tracker_setup_commands.cpp` with stack-usage reporting and failed the `0023g` policy:

```text
0023g stack budget exceeded: ... setupRunAxisAlignment(...) uses 1056 bytes, limit 1024
# FAIL 0023g magnetic coverage reservoir policy (exit=1)
```

Severity: build/acceptance blocker. Runtime behavior was not yet proven unsafe, but a guided-calibration function exceeded the project's per-function stack ceiling and could not be accepted by the cross-ABI gate.

Source: `setupRunAxisAlignment()` kept `SetupMagAxisAutoResult`, `SetupMagAxisDynamicResult`, extensive reporting state, the 48-byte manual line buffer and token array in one compiler-visible function. GCC stack layout differed by ABI: Linux reported 880 bytes while Windows/MSYS2 reported 1056 bytes.

Why the existing test missed it: `0023g` was developed and measured with the Linux compiler. Its 1024-byte ceiling had only 144 bytes of Linux margin, which was insufficient for the larger MSYS2 ABI frame. The policy detected the defect correctly on the user's machine; the implementation lacked cross-ABI margin.

## Fix

`setupRunAxisAlignment()` is now a small no-inline orchestrator. Separate no-inline phases own:

- dynamic dataset solve and reporting;
- optional static cross-check of a valid dynamic result;
- static fallback solve and reporting;
- manual mapping input parsing;
- static and dynamic solver workspaces.

No result object or manual input buffer is moved to heap or persistent configuration. The change only controls lifetime and compiler inlining; magnetic reservoir content, solver thresholds, train/validation partitions, matrix application and transaction behavior are unchanged.

## Stack result

Linux GCC `-std=c++20 -O2 -fstack-usage`:

```text
before setupRunAxisAlignment:                 880 bytes
after  setupRunAxisAlignment:                 144 bytes
setupTryApplyDynamicAxisAlignment:            256 bytes
setupTryApplyStaticAxisAlignment:             160 bytes
setupPrintStaticAxisCrossCheck:               128 bytes
setupPromptManualAxisMapping:                 208 bytes
setupAutoSolveMagAxisDynamic:                  320 bytes
setupAutoSolveMagAxis:                         432 bytes
```

The updated `0023g` policy limits the orchestrator to 512 bytes and every isolated phase to at most 768 bytes. The dedicated `0023ga` policy verifies the no-inline boundaries and reruns the full `0023g` policy.

Host `size` direction for `tracker_setup_commands.cpp` at `-O2`:

```text
before: text=81198 data=2424 bss=7420
after:  text=81842 data=2424 bss=7420
delta:  text=+644  data=0    bss=0
```

The increase is confined to guided setup code and explicit call boundaries; no production hot-path state or RAM was added.

## Regression analysis

- No heap allocation was introduced.
- No additional NVS writes were introduced.
- No collector, reservoir, fit, train/validation or coverage behavior changed.
- No FIFO, ODR, SPI, output, Wi-Fi or SlimeVR configuration changed.
- Manual mapping still rejects blank input and extra tokens.
- Dynamic success still receives the static inclination cross-check when available.
- Dynamic failure still falls back to the static solver and then to explicit manual input.
- Config schema remains 2 and candidate format remains 3.

## Verification completed

- GCC `-O2 -fstack-usage` stack policy: PASS.
- `test_calibration_0023g_policy.py`: PASS.
- `test_calibration_0023ga_policy.py`: PASS.
- all build/check/storage/autonomy/calibration policies through `0023ga`: PASS.
- `test_mag_calibration`: PASS.
- `test_mag_heading_reliability`: PASS.
- source-filter/profile/documentation validators: PASS.
- ASan/UBSan compile-only composition for `tracker_setup_commands.cpp`: PASS.
- `git diff --check`: PASS.

The complete standalone matrix was started, compiled all project and compile-only sources including `tracker_setup_commands.cpp`, and then exceeded the execution window while linking the wider unchanged test suite. The two magnetic regression executables were linked and run separately. PlatformIO is not available in the assistant environment.

## Required user gate

Run:

```bash
python3 tools/check_all.py --clean --require-pio
```

The Windows/MSYS2 output should show both:

```text
# calibration_0023g_policy: PASS
# calibration_0023ga_policy: PASS
```
